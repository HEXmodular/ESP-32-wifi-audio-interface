import { Sequencer } from '@controls/sequencer-control/sequencer';
import React, { useCallback, useEffect, useRef, useState } from 'react';

import { useOscillatorContext } from '@contexts/OscillatorContext';
import { getMIDINoteFrequency } from '@utils/noteFreq';
import { SequencerSelect } from './controls/sequencer-select';

import { SequencerSelectMenuBlock } from './controls/sequencer-select-menu-block';
import { Player } from './mmPlayer';

import './sequencer-magenta.css';

declare global {
    var mm: any;
}

const MAPPING_8 = { 0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7 } as { [key: number]: number };
const MAPPING_4 = { 0: 0, 1: 2, 2: 5, 3: 7 } as { [key: number]: number };

const CONSTANTS = {
    NOTES_PER_OCTAVE: 12,
    WHITE_NOTES_PER_OCTAVE: 7,
    LOWEST_PIANO_KEY_MIDI_NOTE: 21,
    GENIE_CHECKPOINT: 'https://storage.googleapis.com/magentadata/js/checkpoints/piano_genie/model/epiano/stp_iq_auto_contour_dt_166006',
}

interface Prompt {
    readonly promptId: string;
    text: string;
    weight: number;
    cc: number;
    color: string;
  }

// TODO когда сбрасывать модель?
// семена?

const OCTAVES = 2;
const TEMPERATURE = 0.25;
//   const bonusNotes = OCTAVES > 6 ? 4 : 0;  // starts on an A, ends on a C.
const totalNotes = CONSTANTS.NOTES_PER_OCTAVE * OCTAVES; //+ bonusNotes; 
//   const totalWhiteNotes = CONSTANTS.WHITE_NOTES_PER_OCTAVE * OCTAVES //+ (bonusNotes - 1); 
// const keyWhitelist = Array(totalNotes).fill(0).map((x, i) => {
//     // if (OCTAVES > 6) return i;
//     // Starting 3 semitones up on small screens (on a C), and a whole octave up.
//     return i + 3 + CONSTANTS.NOTES_PER_OCTAVE;
// });

const gMajTwoOctaves = [
    22, 24, 26, 27, 29, 31, 33,
    34, 36, 38, 39, 41, 43, 45
];

const keyWhitelist = gMajTwoOctaves;

//   const note = genie.nextFromKeyWhitelist(BUTTON_MAPPING[0], keyWhitelist, TEMPERATURE);
//   const pitch = CONSTANTS.LOWEST_PIANO_KEY_MIDI_NOTE + note;

const INITIAL_ACTIVE_NOTES = Array(8).fill(false).map((_, i) => [0, 2, 5, 7].includes(i));
const INITIAL_SEQUENCE_LENGTH = 16;

const SequencerMagenta: React.FC = () => {
    const BUTTON_MAPPING = MAPPING_4;

    const genieRef = useRef<any | null>(null);
    const sequencersRef = useRef<Sequencer[]>([]);
    const playerRef = useRef<Player | null>(null);

    // State for active notes for each sequencer
    const [activeNotes, setActiveNotes] = useState<boolean[][]>(
        Array(4).fill(null).map(() => [...INITIAL_ACTIVE_NOTES])
    );
    // State for sequence length for each sequencer
    const [sequenceLengths, setSequenceLengths] = useState<number[]>(
        Array(4).fill(INITIAL_SEQUENCE_LENGTH)
    );
    const [isPlayerReady, setIsPlayerReady] = useState(false);
    const [selectedSequencerIndex, setSelectedSequencerIndex] = useState<number>(0);

    // Add menu state
    const [isMenuOpen, setIsMenuOpen] = useState<boolean>(false);
    const [menuData, setMenuData] = useState<{
        activeNotes: boolean[];
        temperature: number;
        steps: string;
    }>({
        activeNotes: INITIAL_ACTIVE_NOTES, // Will be updated based on selectedSequencerIndex
        temperature: 0.5,
        steps: INITIAL_SEQUENCE_LENGTH.toString() // Will be updated based on selectedSequencerIndex
    });

    // State for sequencer values and current step
    const [sequencerState, setSequencerState] = useState<{
        values: boolean[][], // Array of sequences, one per sequencer
        currentStep: number // Global for now, can be per-sequencer if needed
    }>({
        values: Array(4).fill(null).map(() => Array(INITIAL_SEQUENCE_LENGTH).fill(false)),
        currentStep: 0
    });

    const { oscillators, updateOscillator, isLoading: oscillatorsLoading } = useOscillatorContext();

    const updateOscillatorFrequency = (oscillator_id: number, frequency: number) => {
        updateOscillator({ oscillator_id, frequency, amplitude: 1.0 });
    }



    // Initialize PianoGenie and Player
    useEffect(() => {
        if (!mm) return;

        // Initialize PianoGenie
        if (!genieRef.current) {
            const genie = new mm.PianoGenie(CONSTANTS.GENIE_CHECKPOINT);
            genieRef.current = genie;

            console.log('🧞‍♀️ initializing', genie);

            genie.initialize().then(() => {
                console.log('🧞‍♀️ ready!');
            });
        }

        // Initialize Player
        (async () => {
            playerRef.current = await new Player();
            setIsPlayerReady(true);
        })();


        // // Initialize player methods
        // const initPlayer = async () => {
        //     try {
        //         await player.loadSamples();
        //         console.log('Player initialized');
        //         setIsPlayerReady(true);
        //     } catch (err: unknown) {
        //         console.error('Failed to initialize player:', err);
        //     }
        // };

        // initPlayer();

        return () => {
            // if (playerRef.current) {
            //     playerRef.current.stopAll();
            // }
        };
    }, [mm]);

    // Separate effect for sequencer initialization and updates
    useEffect(() => {
        if (!genieRef.current || !playerRef.current || !isPlayerReady) return;

        const handleMasterStepChange = (masterStep: number) => {
            setSequencerState(prev => ({
                ...prev,
                currentStep: masterStep // Update global current step with the master step
            }));

            [0, 1, 2, 3].forEach(sequencerIndex => {
                const currentActiveNotes = activeNotes[sequencerIndex];
                const individualSequenceLength = sequenceLengths[sequencerIndex];
                
                // Calculate effective step for this specific sequencer using modulo
                const effectiveStepForSequencer = masterStep % individualSequenceLength;
                
                const currentSequenceValues = sequencerState.values[sequencerIndex];

                // Ensure currentSequenceValues is valid 
                if (!currentSequenceValues) {
                    return; 
                }
                
                const isActiveStep = currentSequenceValues[effectiveStepForSequencer];

                if (isActiveStep && genieRef.current) {
                    const localActiveNotesMapping = currentActiveNotes
                        .map((isActive, noteIndex) => isActive ? noteIndex : -1)
                        .filter(noteIndex => noteIndex !== -1);

                    if (localActiveNotesMapping.length > 0) {
                        const genieButton = localActiveNotesMapping[0];
                        const mappedGenieButton = BUTTON_MAPPING[genieButton % Object.keys(BUTTON_MAPPING).length];

                        const note = genieRef.current.nextFromKeyWhitelist(
                            mappedGenieButton,
                            keyWhitelist,
                            menuData.temperature
                        );

                        if (note !== undefined) {
                            const pitch = CONSTANTS.LOWEST_PIANO_KEY_MIDI_NOTE + note;
                            console.log(
                                `MasterStep: ${masterStep}, Sequencer: ${sequencerIndex}, EffectiveStep: ${effectiveStepForSequencer}, Note: ${pitch}, Button: ${genieButton}(${mappedGenieButton})`
                            );
                            updateOscillatorFrequency(sequencerIndex, getMIDINoteFrequency(pitch));
                        }
                    }
                }
            });

            // Global Genie reset at the end of the master sequence (INITIAL_SEQUENCE_LENGTH)
            if (genieRef.current && masterStep === INITIAL_SEQUENCE_LENGTH - 1) {
                // console.log(`Global Genie reset at master step ${masterStep}.`);
                // genieRef.current.resetState(); // This resets Genie's state globally.
            }
        };

        if (sequencersRef.current && sequencersRef.current.length > 0 && sequencersRef.current[0]) {
            // If master sequencer instance exists, just update its callback
            sequencersRef.current[0].onStepChange = handleMasterStepChange;
        } else {
            // Initialize the single master sequencer
            const masterSequencer = new Sequencer({
                sequenceLength: INITIAL_SEQUENCE_LENGTH, // Fixed length for the master clock
                timeUnit: 1000, // TODO: make configurable or dynamic if needed
                onStepChange: handleMasterStepChange
            });
            sequencersRef.current = [masterSequencer];
            masterSequencer.start();
        }
        // Cleanup function to stop the sequencer when the component unmounts or dependencies change significantly
        return () => {
            if (sequencersRef.current && sequencersRef.current[0]) {
                sequencersRef.current[0].stop();
                 // sequencersRef.current = []; // Optionally clear the ref
            }
        };
    }, [
        genieRef, // ref itself
        playerRef, // ref itself
        isPlayerReady,
        activeNotes,
        sequenceLengths,
        sequencerState.values, // Note: This might cause re-runs if not careful.
        menuData.temperature,
        keyWhitelist,
        updateOscillator // From useOscillatorContext, assumed stable (useCallback)
        // Removed sequencerState.currentStep as it's set by this effect.
        // Removed selectedSequencerIndex as it's not directly used by master step logic.
    ]);


    const handleValueChange = (sequencerIndex: number, newValues: boolean[]) => {
        setSequencerState(prev => {
            const newSequencerValues = [...prev.values];
            newSequencerValues[sequencerIndex] = newValues;
            return { ...prev, values: newSequencerValues };
        });
    };

    const handleMenuClick = (index: number) => {
        setSelectedSequencerIndex(index);
        setIsMenuOpen(prev => !prev);

        if (!isMenuOpen) { // Means it's about to open
            setMenuData({
                activeNotes: activeNotes[index],
                temperature: menuData.temperature, // Keep global temperature for now
                steps: sequenceLengths[index].toString()
            });
        }
    };

    // TODO: Render 4 SequencerSelect components
    // For now, rendering one and making it use the selectedSequencerIndex

    return (<>
        {isMenuOpen && (
            <SequencerSelectMenuBlock
                activeNotes={menuData.activeNotes}
                temperature={menuData.temperature}
                steps={menuData.steps as "8" | "16"}
                onNotesChange={(newActiveNotesSettings) => {
                    setActiveNotes(prev => {
                        const newNotes = [...prev];
                        newNotes[selectedSequencerIndex] = newActiveNotesSettings;
                        return newNotes;
                    });
                    setMenuData(prevMenuData => ({ ...prevMenuData, activeNotes: newActiveNotesSettings }));
                }}
                onStepsChange={(newStepsValue) => {
                    const newLength = parseInt(newStepsValue);
                    setSequenceLengths(prev => {
                        const newLengths = [...prev];
                        newLengths[selectedSequencerIndex] = newLength;
                        return newLengths;
                    });
                    setMenuData(prevMenuData => ({ ...prevMenuData, steps: newStepsValue }));
                    // Potentially update the master sequencer's length if it's meant to adapt,
                    // or handle varying lengths within handleStepChange.
                    // For now, assuming the master sequencer might have a fixed maximum length or adapts.
                    if (sequencersRef.current[0] && selectedSequencerIndex === 0) { // Example: only adapt if the first sequencer's length changes
                        // sequencersRef.current[0].setSequenceLength(newLength);
                    }
                }}
                onTemperatureChange={(value) => {
                    // Temperature is global for now in menuData
                    setMenuData(prev => ({ ...prev, temperature: value }));
                }}
            />
        )}
        <div className="sequencer-container">
            {!isPlayerReady && <div className="loading">Loading player...</div>}
            {/* Render all four SequencerSelect components */}
            {[0, 1, 2, 3].map(idx => (
                <SequencerSelect
                    key={idx}
                    label={`Voice ${idx + 1}`}
                    rows={activeNotes[idx]?.filter(note => note).length}
                    sequenceLength={sequenceLengths[idx]}
                    currentStep={sequencerState.currentStep} // Assuming global step for now
                    values={sequencerState.values[idx]}
                    onMenuClick={() => handleMenuClick(idx)}
                    onChange={(newValues) => {
                        handleValueChange(idx, newValues as boolean[]);
                    }}
                />
            ))}
            {/* Placeholder for other sequencers - will be added in a later step */}
            {/* {[0,1,2,3].map(idx => (
                 <SequencerSelect
                 key={idx}
                 label={`Voice ${idx + 1}`}
                 rows={activeNotes[idx]?.filter(note => note).length}
                 sequenceLength={sequenceLengths[idx]}
                 currentStep={sequencerState.currentStep} // Assuming global step
                 values={sequencerState.values[idx]}
                 onMenuClick={() => handleMenuClick(idx)}
                 onChange={(newValues) => {
                     handleValueChange(idx, newValues as boolean[]);
                 }}
             />
            ))} */}
        </div>
    </>
    );
};


export default SequencerMagenta;
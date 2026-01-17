import React, { useState, useEffect, useRef } from 'react';
import { SequencerMenuBlock } from '@controls/sequencer-menu-block/sequencer-menu-block';
import { SequencerControl } from '@controls/sequencer-control/sequencer-control';
import { Sequencer } from '@controls/sequencer-control/sequencer';
import ValueControl from '@controls/value-control/value-control';

import { Envelope } from '@audio/envelop';
import type { AudioParamItem } from '@interfaces/AudioParamList';
import { useEffects } from '@contexts/EffectsContext';

import './sequencer-screen.css';

interface SequencerState {
    blockType: string | null;
    parameter: AudioParamItem | null;
    parameterName: string | null;
    isMenuOpen: boolean;
    currentStep: number;
    values: boolean[];
    envelopeParams: {
        attackTime: number;
        peakValue: number;
        decayTime: number;
    };
    durationParams: {
        bars: number;
        sixteenths: number;
    };
}

export const SequencerScreen: React.FC = () => {
    const { audioContext } = useEffects();
    const [bpm, setBpm] = useState(120);
    const [sequencerStates, setSequencerStates] = useState<SequencerState[]>(
        Array(6).fill({
            blockType: null,
            parameter: null,
            parameterName: null,
            isMenuOpen: false,
            currentStep: 0,
            values: Array(8).fill(false),
            envelopeParams: {
                attackTime: 0.2,
                peakValue: 1.0,
                decayTime: 1.2
            },
            durationParams: {
                bars: 1,
                sixteenths: 0
            }
        })
    );

    const sequencersRef = useRef<Sequencer[]>([]);
    const envelopesRef = useRef<Envelope[]>(
        Array(6).fill(null).map(() => new Envelope())
    );

    // Create sequencers when audioContext changes
    useEffect(() => {
        if (!audioContext) return;

        // Cleanup old sequencers
        sequencersRef.current.forEach(sequencer => sequencer.destroy());

        // Create new sequencers
        sequencersRef.current = Array(6).fill(null).map((_, index) => new Sequencer({
            sequenceLength: 8,
            timeUnit: 1000,
            onStepChange: (step) => {
                setSequencerStates(prev => {
                    const newStates = [...prev];
                    newStates[index] = {
                        ...newStates[index],
                        currentStep: step
                    };

                    if (newStates[index].values[step]) {
                        const { blockType, parameter, envelopeParams } = newStates[index];
                        
                        if (blockType && parameter && audioContext) {
                            const envelope = envelopesRef.current[index];
                            envelope.startEnvelope(
                                parameter.audioParam,
                                envelopeParams.attackTime,
                                envelopeParams.peakValue,
                                envelopeParams.decayTime,
                                0.7,
                                audioContext
                            );
                        }
                    }

                    return newStates;
                });
            }
        }));

        // Start all sequencers
        sequencersRef.current.forEach(sequencer => sequencer.start());

        // Cleanup
        return () => {
            sequencersRef.current.forEach(sequencer => sequencer.destroy());
        };
    }, [audioContext]);

    const handleBlockChange = (index: number, blockType: string, parameter: AudioParamItem) => {
        console.log('handleBlockChange', index, blockType, parameter);
        setSequencerStates(prev => {
            const newStates = [...prev];
            newStates[index] = {
                ...newStates[index],
                blockType,
                parameter,
                parameterName: parameter.name || 'Parameter',
                // isMenuOpen: false
            };
            return newStates;
        });
    };

    const handleEnvelopeChange = (index: number, params: { attackTime: number; peakValue: number; decayTime: number }) => {
        setSequencerStates(prev => {
            const newStates = [...prev];
            newStates[index] = {
                ...newStates[index],
                envelopeParams: params
            };
            return newStates;
        });
    };

    const handleMenuClick = (index: number) => {
        setSequencerStates(prev => {
            const newStates = [...prev];
            newStates[index] = {
                ...newStates[index],
                isMenuOpen: !newStates[index].isMenuOpen
            };
            return newStates;
        });
    };

    const handleValueChange = (index: number, values: boolean[]) => {
        setSequencerStates(prev => {
            const newStates = [...prev];
            newStates[index] = {
                ...newStates[index],
                values
            };
            return newStates;
        });
    };

    const handleBpmChange = (value: number) => {
        const newBpm = Math.round(value);
        setBpm(newBpm);
        
        // Update sequencer time units based on new BPM and individual durations
        sequencersRef.current.forEach((sequencer, index) => {
            const { bars, sixteenths } = sequencerStates[index].durationParams;
            const totalSteps = (bars * 16) + sixteenths;
            const timeUnit = (60000 / newBpm) / 4; // Convert BPM to milliseconds per sixteenth note
            sequencer.setTimeUnit(timeUnit * totalSteps);
        });
    };

    const handleDurationChange = (index: number, params: { bars: number; sixteenths: number }) => {
        setSequencerStates(prev => {
            const newStates = [...prev];
            newStates[index] = {
                ...newStates[index],
                durationParams: params
            };
            return newStates;
        });

        // Calculate time unit based on BPM and duration
        const totalSteps = (params.bars * 16) + params.sixteenths;
        const timeUnit = (60000 / bpm) / 4; // Convert BPM to milliseconds per sixteenth note
        sequencersRef.current[index].setTimeUnit(timeUnit*totalSteps);
    };

    return (<>
        <div className="content-block">
            <div className="bpm-control">
                <ValueControl
                    label="BPM"
                    min={40}
                    max={240}
                    value={bpm}
                    onChange={handleBpmChange}
                    formatValue={(value) => Math.round(value).toString()}
                />
            </div>
        </div>
        <div className="content-block">
            <div className="sequencer-blocks">
                {Array.from({ length: 6 }).map((_, index) => (
                    <div key={index} className="sequencer-block">
                        {sequencerStates[index].isMenuOpen && (
                            <SequencerMenuBlock 
                                id={`sequencer-menu-block-${index}`}
                                onBlockChange={(blockType, parameter) => handleBlockChange(index, blockType, parameter)}
                                onEnvelopeChange={(params) => handleEnvelopeChange(index, params)}
                                onDurationChange={(params) => handleDurationChange(index, params)}
                            />
                        )}
                        <SequencerControl
                            sequenceLength={8}
                            currentStep={sequencerStates[index].currentStep}
                            label={sequencerStates[index].parameter 
                                ? `${sequencerStates[index].blockType} - ${sequencerStates[index].parameterName}`
                                : `Sequencer ${index + 1}`
                            }
                            onMenuClick={() => handleMenuClick(index)}
                            onChange={(values) => handleValueChange(index, values)}
                        />
                    </div>
                ))}
            </div>
        </div>
    </>);
};
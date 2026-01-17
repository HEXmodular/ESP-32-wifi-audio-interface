import React, { useState, useCallback, useEffect } from 'react';
import ValueControl from '@controls/value-control/value-control';
import { SelectControl } from '@controls/select-control/select-control';
import { useOscillatorContext } from '@contexts/OscillatorContext';
import './oscillator-control.css';

import type { OscillatorConfig, WaveType } from '@api/oscillatorApi';

interface OscillatorControlProps {
    header?: string;
    showHeader?: boolean;
    showLabel?: boolean;
    config: OscillatorConfig; // Config is now mandatory
}

const OSCILLATOR_TYPES = [
    { value: 'server', label: 'Server Default' },
    { value: 'client-sine', label: 'Client Sine' },
    { value: 'client-square', label: 'Client Square' },
    { value: 'client-sawtooth', label: 'Client Sawtooth' },
    { value: 'client-triangle', label: 'Client Triangle' },
];

// Helper to create the combined type string for the SelectControl
const getCombinedType = (oscType: 'server' | 'client' | undefined, waveType: WaveType): string => {
    if (oscType === 'client') {
        return `client-${waveType.toLowerCase()}`;
    }
    // Default to 'server' if type is undefined or 'server'
    return 'server'; 
};

// Helper to parse the combined type string from the SelectControl
const parseCombinedType = (combinedType: string, currentWaveType: WaveType): { type: 'server' | 'client'; waveType: WaveType } => {
    if (combinedType === 'server') {
        // When switching to server, retain the current wave_type from config,
        // as the server will decide what to do with it or use its own default.
        return { type: 'server', waveType: currentWaveType };
    }
    const parts = combinedType.split('-'); // e.g., "client-sine"
    return { type: 'client', waveType: parts[1] as WaveType };
};


const OscillatorControl: React.FC<OscillatorControlProps> = ({
    header = 'Oscillator',
    showHeader = false,
    showLabel = false,
    config,
}) => {
    const { updateOscillator, oscillators: contextOscillators, oscillatorTypes } = useOscillatorContext();
    
    const actualOscillatorConfig = contextOscillators.find(osc => osc.oscillator_id === config.oscillator_id) || config;
    const currentOscillatorApiType = oscillatorTypes && oscillatorTypes[config.oscillator_id] ? oscillatorTypes[config.oscillator_id] : 'server';

    const [selectedCombinedType, setSelectedCombinedType] = useState<string>(
        getCombinedType(currentOscillatorApiType, actualOscillatorConfig.wave_type)
    );
    
    const [frequency, setFrequency] = useState(actualOscillatorConfig.frequency);
    const [amplitude, setAmplitude] = useState(actualOscillatorConfig.amplitude);
    const [isUpdating, setIsUpdating] = useState(false);

    useEffect(() => {
        const newActualOscillatorConfig = contextOscillators.find(osc => osc.oscillator_id === config.oscillator_id);
        if (newActualOscillatorConfig) {
            setFrequency(newActualOscillatorConfig.frequency);
            setAmplitude(newActualOscillatorConfig.amplitude);
            const newApiType = oscillatorTypes && oscillatorTypes[config.oscillator_id] ? oscillatorTypes[config.oscillator_id] : 'server';
            setSelectedCombinedType(getCombinedType(newApiType, newActualOscillatorConfig.wave_type));
        }
    }, [contextOscillators, config.oscillator_id, oscillatorTypes]);

    // const noteNames = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'];

    // const getNoteDisplay = useCallback((midiNote: number) => {
    //     const octave = Math.floor(midiNote / 12) - 1;
    //     const noteName = noteNames[midiNote % 12];
    //     return `${noteName}${octave}`;
    // }, [noteNames]);

    const oscillatorTypeLabels = OSCILLATOR_TYPES.map(opt => opt.label);
    const selectedTypeIndex = OSCILLATOR_TYPES.findIndex(opt => opt.value === selectedCombinedType);

    const frequencyToMidiNote = useCallback((freq: number) => {
        if (freq <= 0) return 0; // Avoid log(0) or negative
        return Math.round(12 * Math.log2(freq / 440) + 69);
    }, []);

    const midiNoteToFrequency = useCallback((note: number) => {
        return 440 * Math.pow(2, (note - 69) / 12);
    }, []);

    // Updated to match SelectControl's onChange signature for single mode
    const handleTypeChange = useCallback(async (controlId: string, index: number, isSelected: boolean) => {
        if (!isSelected || isUpdating || !config) return; // Only proceed if a new item is selected
        
        const newCombinedType = OSCILLATOR_TYPES[index]?.value;
        if (!newCombinedType) return;

        setIsUpdating(true);
        
        const currentActualOscillator = contextOscillators.find(osc => osc.oscillator_id === config.oscillator_id) || config;
        const { type: newContextType, waveType: newWaveType } = parseCombinedType(newCombinedType, currentActualOscillator.wave_type);
        
        // Update local state immediately for responsive UI
        setSelectedCombinedType(newCombinedType);

        const newApiConfig: OscillatorConfig = {
            ...currentActualOscillator,
            wave_type: newWaveType, // Update wave_type based on selection
            frequency: frequency,   // Persist current frequency
            amplitude: amplitude, // Persist current amplitude
            // Do not set 'type' here, it's not part of OscillatorConfig for the API
        };
        
        try {
            await updateOscillator(newApiConfig, newContextType);
        } catch (error) {
            console.error("Failed to update oscillator type", error);
            // Revert selection on error
            const oldApiType = oscillatorTypes && oscillatorTypes[config.oscillator_id] ? oscillatorTypes[config.oscillator_id] : 'server';
            setSelectedCombinedType(getCombinedType(oldApiType, currentActualOscillator.wave_type));
        } finally {
            setIsUpdating(false);
        }
    }, [isUpdating, config, updateOscillator, contextOscillators, frequency, amplitude, oscillatorTypes]);

    const handleFrequencyChange = useCallback(async (newFrequency: number) => {
        if (isUpdating || !config) return;
        setIsUpdating(true);
        setFrequency(newFrequency); // Update local state for UI

        const currentActualOscillator = contextOscillators.find(osc => osc.oscillator_id === config.oscillator_id) || config;
        const { type: currentContextType } = parseCombinedType(selectedCombinedType, currentActualOscillator.wave_type);
        
        const newApiConfig: OscillatorConfig = {
            ...currentActualOscillator,
            frequency: Math.round(newFrequency),
            amplitude: amplitude, // Ensure current amplitude is part of the update
            // wave_type remains as per currentActualOscillator.wave_type or what selectedCombinedType implies for client
        };
        
        try {
            await updateOscillator(newApiConfig, currentContextType);
        } catch (error) {
            console.error("Failed to update frequency", error);
            // Potentially revert local frequency if update fails
            setFrequency(currentActualOscillator.frequency);
        } finally {
            setIsUpdating(false);
        }
    }, [isUpdating, config, updateOscillator, selectedCombinedType, contextOscillators, amplitude]);
    
    const handleNoteChange = useCallback(async (note: number) => {
        // No need for isUpdating check here, handleFrequencyChange will do it.
        const newFrequency = midiNoteToFrequency(note);
        await handleFrequencyChange(newFrequency); 
    }, [midiNoteToFrequency, handleFrequencyChange]);

    const handleAmplitudeChange = useCallback(async (newAmplitude: number) => {
        if (isUpdating || !config) return;
        setIsUpdating(true);
        setAmplitude(newAmplitude); // Update local state for UI

        const currentActualOscillator = contextOscillators.find(osc => osc.oscillator_id === config.oscillator_id) || config;
        const { type: currentContextType } = parseCombinedType(selectedCombinedType, currentActualOscillator.wave_type);

        const newApiConfig: OscillatorConfig = {
            ...currentActualOscillator,
            amplitude: Math.round(newAmplitude),
            frequency: frequency, // Ensure current frequency is part of the update
        };

        try {
            await updateOscillator(newApiConfig, currentContextType);
        } catch (error) {
            console.error("Failed to update amplitude", error);
            setAmplitude(currentActualOscillator.amplitude);
        } finally {
            setIsUpdating(false);
        }
    }, [isUpdating, config, updateOscillator, selectedCombinedType, contextOscillators, frequency]);

    return (
        <div className="oscillator-control">
            {showHeader && <div className="title">{header}</div>}
            <div className="oscillator-control-container">
                <ValueControl
                    id={`${config.oscillator_id}-frequency`}
                    label="Frequency (Hz)"
                    min={20}
                    max={20000} // Increased max frequency
                    value={frequency}
                    onChange={handleFrequencyChange}
                    showLabel={showLabel}
                />
                <ValueControl
                    id={`${config.oscillator_id}-note`}
                    label="Note"
                    min={0}
                    max={127}
                    value={frequencyToMidiNote(frequency)}
                    onChange={handleNoteChange}
                    showLabel={showLabel}
                />
                <ValueControl
                    id={`${config.oscillator_id}-amplitude`}
                    label="Amplitude" // Assuming 0-100 from context
                    min={0}
                    max={100}
                    value={amplitude}
                    onChange={handleAmplitudeChange}
                    showLabel={showLabel} // ValueControl uses showLabel
                />
                <SelectControl
                    id={`${config.oscillator_id}-type`}
                    labels={oscillatorTypeLabels}
                    value={selectedTypeIndex >= 0 ? selectedTypeIndex : undefined} // Pass undefined if not found, though it should always be found
                    onChange={handleTypeChange}
                    mode="single"
                    columns={1} // Adjust columns as needed, e.g., 1 for a dropdown-like list or 2 for 2xN grid
                />
            </div>
        </div>
    );
};

export default OscillatorControl;
import React, { useState, useMemo } from 'react';
import { SelectControl } from '@controls/select-control/select-control';
import { useEffects } from '@contexts/EffectsContext';
import ValueControl from '@controls/value-control/value-control';
import './sequencer-select-menu-block.css';

interface SequencerSelectMenuBlockProps {
    onNotesChange: (values: boolean[]) => void;
    onStepsChange: (value: '8' | '16') => void;
    onTemperatureChange: (value: number) => void;
    activeNotes?: boolean[];
    temperature?: number;
    steps?: '8' | '16';
}

export const SequencerSelectMenuBlock: React.FC<SequencerSelectMenuBlockProps> = ({
    onNotesChange,
    onStepsChange,
    onTemperatureChange,
    activeNotes = Array(8).fill(false),
    temperature = 0.5,
    steps = '8'
}) => {
    // const [activeNotes, setActiveNotes] = useState<boolean[]>(activeNotes);
    // const [temperature, setTemperature] = useState(temperature);
    // const [steps, setSteps] = useState<'8' | '16'>(steps);

    const handleNotesChange = (values: boolean[]) => {
        // setActiveNotes(values);
        onNotesChange(values);
    };

    const handleTemperatureChange = (value: number) => {
        // setTemperature(value);
        onTemperatureChange(value);
    };

    const handleStepsChange = (value: string) => {
        const newSteps = value as '8' | '16';
        // setSteps(newSteps);
        onStepsChange(newSteps);
    };

    return (
        <div className="sequencer-select-menu-block">
            <div className="active-notes-selector">
                <SelectControl
                    labels={['1', '2', '3', '4', '5', '6', '7', '8']}
                    columns={8}
                    values={activeNotes}
                    onChange={(id, index, value, values) => handleNotesChange(values)}
                />
            </div>
            <div className="other-parameters"> 
                <ValueControl
                    min={0}
                    max={1}
                    sensitivity={0.01}
                    formatValue={(value) => value.toFixed(2)}
                    label="Temperature"
                    value={temperature}
                    onChange={handleTemperatureChange}
                />
                <SelectControl
                    label="Steps"
                    labels={['8', '16']}
                    mode="single"
                    columns={2}
                    values={[steps === '8' ? 0 : 1]}
                    onChange={(values) => handleStepsChange(values[0] ? '8' : '16')}
                />
            </div>
        </div>
    );
};


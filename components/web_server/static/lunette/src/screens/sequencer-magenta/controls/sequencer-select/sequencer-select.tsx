import React, { useState, useEffect } from 'react';
import { SequencerControl } from '@controls/sequencer-control/sequencer-control';
// import { Sequencer } from '@controls/sequencer-control/sequencer';
import './sequencer-select.css';

interface SequencerSelectProps {
    label: string;
    sequenceLength: number;
    rows: number;
    currentStep: number;
    values: boolean[];
    onMenuClick: () => void;
    onChange: (values: boolean[]) => void;
}

export const SequencerSelect: React.FC<SequencerSelectProps> = ({ 
    label,
    sequenceLength, 
    rows, 
    currentStep, 
    values, 
    onMenuClick, 
    onChange 
}) => {
    // State for sequencer values
    const [sequencerValues, setSequencerValues] = useState<boolean[][]>(
        Array(rows).fill(null).map(() => Array(sequenceLength).fill(false))
    );

    // Update sequencer values when values prop changes
    useEffect(() => {
        if (values.length > 0) {
            setSequencerValues(prev => {
                const newValues = [...prev];
                newValues[0] = values;
                return newValues;
            });
        }
    }, [values]);

    // Handle changes in rows prop
    useEffect(() => {
        setSequencerValues(prev => {
            const newValues = Array(rows).fill(null).map((_, rowIndex) => {
                // If we have existing values for this row, use them
                if (rowIndex < prev.length) {
                    return [...prev[rowIndex]];
                }
                // Otherwise create a new row of false values
                return Array(sequenceLength).fill(false);
            });
            return newValues;
        });
    }, [rows, sequenceLength]);

    const handleValueChange = (sequencerIndex: number, newValues: boolean[]) => {
        setSequencerValues(prev => {
            const newStates = [...prev];

            // For each column, ensure only one value is true
            for (let col = 0; col < sequenceLength; col++) {
                if (newValues[col]) {
                    // If this column is being set to true, set all others to false
                    for (let seq = 0; seq < rows; seq++) {
                        if (seq !== sequencerIndex) {
                            newStates[seq][col] = false;
                        }
                    }
                }
            }

            newStates[sequencerIndex] = newValues;
            // As per instruction, pass only the single row that was affected.
            onChange(newStates[sequencerIndex]); 
            return newStates;
        });
    };

    return (
        <div className="sequencer-select-scroll-container">
            <div className="sequencer-select-container"
                data-length={sequenceLength}
            >
            {sequencerValues.map((rowInternalValues, index) => ( // Renamed 'values' to 'rowInternalValues' for clarity
                <SequencerControl
                    key={index}
                    label={index === 0 ? label : ''}
                    sequenceLength={sequenceLength}
                    currentStep={currentStep}
                    values={rowInternalValues} // Pass the specific row's values to SequencerControl
                    onMenuClick={onMenuClick}
                    onChange={(newRowValues: boolean[]) => handleValueChange(index, newRowValues)}
                    hideMenu={index !== 0}
                    highlightedCells={Array(sequenceLength).fill(false).map((_, i) => i % 4 === 0)}
                    />
                ))}
            </div>
        </div>
    );
};
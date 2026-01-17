import React, { useState, useCallback, useEffect } from 'react';
import './select-control.css';

interface SelectControlProps {
    id?: string;
    label?: string;
    labels: string[];
    columns?: number;
    mode?: 'multiple' | 'single';
    value?: number;
    values?: number[];
    fontSize?: number;
    disabled?: boolean;
    onChange: (id: string, index: number, value: boolean, values?: boolean[]) => void;
}

export const SelectControl: React.FC<SelectControlProps> = ({
    id = '',
    label,
    labels,
    columns = 4,
    mode = 'multiple',
    value,
    values,
    fontSize = 16,
    disabled,
    onChange,
}) => {


    // console.log('SelectControl id', id, 'value', value);


    const valuesState = (valueArg?: number, valuesArray?: number[]) => {
        if (valueArg !== undefined) {
            // console.log('SelectControl valueArg', valueArg, Array(labels.length).fill(false).map((_, index) => index == valueArg));
            return Array(labels.length).fill(false).map((_, index) => index == valueArg);
        }
        if (valuesArray) {
            return valuesArray;
        }
        return Array(labels.length).fill(false);
    }

    const [isActive, setIsActive] = useState<boolean[]>(Array(labels.length).fill(false));

    useEffect(() => {
        if (disabled) {
            return;
        }
        setIsActive(valuesState(value, values));
    }, [disabled, value, values]);

    const handleToggle = useCallback((index: number) => {
        if (disabled) {
            return;
        }
        if (mode === 'single') {
            const newState = Array(labels.length).fill(false);
            newState[index] = true;
            setIsActive(newState);
            onChange?.(id, index, true);
        } else {
            const newState = [...isActive];
            newState[index] = !newState[index];
            setIsActive(newState);
            onChange?.(id, index, newState[index], newState);
        }


    }, [mode, labels.length, onChange, id]);

    // const getActiveIndices = useCallback(() => {
    //     return state.map((value, index) => value ? index : -1).filter(index => index !== -1);
    // }, [state]);

    return (
        <div className="select-control-wrapper">
            {label && (
                <div 
                    className="select-control-label"
                >
                    {label}
                </div>
            )}
            <div 
                className="logic-block-control"
                style={{
                    display: 'grid',
                    gridTemplateColumns: `repeat(${columns}, 1fr)`,
                    gap: '2px',
                    fontSize: `${fontSize}px`
                }}
            >
                {labels.map((label, index) => (
                    <button
                        key={index}
                        className={`logic-block-btn ${isActive[index] ? 'active' : ''}`}
                        onClick={() => handleToggle(index)}
                        data-index={index}
                    >
                        {label}
                    </button>
                ))}
            </div>
        </div>
    );
};

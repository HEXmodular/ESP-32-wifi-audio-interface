import React from 'react';
import { SelectControl } from '@controls/select-control/select-control';
import './mixer-block.css';

const mixerTypes = ['1', '2', '3', '4', 'L1', 'L2', 'L3', 'OFF'];

interface MixerBlockProps {
    id: string;
    title?: string;
    onMixerChange: (mixerId: string, ids: string[]) => void;
}

const MixerBlock: React.FC<MixerBlockProps> = ({ id, title, onMixerChange }) => {
    return (
        <div className="mixer-block">
            <div className="title">{title}</div>

            <SelectControl id="mixer-type" labels={mixerTypes} mode="single"
                onChange={(value) => {
                    if (value) {
                        onMixerChange(id, [value]);
                    }
                }} />
        </div>
    );
};

export default MixerBlock;

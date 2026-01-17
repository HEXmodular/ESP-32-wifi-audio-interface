import React, { useState, useMemo } from 'react';
import { SelectControl } from '@controls/select-control/select-control';
import { useEffects } from '@contexts/EffectsContext';
import ValueControl from '@controls/value-control/value-control';
import './sequencer-menu-block.css';
import type { AudioParamItem } from '@/interfaces/AudioParamList';

interface SequencerMenuBlockProps {
    id: string;
    onBlockChange?: (blockId: string, parameter:  AudioParamItem) => void;
    onEnvelopeChange?: (params: { attackTime: number; peakValue: number; decayTime: number }) => void;
    onDurationChange?: (params: { bars: number; sixteenths: number }) => void;
}

type BlockType = 'reverbAlgo' | 'outputFilters';

const availableBlocks = ['Reverb Algo', 'OUT Filters'] as const;

export const SequencerMenuBlock: React.FC<SequencerMenuBlockProps> = ({
    id,
    onBlockChange,
    onEnvelopeChange,
    onDurationChange
}) => {
    const [selectedBlock, setSelectedBlock] = useState<BlockType | null>(null);
    // const [selectedParameter, setSelectedParameter] = useState<AudioParam | null>(null);
    const [envelopeParams, setEnvelopeParams] = useState({
        attackTime: 0.1,
        peakValue: 1.0,
        decayTime: 0.2
    });
    const [durationParams, setDurationParams] = useState({
        bars: 1,
        sixteenths: 0
    });
    const { reverbAlgoAutomatedParameters, outputFiltersAutomatedParameters } = useEffects();

    const blockParameters = useMemo(() => {
        const reverbParams = reverbAlgoAutomatedParameters?.audioParamList || [];
        const filterParams = outputFiltersAutomatedParameters?.audioParamList || [];
        
        return {
            reverbAlgo: reverbParams,
            outputFilters: filterParams
        } as Record<BlockType, { name: string; shortName: string; audioParam: AudioParam }[]>;
    }, [reverbAlgoAutomatedParameters, outputFiltersAutomatedParameters]);

    const handleBlockChange = (_:any, index: number, value: boolean) => {
        if (value) {
            const blockType = index === 0 ? 'reverbAlgo' : 'outputFilters';
            setSelectedBlock(blockType);
            // setSelectedParameter(null);
        }
    };

    const handleParameterChange = (_: any, index: number, value: boolean) => {
        if (value && selectedBlock) {
            const parameter = blockParameters[selectedBlock][index];
            console.log('parameter', parameter, selectedBlock, );
            onBlockChange?.(selectedBlock, parameter);
        }
    };

    const handleEnvelopeParamChange = (param: keyof typeof envelopeParams, value: number) => {
        const newParams = { ...envelopeParams, [param]: value };
        setEnvelopeParams(newParams);
        onEnvelopeChange?.(newParams);
    };

    const handleDurationChange = (param: 'bars' | 'sixteenths', value: number) => {
        const newParams = { ...durationParams, [param]: value };
        setDurationParams(newParams);
        onDurationChange?.(newParams);
    };

    return (
        <div className="sequencer-menu-block">
            <div className="block-selector">
                <SelectControl
                    id={`${id}-block`}
                    labels={[...availableBlocks]}
                    mode="single"
                    onChange={handleBlockChange}
                />
            </div>
            {selectedBlock && (
                <div className="parameter-selector">
                    <SelectControl
                        id={`${id}-parameter`}
                        labels={blockParameters[selectedBlock].map(param => param.shortName)}
                        mode="single"
                        onChange={handleParameterChange}
                    />
                </div>
            )}

            <div className="envelope-params">
                <div className="envelope-param">
                    <ValueControl
                        label="Attack Time"
                        min={0.01}
                        max={2}
                        value={envelopeParams.attackTime}
                        onChange={(value) => handleEnvelopeParamChange('attackTime', value)}
                        formatValue={(value) => `${value.toFixed(2)}s`}
                    />
                </div>
                <div className="envelope-param">
                    <ValueControl
                        label="Peak Value"
                        min={-1}
                        max={1}
                        sensitivity={0.01}
                        value={envelopeParams.peakValue}
                        onChange={(value) => handleEnvelopeParamChange('peakValue', value)}
                        formatValue={(value) => value.toFixed(2)}
                    />
                </div>
                <div className="envelope-param">
                    <ValueControl
                        label="Decay Time"
                        min={0.01}
                        max={2}
                        sensitivity={0.01}
                        value={envelopeParams.decayTime}
                        onChange={(value) => handleEnvelopeParamChange('decayTime', value)}
                        formatValue={(value) => `${value.toFixed(2)}s`}
                    />
                </div>
            </div>

            <div className="duration-params">
                <ValueControl
                    label="Bars"
                    min={1}
                    max={16}
                    value={durationParams.bars}
                    onChange={(value) => handleDurationChange('bars', Math.round(value))}
                    formatValue={(value) => Math.round(value).toString()}
                />
                <ValueControl
                    label="/16"
                    min={0}
                    max={15}
                    value={durationParams.sixteenths}
                    onChange={(value) => handleDurationChange('sixteenths', Math.round(value))}
                    formatValue={(value) => Math.round(value).toString()}
                />
            </div>
        </div>
    );
};


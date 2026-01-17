// screen for managing effects
import React, { useEffect, useState } from 'react';
import ValueControl from '@controls/value-control/value-control';
import { useEffects } from '@contexts/EffectsContext';
import { type OutputFiltersParameters } from '@audio/output-filters';
import { type ReverbParameters } from '@audio/reverbAlgo';
import './effects-screen.css';

interface EffectsScreenProps {
    inputNode?: AudioNode | null;
    outputNode?: AudioNode | null;
    audioContext: AudioContext | null;
}

const EffectsScreen: React.FC<EffectsScreenProps> = ({
    inputNode,
    outputNode,
    audioContext
}) => {
    const {
        reverbAlgoNode,
        reverbAlgoParameters,
        setReverbAlgoParameters,
        outputFiltersNode,
        outputFiltersParameters,
        setOutputFiltersParameters
    } = useEffects();

    // const [reverbAlgoParametersInner, setReverbAlgoParametersInner] = useState<ReverbParameters | null>(null);

    useEffect(() => {
        if (audioContext && inputNode && outputNode && reverbAlgoNode && outputFiltersNode) {
            inputNode.connect(outputFiltersNode.input);
            outputFiltersNode.output.connect(reverbAlgoNode.input);
            reverbAlgoNode.output.connect(outputNode);


            const initialParams: ReverbParameters = {
                delayTime1: 2,
                delayTime2: 3,
                allpassFreq1: 1000,
                allpassFreq2: 2000,
                allpassFreq3: 3000,
                allpassFreq4: 4000,
                feedbackGain: 0.33,
                lowpassFreq: 10000,
                wetDryMix: 0.75,
            };

            // inputNode.connect(outputNode);
            setReverbAlgoParameters(initialParams);

            setOutputFiltersParameters({
                highpassFreq: 20,
                lowpassFreq: 12000,
            });
        }
    }, [audioContext, inputNode, outputNode, reverbAlgoNode, outputFiltersNode]);

    const handleParamChange = (param: keyof ReverbParameters, value: number) => {
        if (!reverbAlgoParameters) return;
        const newParams = { ...reverbAlgoParameters, [param]: value };
        setReverbAlgoParameters(newParams);
    };

    const handleOutputFiltersParamChange = (param: keyof OutputFiltersParameters, value: number) => {
        if (!outputFiltersParameters) return;
        const newParams = { ...outputFiltersParameters, [param]: value };
        setOutputFiltersParameters(newParams);
    };

    const isLoading = !inputNode || !outputNode || !reverbAlgoNode || !outputFiltersNode;

    return (
        <div className="effects-screen">
            <div className={`content-block ${!isLoading || "loading-block"}`}>
                <div className="output-filters-controls cols-2">
                    <ValueControl
                        label="OUTPUT HP"
                        min={10}
                        max={8000}
                        sensitivity={20.0}
                        value={outputFiltersParameters?.highpassFreq}
                        onChange={(value) => handleOutputFiltersParamChange('highpassFreq', value)}
                    />
                    <ValueControl
                        label="Lowpass Freq"
                        min={100}
                        max={16000}
                        sensitivity={20.0}
                        value={outputFiltersParameters?.lowpassFreq}
                        onChange={(value) => handleOutputFiltersParamChange('lowpassFreq', value)}
                    />
                </div>
            </div>

            <div className={`content-block ${!isLoading || "loading-block"}`}>
                <div className="reverb-controls">
                    <div className="reverb-control-row cols-2">
                        {/* {JSON.stringify(reverbAlgoParameters)} */}
                        <ValueControl
                            label="REVERB Delay"
                            min={0.01}
                            max={10.0}
                            sensitivity={0.01}
                            formatValue={(value) => value === undefined ? ' ' : value.toFixed(2)}
                            value={reverbAlgoParameters?.delayTime1}
                            onChange={(value) => handleParamChange('delayTime1', value)}
                        />
                        <ValueControl
                            label="&nbsp;"
                            min={0.01}
                            max={10.0}
                            sensitivity={0.01}
                            formatValue={(value) => value === null ? ' ' : value.toFixed(2)}
                            value={reverbAlgoParameters?.delayTime2}
                            onChange={(value) => handleParamChange('delayTime2', value)}
                        />
                    </div>

                    <div className="reverb-control-row cols-2">
                        <ValueControl
                            label="Allpass Freq"
                            min={100}
                            max={8000}
                            sensitivity={20.0}
                            value={reverbAlgoParameters?.allpassFreq1}
                            onChange={(value) => handleParamChange('allpassFreq1', value)}
                        />
                        <ValueControl
                            label="&nbsp;"
                            min={100}
                            max={8000}
                            sensitivity={20.0}
                            value={reverbAlgoParameters?.allpassFreq2}
                            onChange={(value) => handleParamChange('allpassFreq2', value)}
                        />
                    </div>

                    <div className="reverb-control-row cols-2">
                        <ValueControl
                            min={100}
                            max={8000}
                            sensitivity={20.0}
                            value={reverbAlgoParameters?.allpassFreq3}
                            onChange={(value) => handleParamChange('allpassFreq3', value)}
                        />
                        <ValueControl
                            min={100}
                            max={8000}
                            sensitivity={20.0}
                            value={reverbAlgoParameters?.allpassFreq4}
                            onChange={(value) => handleParamChange('allpassFreq4', value)}
                        />
                    </div>

                    <div className="reverb-control-row cols-2">
                        <ValueControl
                            label="Feedback Gain"
                            min={0.0}
                            max={2.0}
                            sensitivity={0.01}
                            formatValue={(value) => value.toFixed(2)}
                            value={reverbAlgoParameters?.feedbackGain}
                            onChange={(value) => handleParamChange('feedbackGain', value)}
                        />
                        <ValueControl
                            label="Lowpass Freq"
                            min={20}
                            max={20000}
                            sensitivity={20.0}
                            value={reverbAlgoParameters?.lowpassFreq}
                            onChange={(value) => handleParamChange('lowpassFreq', value)}
                        />
                    </div>

                    <ValueControl
                        label="Wet/Dry Mix"
                        min={0}
                        max={1}
                        sensitivity={0.01}
                        formatValue={(value) => value.toFixed(2)}
                        value={reverbAlgoParameters?.wetDryMix}
                        onChange={(value) => handleParamChange('wetDryMix', value)}
                    />
                </div>
            </div>
            <div id="debug-log" className="debug-log" style={{ color: 'black' }}>
            </div>
        </div>
    );
};

export default EffectsScreen;

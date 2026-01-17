// lowpass and highpass pair
import type { AudioParamList } from "@/interfaces/AudioParamList";
import { useRef, useEffect } from "react";

export interface OutputFiltersParameters {
    lowpassFreq: number;
    highpassFreq: number;
}

export class OutputFilters {
    // private context: AudioContext;
    private lowpassFilter: BiquadFilterNode;
    private highpassFilter: BiquadFilterNode;

    constructor(context: AudioContext) {
        // this.context = context;
        
        // Create and configure filters
        this.lowpassFilter = context.createBiquadFilter();
        this.highpassFilter = context.createBiquadFilter();

        this.lowpassFilter.type = 'lowpass';
        this.highpassFilter.type = 'highpass';

        this.lowpassFilter.frequency.value = 1200;
        this.highpassFilter.frequency.value = 20;

        // Connect filters
        this.highpassFilter.connect(this.lowpassFilter);
    }

    get input(): AudioNode {
        return this.highpassFilter;
    }

    get output(): AudioNode {
        return this.lowpassFilter;
    }

    connect(destination: AudioNode): this {
        this.output.connect(destination);
        return this;
    }

    getAllParameters(): OutputFiltersParameters {
        return {
            lowpassFreq: this.lowpassFilter.frequency.value,
            highpassFreq: this.highpassFilter.frequency.value
        };
    }

    getAllAutomatedParameters(): AudioParamList {
        return {
            hostName: 'reverbAlgo',
            audioParamList: [
                {
                    name: 'Lowpass Freq',
                    shortName: 'LPF',
                    audioParam: this.lowpassFilter.frequency,
                },
                {
                    name: 'Highpass Freq',
                    shortName: 'HPF',
                    audioParam: this.highpassFilter.frequency,
                },
            ],
        };
    }

    public setParameters(params: OutputFiltersParameters): void {
        this.lowpassFilter.frequency.value = params.lowpassFreq;
        this.highpassFilter.frequency.value = params.highpassFreq;
    }

    destroy(): void {
        this.lowpassFilter.disconnect();
        this.highpassFilter.disconnect();
    }
}

export const useOutputFilters = (context: AudioContext | null) => {
    const outputFiltersRef = useRef<OutputFilters | null>(null);

    useEffect(() => {
        if (!context) {
            console.error('OutputFilters: Audio context is not initialized');
            return;
        }

        outputFiltersRef.current = new OutputFilters(context);

        return () => {
            if (outputFiltersRef.current) {
                outputFiltersRef.current.destroy();
                outputFiltersRef.current = null;
            }
        };
    }, [context]);

    const setParameters = (params: OutputFiltersParameters) => {
        if (outputFiltersRef.current) {
            outputFiltersRef.current.setParameters(params);
        }
    };

    return {
        outputFiltersNode: outputFiltersRef.current,
        outputFiltersParameters: outputFiltersRef.current?.getAllParameters() ?? null,
        outputFiltersAutomatedParameters: outputFiltersRef.current?.getAllAutomatedParameters() ?? null,
        setOutputFiltersParameters: setParameters,
    };

};

// контекст для хранения и предоставляени экземпляров созданных эффектов
// нужно взять код из effects-screen.tsx и перенести в контекст
//     const { reverbAlgoNode, reverbAlgoParameters, setReverbAlgoParameters } = useReverbAlgo(audioContext);
// const { outputFiltersNode, outputFiltersParameters, setOutputFiltersParameters } = useOutputFilters(audioContext);
// после этого в файлы effects-screen.tsx и sequencer-menu-block.tsx нужно будет добавить провайдер этого контекста

import React, { createContext, useContext, type ReactNode } from 'react';
import useReverbAlgo, { type ReverbParameters, type AlgorithmicReverb } from '@audio/reverbAlgo';
import { useOutputFilters, type OutputFiltersParameters, type OutputFilters } from '@audio/output-filters';
import type { AudioParamList } from '@/interfaces/AudioParamList';

interface EffectsContextType {
    audioContext: AudioContext | null;
    reverbAlgoNode: AlgorithmicReverb | null;
    reverbAlgoParameters: ReverbParameters | null;
    reverbAlgoAutomatedParameters: AudioParamList | null;
    setReverbAlgoParameters: (params: ReverbParameters) => void;
    outputFiltersNode: OutputFilters | null;
    outputFiltersParameters: OutputFiltersParameters | null;
    outputFiltersAutomatedParameters: AudioParamList | null;
    setOutputFiltersParameters: (params: OutputFiltersParameters) => void;
}

const EffectsContext = createContext<EffectsContextType | null>(null);

interface EffectsProviderProps {
    children: ReactNode;
    audioContext: AudioContext | null;
}

export const EffectsProvider: React.FC<EffectsProviderProps> = ({ children, audioContext }) => {
    const { reverbAlgoNode, reverbAlgoParameters, reverbAlgoAutomatedParameters, setReverbAlgoParameters,  } = useReverbAlgo(audioContext);
    const { outputFiltersNode, outputFiltersParameters, outputFiltersAutomatedParameters, setOutputFiltersParameters } = useOutputFilters(audioContext);

    const value = {
        audioContext,
        reverbAlgoNode,
        reverbAlgoParameters,
        reverbAlgoAutomatedParameters,
        setReverbAlgoParameters,
        outputFiltersNode,
        outputFiltersParameters,
        outputFiltersAutomatedParameters,
        setOutputFiltersParameters
    };

    return (
        <EffectsContext.Provider value={value}>
            {children}
        </EffectsContext.Provider>
    );
};

export const useEffects = () => {
    const context = useContext(EffectsContext);
    if (!context) {
        throw new Error('useEffects must be used within an EffectsProvider');
    }
    return context;
};


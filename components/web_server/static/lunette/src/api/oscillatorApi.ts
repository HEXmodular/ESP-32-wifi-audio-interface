import { BaseApi } from './baseApi';
import customDebounce from '@utils/customDebounce';

export interface OscillatorConfig {
    oscillator_id: number;
    frequency: number;
    amplitude: number;
}

interface OscillatorResponse {
    oscillators: OscillatorConfig[];
}

const useOscillatorApi = () => {
    const updateOscillator = customDebounce(async (config: OscillatorConfig): Promise<void> => {
        const result = await BaseApi.post('oscillator', config);
        if (!result.success) {
            throw result.error;
        }
        // No specific return value needed if successful, or handle result.data if it's meaningful
    }, 500, { maxWait: 1000 });

    const getOscillators = async (): Promise<OscillatorConfig[]> => {
        const result = await BaseApi.get<OscillatorResponse>('oscillators');
        if (result.success) {
            return result.data.oscillators;
        } else {
            throw result.error;
        }
    };

    return {
        updateOscillator,
        getOscillators,
    };
};

export default useOscillatorApi;
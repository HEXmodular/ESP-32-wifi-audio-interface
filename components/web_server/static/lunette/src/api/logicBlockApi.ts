import { BaseApi } from './baseApi';

export interface LogicBlockConfig {
    logic_block_id: number;
    operation_type: string;
    input1_id: number;
    input1_type: string;
    input2_id: number;
    input2_type: string;
}

interface LogicBlockResponse {
    logical_ops: LogicBlockConfig[];
}

const useLogicBlockApi = () => {

    const getLogicBlocks = async (): Promise<LogicBlockConfig[]> => {
        const result = await BaseApi.get<LogicBlockResponse>('logical-ops');
        if (result.success) {
            return result.data.logical_ops;
        } else {
            throw result.error;
        }
    };

    const updateLogicBlock = async (config: LogicBlockConfig): Promise<void> => {
        const result = await BaseApi.post('logical-ops', config);
        if (!result.success) {
            throw result.error;
        }
    };

    return {
        updateLogicBlock,
        getLogicBlocks,
    };
};

export default useLogicBlockApi;
import React, { useEffect, useState } from 'react';
import { SelectControl } from '@controls/select-control/select-control';
import type { LogicBlockConfig } from '@api/logicBlockApi';

import './algorithm-block.css';

const logicBlockTypes = ['AND', 'NAND', 'OR', 'NOR', 'XOR', 'XNOR', 'OFF'];
const serverLogicBlockTypes = [
    'LOGICAL_OP_AND', 'LOGICAL_OP_NAND', 'LOGICAL_OP_OR', 'LOGICAL_OP_NOR', 'LOGICAL_OP_XOR', 'LOGICAL_OP_XNOR', 'LOGICAL_OP_OFF'];
const logicConnectionsTypes = ['1', '2', '3', '4', 'L1', 'L2', 'L3', 'OFF'];

interface AlgorithmBlockProps {
    id: number;
    title?: string;
    config?: LogicBlockConfig;
    disabled?: boolean;
    onBlockChange: (data: LogicBlockConfig) => void;
}

const AlgorithmBlock: React.FC<AlgorithmBlockProps> = ({
    id,
    title,
    config,
    disabled,
    onBlockChange,
}) => {
    const [blockType, setBlockType] = useState<number | undefined>();
    const [input1Id, setInput1Id] = useState<number | undefined>();
    const [input2Id, setInput2Id] = useState<number | undefined>();


    const getBlockTypeValue = () => {
        return serverLogicBlockTypes.indexOf(config?.operation_type || serverLogicBlockTypes[0]);
    }

    useEffect(() => {
        handleChange();
    }, [input1Id, input2Id, blockType]);

    const handleChange = () => {
        const data = {
            logic_block_id: id,
            operation_type: blockType !== undefined ? serverLogicBlockTypes[blockType] : (config?.operation_type || serverLogicBlockTypes[0]),
            input1_id: input1Id !== undefined ? input1Id : (config?.input1_id || 0),
            input1_type: config?.input1_type || logicConnectionsTypes[0],
            input2_id: input2Id !== undefined ? input2Id : (config?.input2_id || 0),
            input2_type: config?.input2_type || logicConnectionsTypes[0]
        }

        if (!disabled && onBlockChange) {
            onBlockChange(data);
        }
    }

    const onConnectionTypeChange = (id: string, index: number, value: boolean) => {
        if (!value) {
            return;
        }

        if (id == '1') {
            setInput1Id(index);
        } else {
            setInput2Id(index);
        }
    }

    const onBlockTypeChange = (_: any, index: number, value: boolean) => {
        if (!value) {
            return;
        }

        setBlockType(index);
    }

    return (
        <div className="logic-block-section">
            <div className="title">{title}</div>
            <div className="connection-block">
                <div className="logic-block">
                    <SelectControl
                        id="1"
                        labels={logicConnectionsTypes}
                        mode="single"
                        columns={2}
                        value={config?.input1_id}
                        onChange={onConnectionTypeChange}
                        disabled={disabled}
                    />
                </div>
                <div className="logic-block">
                    <SelectControl
                        id="2"
                        labels={logicConnectionsTypes}
                        mode="single"
                        columns={2}
                        value={config?.input2_id}
                        onChange={onConnectionTypeChange}
                        disabled={disabled}
                    />
                </div>
            </div>

            <div className="arrow-down-container">
                <div className="arrow-down">↓</div>
                <div className="arrow-down">↓</div>
            </div>
            <div className="logic-block">
                <div className="logic-block-section">
                    <SelectControl
                        id="block-type"
                        labels={logicBlockTypes}
                        mode="single"
                        value={getBlockTypeValue()}
                        fontSize={12}
                        onChange={onBlockTypeChange}
                        disabled={disabled}
                    />
                </div>
            </div>
        </div>
    );
};

export default AlgorithmBlock;


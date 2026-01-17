import { useEffect, useState, useRef } from 'react'
import OscillatorControl from '@controls/oscillator-control/oscillator-control'
import AlgorithmBlock from '@algorithm/algorithm-4o3l/algorithm-block'
import MixerBlock from '@controls/mixer-block/mixer-block'
import useLogicBlockApi, { type LogicBlockConfig } from '@api/logicBlockApi'
import { useOscillatorContext } from '@contexts/OscillatorContext'
import './main-screen.css'

const MainScreen: React.FC = () => {
  const [logicBlocks, setLogicBlocks] = useState<LogicBlockConfig[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const dataFetching = useRef({ logicBlocks: false }); // to prevent multiple fetching in development mode
  const { oscillators, updateOscillator, isLoading: oscillatorsLoading } = useOscillatorContext();
  const { getLogicBlocks, updateLogicBlock } = useLogicBlockApi();

  useEffect(() => { 
    const fetchLogicBlocks = async () => {
      if (!dataFetching.current.logicBlocks) {
        try {
          dataFetching.current.logicBlocks = true;
          setIsLoading(true);
          setError(null);
          const logics = await getLogicBlocks();
          setLogicBlocks(logics);
        } catch (err) {
          setError(err instanceof Error ? err.message : 'Failed to fetch logic blocks');
          // Set empty array to prevent UI from breaking
          setLogicBlocks([]);
        } finally {
          setIsLoading(false);
        }
      }
    };

    fetchLogicBlocks();
  }, [getLogicBlocks]);

  const setValueBlock = async (config: LogicBlockConfig) => {
    try {
      await updateLogicBlock(config);
      const index = config.logic_block_id;
      const newLogicBlocks = [...logicBlocks];
      newLogicBlocks[index] = config;
      setLogicBlocks(newLogicBlocks);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to update logic block');
    }
  }

  return (
    <div>
      {error && (
        <div className="error-message">
          {error}
        </div>
      )}
      <div className={`content-block ${oscillatorsLoading ? "loading-block" : ""}`}>
        {oscillators?.map((oscillator, index) => (
          <OscillatorControl
            key={oscillator.oscillator_id}
            config={oscillator}
            showLabel={index === 0}
            onChange={(frequency) => {
              updateOscillator({
                oscillator_id: index,
                frequency: frequency,
                amplitude: 1.0,
              });
            }}
          />
        ))}
      </div>

      <div className={`content-block ${isLoading ? "loading-block" : ""}`}>
        <div className="cols-2 block-container">
          <AlgorithmBlock title="LOP 1" id={0} onBlockChange={setValueBlock}
            disabled={!logicBlocks?.[0]}
            config={logicBlocks?.[0]}
          />
          <AlgorithmBlock title="LOP 2" id={1} onBlockChange={setValueBlock}
            disabled={!logicBlocks?.[1]}
            config={logicBlocks?.[1]}
          />
        </div>
        <div className="cols-2 block-container">
          <AlgorithmBlock title="LOP 3" id={2} onBlockChange={setValueBlock}
            disabled={!logicBlocks?.[2]}
            config={logicBlocks?.[2]}
          />
          <MixerBlock id="mixer-1" title="MIXER 1" onMixerChange={() => { }} />
        </div>
      </div>
    </div>
  );
};

export default MainScreen;

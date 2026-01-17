// нужен атрибут который будет отображать длину секвенсора, это крестик в квадрате соответствующего шага
// по длинному нажатию на ячейку


import React, { useState, useEffect, useRef } from 'react';
import './sequencer-control.css';

interface SequencerControlProps {
  onChange?: (activeCells: boolean[]) => void;
  onMenuClick?: () => void;
  currentStep?: number;
  sequenceLength?: number;
  label?: string;
  hideMenu?: boolean;
  autoScroll?: boolean;
  highlightedCells?: boolean[]; // Array of booleans indicating which cells should be highlighted in gray
}

export const SequencerControl: React.FC<SequencerControlProps> = ({
  onChange,
  onMenuClick,
  currentStep = -1,
  sequenceLength = 8,
  label = '',
  hideMenu = false,
  autoScroll = false,
  highlightedCells = []
}) => {
  const [cells, setCells] = useState<boolean[]>(Array(sequenceLength).fill(false));
  const containerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    // Update cells when sequenceLength changes
    setCells(Array(sequenceLength).fill(false));
  }, [sequenceLength]);

  useEffect(() => {
    if (!autoScroll) return;
    const container = containerRef.current;
    if (currentStep == 0 && container) {
      container.scrollLeft = 0;
    }
    // Auto-scroll when current step is greater than 8
    if (currentStep == 8 && container) {
      // const cellWidth = container.scrollWidth / 2;
      const scrollPosition = container.scrollWidth / 1;
      container.scrollLeft = scrollPosition;
      console.log("scrolling to", scrollPosition, container.scrollWidth);
    }
  }, [currentStep, sequenceLength]);

  const handleCellClick = (index: number) => {
    const newCells = [...cells];
    newCells[index] = !newCells[index];
    setCells(newCells);
    onChange?.(newCells);
  };

  return (<>
    <div className="sequencer-label">{label}</div>
    <div className={`sequencer-control-container ${autoScroll ? 'auto-scroll' : ''}`}
      ref={containerRef}
    >
      <div
        className="sequencer-control"

        style={{ '--sequence-length': sequenceLength+1 } as React.CSSProperties}
        data-length={sequenceLength+1}
      >
        {cells.map((isActive, index) => (
          <button
            key={index}
            className={`sequencer-cell ${isActive ? 'active' : ''} ${index === currentStep ? 'current' : ''} ${index === sequenceLength - 1 ? 'length-indicator' : ''} ${highlightedCells[index] ? 'highlighted' : ''}`}
            onClick={() => handleCellClick(index)}
          >
            {index === sequenceLength - 1 && <span className="length-cross">×</span>}
          </button>
        ))}
        {!hideMenu && <button
          className="sequencer-menu-button"
          onClick={onMenuClick}
          aria-label="Menu"
        >
          ≡
        </button>}
      </div>
    </div>
  </>
  );
};

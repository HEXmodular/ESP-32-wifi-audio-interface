// DONE - The component is optimized for mobile screens with a max-width of 320px
// DONE - The keys are touch-friendly with appropriate sizing and spacing
// DONE - Black keys are transparent (not filled)
// DONE - Active keys are highlighted with a different color
// DONE - Each C note shows the octave number (e.g., C5)
// DONE - Quick octave selection is available through buttons
// DONE - Support for showing two octaves stacked vertically
// DONE - activeNote: The currently selected note (e.g., "C4")
// DONE - onNoteSelect: Callback function when a note is selected
// DONE - showTwoOctaves: Boolean to show two octaves stacked vertically
// DONE - Store and maintain the keyboard state after key press
// DONE - Replace the custom swipe indicator with a native scrollbar for octave selection
// DONE - Limit the scrollbar height to match the piano keyboard height
// DONE - Show the current octave in the scrollbar

// переключатель октавы должен быть по высоте в два раза ваше, если отображается сразу две клавиатуры showTwoOctaves

import React, { useEffect, useRef, useState } from 'react';
import './piano-control.css';

interface PianoControlProps {
  freq?: number;
  onNoteSelect?: (frequency: number) => void;
  showTwoOctaves?: boolean;
}

const NOTES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'];
const OCTAVES = [-1, 0, 1, 2, 3, 4, 5, 6];

// A4 = 440Hz
const A4_FREQUENCY = 440;
const A4_OCTAVE = 4;


const getNoteFrequency = (note: string, octave: number): number => {
  const noteIndex = NOTES.indexOf(note);
  const octaveDiff = octave - A4_OCTAVE;
  const noteDiff = noteIndex - NOTES.indexOf('A');
  return A4_FREQUENCY * Math.pow(2, octaveDiff + noteDiff / 12);
};

const getNoteFromFrequency = (frequency?: number): string => {
  if (!frequency) return '';
  const noteDiff = Math.round(12 * Math.log2(frequency / A4_FREQUENCY));
  const octaveDiff = Math.floor(noteDiff / 12);
  const noteIndex = (noteDiff % 12 + 12) % 12;
  const octave = A4_OCTAVE + octaveDiff;
  return `${NOTES[noteIndex]}${octave}`;
};

export const PianoControl: React.FC<PianoControlProps> = ({
  freq,
  onNoteSelect,
  showTwoOctaves = true,
}) => {
  const [selectedOctave, setSelectedOctave] = useState(4);
  const [secondOctave, setSecondOctave] = useState(5);
  const [pressedKey, setPressedKey] = useState("");
  const pianoRef = useRef<HTMLDivElement>(null);
  const octaveScrollRef = useRef<HTMLDivElement>(null);

  const scrollToNote = (note: string) => {
    if (!pianoRef.current) return;
    const noteElement = pianoRef.current.querySelector(`[data-note="${note}"]`);
    if (noteElement) {
      noteElement.scrollIntoView({ behavior: 'smooth', block: 'nearest', inline: 'center' });
    }
  };

  useEffect(() => {
    if (freq) {
      const note = getNoteFromFrequency(freq);
      scrollToNote(note);
    }
  }, [freq]);

  useEffect(() => {
    if (octaveScrollRef.current) {
      const markerHeight = octaveScrollRef.current.clientHeight / (showTwoOctaves ? 2 : 1);
      const scrollPosition = OCTAVES.indexOf(selectedOctave) * markerHeight;
      octaveScrollRef.current.scrollTop = scrollPosition;
    }
  }, [selectedOctave, showTwoOctaves]);

  const handleNoteClick = (note: string, octave: number) => {
    const fullNote = `${note}${octave}`;
    const frequency = getNoteFrequency(note, octave);
    setPressedKey(fullNote);
    onNoteSelect?.(frequency);
  };

  const handleOctaveScroll = (e: React.UIEvent<HTMLDivElement>) => {
    const scrollTop = e.currentTarget.scrollTop;
    const markerHeight = e.currentTarget.clientHeight / (showTwoOctaves ? 2 : 1);
    const octaveIndex = Math.round(scrollTop / markerHeight);
    
    if (octaveIndex >= 0 && octaveIndex < OCTAVES.length) {
      setSelectedOctave(OCTAVES[octaveIndex]);
      if (octaveIndex + 1 < OCTAVES.length) {
        setSecondOctave(OCTAVES[octaveIndex + 1]);
      }
    }
  };

  const renderPianoKeys = (octave: number) => {
    return NOTES.map((note) => {
      const isBlackKey = note.includes('#');
      const fullNote = `${note}${octave}`;
      const isOutsideFrequency = getNoteFromFrequency(freq) === fullNote;
      const isPressed = pressedKey === fullNote;
      const showOctaveLabel = note === 'C';

      const isActive = pressedKey === "" ? isOutsideFrequency : isPressed;

      return (
        <div
          key={fullNote}
          className={`piano-key ${isBlackKey ? 'black-key' : 'white-key'} ${isActive ? 'active' : ''}`}
          data-note={fullNote}
          onClick={() => handleNoteClick(note, octave)}
        >
          {showOctaveLabel && <span className="octave-label">C{octave}</span>}
        </div>
      );
    });
  };

  return (
    <div className="piano-control">
      <div className="piano-container" ref={pianoRef}>
        <div className="piano-keys">
          {renderPianoKeys(selectedOctave)}
        </div>
        
        {showTwoOctaves && (
          <div className="piano-keys">
            {renderPianoKeys(secondOctave)}
          </div>
        )}
      </div>
      
      <div 
        className={`octave-scrollbar ${showTwoOctaves ? 'two-octaves' : ''}`}
        ref={octaveScrollRef}
        onScroll={handleOctaveScroll}
      >
        <div className="octave-markers">
          {OCTAVES.map(octave => (
            <div 
              key={octave}
              className={`octave-marker ${selectedOctave === octave ? 'active' : ''}`}
            >
              {octave}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};



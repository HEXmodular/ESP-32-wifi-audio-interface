import { PianoControl } from "@controls/piano-control/piano-control";
import './keyboard-screen.css';
import { useOscillatorContext } from "@/contexts/OscillatorContext";

export const KeyboardScreen = () => {
    const { oscillators, updateOscillator, isLoading: oscillatorsLoading } = useOscillatorContext();

    const updateOscillatorFrequency = (oscillator_id: number, frequency: number) => {
        updateOscillator({ oscillator_id, frequency, amplitude: 1.0 });
    }

  return (
    <div className={`keyboard-screen ${!oscillatorsLoading || "loading-block"} `}>
      <PianoControl freq={oscillators[0]?.frequency} onNoteSelect={(value) => updateOscillatorFrequency(0, value)} />
      <PianoControl freq={oscillators[1]?.frequency} onNoteSelect={(value) => updateOscillatorFrequency(1, value)} />
      <PianoControl freq={oscillators[2]?.frequency} onNoteSelect={(value) => updateOscillatorFrequency(2, value)} />
      <PianoControl freq={oscillators[3]?.frequency} onNoteSelect={(value) => updateOscillatorFrequency(3, value)} />
    </div>
  );
};

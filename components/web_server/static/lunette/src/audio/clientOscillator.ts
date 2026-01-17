import type { OscillatorConfig, WaveType } from '@api/oscillatorApi';

// Helper to convert API WaveType to Web Audio OscillatorType
const toWebAudioWaveType = (waveType: WaveType): OscillatorType => {
  return waveType.toLowerCase() as OscillatorType;
};

export interface ClientOscillatorNodes {
  oscillatorNode: OscillatorNode;
  gainNode: GainNode;
}

/**
 * Creates and configures a client-side oscillator using the Web Audio API.
 * @param audioContext The AudioContext to use.
 * @param initialConfig The initial configuration for the oscillator.
 * @returns An object containing the OscillatorNode and GainNode.
 */
export const createClientOscillator = (
  audioContext: AudioContext,
  initialConfig: Pick<OscillatorConfig, 'frequency' | 'amplitude' | 'wave_type' | 'is_active'>
): ClientOscillatorNodes | null => {
  if (!initialConfig.is_active) {
    return null; // Don't create if not active initially
  }

  const oscillatorNode = audioContext.createOscillator();
  const gainNode = audioContext.createGain();

  oscillatorNode.type = toWebAudioWaveType(initialConfig.wave_type);
  oscillatorNode.frequency.setValueAtTime(initialConfig.frequency, audioContext.currentTime);
  gainNode.gain.setValueAtTime(initialConfig.amplitude / 100, audioContext.currentTime); // Assuming amplitude is 0-100

  oscillatorNode.connect(gainNode);
  gainNode.connect(audioContext.destination);
  oscillatorNode.start();

  return { oscillatorNode, gainNode };
};

/**
 * Updates the frequency of an existing client-side oscillator.
 * @param oscillatorNode The OscillatorNode to update.
 * @param frequency The new frequency value.
 * @param audioContext The AudioContext for timing.
 */
export const updateClientOscillatorFrequency = (
  oscillatorNode: OscillatorNode,
  frequency: number,
  audioContext: AudioContext
): void => {
  oscillatorNode.frequency.setValueAtTime(frequency, audioContext.currentTime);
};

/**
 * Updates the amplitude (gain) of an existing client-side oscillator.
 * @param gainNode The GainNode to update.
 * @param amplitude The new amplitude value (0-100).
 * @param audioContext The AudioContext for timing.
 */
export const updateClientOscillatorAmplitude = (
  gainNode: GainNode,
  amplitude: number, // Assuming 0-100
  audioContext: AudioContext
): void => {
  gainNode.gain.setValueAtTime(amplitude / 100, audioContext.currentTime);
};

/**
 * Updates the waveform type of an existing client-side oscillator.
 * @param oscillatorNode The OscillatorNode to update.
 * @param waveType The new WaveType.
 */
export const updateClientOscillatorWaveType = (
  oscillatorNode: OscillatorNode,
  waveType: WaveType
): void => {
  oscillatorNode.type = toWebAudioWaveType(waveType);
};

/**
 * Stops and disconnects a client-side oscillator.
 * @param oscillatorNode The OscillatorNode to stop.
 * @param gainNode The GainNode to disconnect.
 */
export const stopClientOscillator = (
  oscillatorNode: OscillatorNode,
  gainNode: GainNode
): void => {
  try {
    oscillatorNode.stop();
  } catch (e) {
    // Oscillator might already be stopped or in a state where stop() throws an error.
    console.warn('Error stopping oscillator node:', e);
  }
  oscillatorNode.disconnect();
  gainNode.disconnect();
};

/**
 * Starts or ensures a client-side oscillator is running.
 * If oscillator is suspended, resumes it.
 * Note: This doesn't handle creation, only ensures an existing one is playing.
 * @param audioContext The AudioContext.
 */
export const ensureOscillatorIsRunning = (audioContext: AudioContext): void => {
  if (audioContext.state === 'suspended') {
    audioContext.resume().catch(err => console.error("Error resuming AudioContext:", err));
  }
};

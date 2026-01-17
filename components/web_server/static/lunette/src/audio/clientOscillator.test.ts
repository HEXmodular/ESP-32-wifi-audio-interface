import {
  createClientOscillator,
  updateClientOscillatorFrequency,
  updateClientOscillatorAmplitude,
  updateClientOscillatorWaveType,
  stopClientOscillator,
  ensureOscillatorIsRunning,
  // toWebAudioWaveType is implicitly tested via its usage in create and update wave type.
} from './clientOscillator';
import type { OscillatorConfig, WaveType } from '@api/oscillatorApi';

// Mock Web Audio API parts
const mockSetValueAtTime = jest.fn();
const mockStart = jest.fn();
const mockStop = jest.fn();
const mockDisconnect = jest.fn();
const mockResume = jest.fn().mockResolvedValue(undefined);

const mockOscillatorNode = {
  type: 'sine' as OscillatorType,
  frequency: { value: 440, setValueAtTime: mockSetValueAtTime },
  connect: jest.fn(),
  start: mockStart,
  stop: mockStop,
  disconnect: mockDisconnect,
};

const mockGainNode = {
  gain: { value: 1, setValueAtTime: mockSetValueAtTime },
  connect: jest.fn(),
  disconnect: mockDisconnect,
};

const mockAudioContext = {
  currentTime: 0,
  state: 'running' as AudioContextState,
  createOscillator: jest.fn(() => mockOscillatorNode as any), // Use 'as any' to bypass strict type checks for mocks
  createGain: jest.fn(() => mockGainNode as any),
  resume: mockResume,
  destination: {} as AudioDestinationNode, // Dummy destination
};

describe('clientOscillator', () => {
  beforeEach(() => {
    // Reset mocks before each test
    jest.clearAllMocks();
    mockOscillatorNode.type = 'sine';
    mockOscillatorNode.frequency.value = 440;
    mockGainNode.gain.value = 1;
    mockAudioContext.state = 'running';
    mockAudioContext.currentTime = 0;
  });

  describe('createClientOscillator', () => {
    const baseConfig: Pick<OscillatorConfig, 'frequency' | 'amplitude' | 'wave_type' | 'is_active'> = {
      frequency: 440,
      amplitude: 50, // 0-100 scale
      wave_type: 'SINE',
      is_active: true,
    };

    it('should return null if initialConfig.is_active is false', () => {
      const result = createClientOscillator(mockAudioContext as any, { ...baseConfig, is_active: false });
      expect(result).toBeNull();
      expect(mockAudioContext.createOscillator).not.toHaveBeenCalled();
    });

    it('should create, configure, and start oscillator and gain nodes', () => {
      const result = createClientOscillator(mockAudioContext as any, baseConfig);

      expect(mockAudioContext.createOscillator).toHaveBeenCalledTimes(1);
      expect(mockAudioContext.createGain).toHaveBeenCalledTimes(1);

      expect(result?.oscillatorNode).toBe(mockOscillatorNode);
      expect(result?.gainNode).toBe(mockGainNode);

      expect(mockOscillatorNode.type).toBe('sine');
      expect(mockOscillatorNode.frequency.setValueAtTime).toHaveBeenCalledWith(baseConfig.frequency, mockAudioContext.currentTime);
      expect(mockGainNode.gain.setValueAtTime).toHaveBeenCalledWith(baseConfig.amplitude / 100, mockAudioContext.currentTime);
      
      expect(mockOscillatorNode.connect).toHaveBeenCalledWith(mockGainNode);
      expect(mockGainNode.connect).toHaveBeenCalledWith(mockAudioContext.destination);
      expect(mockOscillatorNode.start).toHaveBeenCalledTimes(1);
    });

    it('should correctly set different wave types', () => {
      createClientOscillator(mockAudioContext as any, { ...baseConfig, wave_type: 'SQUARE' });
      expect(mockOscillatorNode.type).toBe('square');
    });
  });

  describe('updateClientOscillatorFrequency', () => {
    it('should call setValueAtTime on oscillator frequency', () => {
      updateClientOscillatorFrequency(mockOscillatorNode as any, 880, mockAudioContext as any);
      expect(mockOscillatorNode.frequency.setValueAtTime).toHaveBeenCalledWith(880, mockAudioContext.currentTime);
    });
  });

  describe('updateClientOscillatorAmplitude', () => {
    it('should call setValueAtTime on gain node gain with normalized value', () => {
      updateClientOscillatorAmplitude(mockGainNode as any, 75, mockAudioContext as any); // 75%
      expect(mockGainNode.gain.setValueAtTime).toHaveBeenCalledWith(0.75, mockAudioContext.currentTime);
    });
  });

  describe('updateClientOscillatorWaveType', () => {
    it('should set the type on the oscillator node', () => {
      updateClientOscillatorWaveType(mockOscillatorNode as any, 'TRIANGLE');
      expect(mockOscillatorNode.type).toBe('triangle');
    });
  });

  describe('stopClientOscillator', () => {
    it('should call stop and disconnect on nodes', () => {
      stopClientOscillator(mockOscillatorNode as any, mockGainNode as any);
      expect(mockOscillatorNode.stop).toHaveBeenCalledTimes(1);
      expect(mockOscillatorNode.disconnect).toHaveBeenCalledTimes(1);
      expect(mockGainNode.disconnect).toHaveBeenCalledTimes(1);
    });

    it('should not throw if stop() throws (e.g. already stopped)', () => {
      mockOscillatorNode.stop.mockImplementationOnce(() => { throw new Error('Already stopped'); });
      expect(() => stopClientOscillator(mockOscillatorNode as any, mockGainNode as any)).not.toThrow();
      expect(mockOscillatorNode.disconnect).toHaveBeenCalledTimes(1); // Still disconnects
    });
  });
  
  describe('ensureOscillatorIsRunning', () => {
    it('should call resume if AudioContext is suspended', () => {
      mockAudioContext.state = 'suspended';
      ensureOscillatorIsRunning(mockAudioContext as any);
      expect(mockAudioContext.resume).toHaveBeenCalledTimes(1);
    });

    it('should not call resume if AudioContext is running', () => {
      mockAudioContext.state = 'running';
      ensureOscillatorIsRunning(mockAudioContext as any);
      expect(mockAudioContext.resume).not.toHaveBeenCalled();
    });
  });
});

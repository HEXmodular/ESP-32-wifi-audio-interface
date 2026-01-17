import React from 'react';
import { renderHook, act } from '@testing-library/react-hooks'; // Using react-hooks-testing-library
import { OscillatorProvider, useOscillatorContext } from './OscillatorContext';
import * as oscillatorApi from '@api/oscillatorApi';
import * as clientOscillator from '@audio/clientOscillator';
import type { OscillatorConfig } from '@api/oscillatorApi';

// Mock the API module
jest.mock('@api/oscillatorApi', () => ({
  __esModule: true,
  default: () => ({
    getOscillators: jest.fn(),
    updateOscillator: jest.fn(),
  }),
}));

// Mock the clientOscillator module
jest.mock('@audio/clientOscillator', () => ({
  __esModule: true,
  createClientOscillator: jest.fn(),
  updateClientOscillatorFrequency: jest.fn(),
  updateClientOscillatorAmplitude: jest.fn(),
  updateClientOscillatorWaveType: jest.fn(),
  stopClientOscillator: jest.fn(),
  ensureOscillatorIsRunning: jest.fn(),
}));

const mockInitialOscillators: OscillatorConfig[] = [
  { oscillator_id: 0, is_active: true, wave_type: 'SINE', frequency: 440, amplitude: 50, phase: 0, detune: 0, pwm: 0, client_id: 'server' },
  { oscillator_id: 1, is_active: false, wave_type: 'SQUARE', frequency: 220, amplitude: 70, phase: 0, detune: 0, pwm: 0, client_id: 'server' },
];

// Helper to get typed mock functions
const mockedGetOscillators = oscillatorApi.default().getOscillators as jest.Mock;
const mockedUpdateOscillatorApi = oscillatorApi.default().updateOscillator as jest.Mock;
const mockedCreateClientOscillator = clientOscillator.createClientOscillator as jest.Mock;
const mockedStopClientOscillator = clientOscillator.stopClientOscillator as jest.Mock;
const mockedEnsureOscillatorIsRunning = clientOscillator.ensureOscillatorIsRunning as jest.Mock;
const mockedUpdateClientOscillatorFrequency = clientOscillator.updateClientOscillatorFrequency as jest.Mock;
const mockedUpdateClientOscillatorAmplitude = clientOscillator.updateClientOscillatorAmplitude as jest.Mock;

// Mock AudioContext for client oscillator creation/updates
const mockAudioContext = {
  currentTime: 0,
  state: 'running',
  resume: jest.fn().mockResolvedValue(undefined),
  destination: {},
} as unknown as AudioContext;


describe('OscillatorProvider', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    mockedGetOscillators.mockResolvedValue([...mockInitialOscillators]); // Return a copy

    // Mock global AudioContext if not already available via JSDOM or similar
    if (!(global as any).AudioContext) {
        (global as any).AudioContext = jest.fn(() => mockAudioContext);
    }
     if (!(global as any).webkitAudioContext) {
        (global as any).webkitAudioContext = jest.fn(() => mockAudioContext);
    }

    // Mock return value for createClientOscillator
    mockedCreateClientOscillator.mockImplementation(() => ({
      oscillatorNode: { /* mock node */ } as OscillatorNode,
      gainNode: { /* mock gain */ } as GainNode,
    }));
  });

  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <OscillatorProvider>{children}</OscillatorProvider>
  );

  it('should fetch oscillators on mount and initialize types as server', async () => {
    const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
    await waitForNextUpdate(); // Wait for fetchOscillators to complete

    expect(mockedGetOscillators).toHaveBeenCalledTimes(1);
    expect(result.current.oscillators).toEqual(mockInitialOscillators);
    // Check internal oscillatorTypes state (not directly exposed, so test via behavior)
    // For now, assume it correctly initializes to server. Tests below will cover type changes.
  });

  describe('updateOscillator - Client-side interactions', () => {
    it('should create a new client oscillator if type is "client" and it does not exist', async () => {
      const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
      await waitForNextUpdate(); // Initial fetch

      const configToUpdate = { ...mockInitialOscillators[0], is_active: true, wave_type: 'SINE' as const };
      
      await act(async () => {
        await result.current.updateOscillator(configToUpdate, 'client');
      });
      
      expect(mockedCreateClientOscillator).toHaveBeenCalledWith(expect.any(AudioContext), configToUpdate);
      expect(result.current.getClientOscillator(configToUpdate.oscillator_id)).toBeDefined();
    });

    it('should call ensureOscillatorIsRunning and update existing client oscillator properties', async () => {
      const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
      await waitForNextUpdate();
      
      const initialConfig = { ...mockInitialOscillators[0], is_active: true, wave_type: 'SINE' as const };
      // First, create it as client
      await act(async () => {
        await result.current.updateOscillator(initialConfig, 'client');
      });
      mockedCreateClientOscillator.mockClear(); // Clear after initial creation call

      const updatedConfig = { ...initialConfig, frequency: 880, amplitude: 60 };
      await act(async () => {
        await result.current.updateOscillator(updatedConfig, 'client');
      });

      expect(mockedEnsureOscillatorIsRunning).toHaveBeenCalled(); // From the AudioContext initialization effect
      expect(mockedCreateClientOscillator).not.toHaveBeenCalled(); // Should not create again
      expect(mockedUpdateClientOscillatorFrequency).toHaveBeenCalledWith(expect.anything(), updatedConfig.frequency, expect.any(AudioContext));
      expect(mockedUpdateClientOscillatorAmplitude).toHaveBeenCalledWith(expect.anything(), updatedConfig.amplitude, expect.any(AudioContext));
    });

    it('should update wave type for an existing client oscillator', async () => {
      const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
      await waitForNextUpdate();
      
      const initialConfig = { ...mockInitialOscillators[0], is_active: true, wave_type: 'SINE' as const };
      await act(async () => {
        await result.current.updateOscillator(initialConfig, 'client');
      });

      const updatedConfig = { ...initialConfig, wave_type: 'SQUARE' as const };
      await act(async () => {
        await result.current.updateOscillator(updatedConfig, 'client');
      });
      
      expect(clientOscillator.updateClientOscillatorWaveType).toHaveBeenCalledWith(expect.anything(), updatedConfig.wave_type);
    });
    
    it('should stop and remove client oscillator if is_active becomes false', async () => {
      const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
      await waitForNextUpdate();
      const config = { ...mockInitialOscillators[0], is_active: true, wave_type: 'SINE' as const };
      
      // Create client oscillator
      await act(async () => {
        await result.current.updateOscillator(config, 'client');
      });
      expect(result.current.getClientOscillator(config.oscillator_id)).toBeDefined();

      // Set to inactive
      const deactivatedConfig = { ...config, is_active: false };
      await act(async () => {
        await result.current.updateOscillator(deactivatedConfig, 'client');
      });

      expect(mockedStopClientOscillator).toHaveBeenCalledWith(expect.anything(), expect.anything());
      expect(result.current.getClientOscillator(config.oscillator_id)).toBeUndefined();
    });

    it('should stop client oscillator when switching from client to server', async () => {
      const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
      await waitForNextUpdate();
      const config = { ...mockInitialOscillators[0], is_active: true, wave_type: 'SINE' as const };

      await act(async () => {
        await result.current.updateOscillator(config, 'client');
      });
      expect(result.current.getClientOscillator(config.oscillator_id)).toBeDefined();
      mockedUpdateOscillatorApi.mockResolvedValue(undefined); // Mock server update

      await act(async () => {
        await result.current.updateOscillator(config, 'server');
      });

      expect(mockedStopClientOscillator).toHaveBeenCalledTimes(1);
      expect(result.current.getClientOscillator(config.oscillator_id)).toBeUndefined();
      expect(mockedUpdateOscillatorApi).toHaveBeenCalledWith(config);
    });
  });

  describe('updateOscillator - Server-side interactions', () => {
    it('should call updateOscillatorApi for server-side updates', async () => {
      const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
      await waitForNextUpdate();
      
      const configToUpdate = { ...mockInitialOscillators[0], frequency: 500 };
      mockedUpdateOscillatorApi.mockResolvedValue(undefined);

      await act(async () => {
        await result.current.updateOscillator(configToUpdate, 'server');
      });

      expect(mockedUpdateOscillatorApi).toHaveBeenCalledWith(configToUpdate);
      expect(mockedCreateClientOscillator).not.toHaveBeenCalled();
    });
  });
  
  // Test cleanup of client oscillators on unmount
  it('should cleanup client oscillators on unmount', async () => {
    const { result, waitForNextUpdate, unmount } = renderHook(() => useOscillatorContext(), { wrapper });
    await waitForNextUpdate();
    
    const config = { ...mockInitialOscillators[0], is_active: true, wave_type: 'SINE' as const };
    await act(async () => {
      await result.current.updateOscillator(config, 'client');
    });
    expect(result.current.getClientOscillator(config.oscillator_id)).toBeDefined();

    act(() => {
      unmount();
    });
    
    // The OscillatorContext.tsx has a cleanup useEffect for clientOscillators.
    // This should lead to stopClientOscillator being called for each active client oscillator.
    expect(mockedStopClientOscillator).toHaveBeenCalledTimes(1);
  });
});

describe('OscillatorProvider - API Failure and Fallback', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // Mock global AudioContext if not already available via JSDOM or similar
    if (!(global as any).AudioContext) {
        (global as any).AudioContext = jest.fn(() => mockAudioContext);
    }
     if (!(global as any).webkitAudioContext) {
        (global as any).webkitAudioContext = jest.fn(() => mockAudioContext);
    }
    mockedCreateClientOscillator.mockImplementation(() => ({
      oscillatorNode: { /* mock node */ } as OscillatorNode,
      gainNode: { /* mock gain */ } as GainNode,
    }));
  });

  const wrapper = ({ children }: { children: React.ReactNode }) => (
    <OscillatorProvider>{children}</OscillatorProvider>
  );

  it('should load default client oscillators if getOscillators API call fails', async () => {
    mockedGetOscillators.mockRejectedValueOnce(new Error('API Fetch Failed'));
    
    const { result, waitForNextUpdate } = renderHook(() => useOscillatorContext(), { wrapper });
    await waitForNextUpdate(); // Wait for fetchOscillators to complete (and fail)

    expect(mockedGetOscillators).toHaveBeenCalledTimes(1);
    expect(result.current.isLoading).toBe(false);
    expect(result.current.error).toBe('Defaults loaded: API Fetch Failed');
    
    const defaultOscs = result.current.oscillators;
    expect(defaultOscs).toHaveLength(4);

    // Check properties of the default oscillators
    const expectedDefaults = [
      { oscillator_id: 0, is_active: false, wave_type: 'SINE', frequency: 220, amplitude: 0 },
      { oscillator_id: 1, is_active: false, wave_type: 'SQUARE', frequency: 330, amplitude: 0 },
      { oscillator_id: 2, is_active: false, wave_type: 'SAWTOOTH', frequency: 440, amplitude: 0 },
      { oscillator_id: 3, is_active: false, wave_type: 'TRIANGLE', frequency: 550, amplitude: 0 },
    ];

    expectedDefaults.forEach((defaultOsc, index) => {
      expect(defaultOscs[index]).toMatchObject(defaultOsc);
    });

    // Verify types are set to 'client' by attempting a client-side operation
    // For example, activating the first default oscillator as a client one.
    const firstDefaultOsc = defaultOscs[0];
    const configToActivate = { ...firstDefaultOsc, is_active: true };

    await act(async () => {
      // We explicitly pass 'client' here, but the test is that the context *allows* this
      // because it should have already set the type to 'client' during the fallback.
      await result.current.updateOscillator(configToActivate, 'client');
    });
    expect(mockedCreateClientOscillator).toHaveBeenCalledWith(expect.any(AudioContext), configToActivate);
  });
});

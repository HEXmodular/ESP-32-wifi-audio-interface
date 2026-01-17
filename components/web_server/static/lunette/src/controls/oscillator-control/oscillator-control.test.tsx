import React from 'react';
import { render, fireEvent, screen, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import OscillatorControl from './oscillator-control'; // Adjust path as necessary
import { useOscillatorContext } from '@contexts/OscillatorContext';
import type { OscillatorConfig, WaveType } from '@api/oscillatorApi';

// Mock OscillatorContext
const mockUpdateOscillator = jest.fn();
const mockGetClientOscillator = jest.fn();
jest.mock('@contexts/OscillatorContext', () => ({
  useOscillatorContext: jest.fn(),
}));

// Mock child components
jest.mock('@controls/value-control/value-control', () => (props: any) => (
  <div data-testid={`value-control-${props.id}`}>
    <span>{props.label}</span>
    <input
      type="number"
      value={props.value}
      onChange={(e) => props.onChange(parseFloat(e.target.value))}
    />
  </div>
));

// Functional mock for SelectControl to capture onChange and simulate interaction
const mockSelectControlOnChange = jest.fn();
jest.mock('@controls/select-control/select-control', () => (props: any) => {
  mockSelectControlOnChange.mockImplementation((id, index, isSelected) => {
    props.onChange(id, index, isSelected);
  });
  return (
    <div data-testid={`select-control-${props.id}`}>
      <span>SelectControl Mock</span>
      {/* Simulate selection by calling the passed onChange */}
      {/* Test will need to call mockSelectControlOnChange with index */}
    </div>
  );
});


const baseConfig: OscillatorConfig = {
  oscillator_id: 0,
  is_active: true,
  wave_type: 'SINE',
  frequency: 440,
  amplitude: 50,
  phase: 0,
  detune: 0,
  pwm: 0,
};

const mockOscillatorsState: OscillatorConfig[] = [baseConfig];
const mockOscillatorTypesState: Record<number, 'server' | 'client'> = {
  [baseConfig.oscillator_id]: 'server',
};


describe('OscillatorControl', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    (useOscillatorContext as jest.Mock).mockReturnValue({
      updateOscillator: mockUpdateOscillator,
      oscillators: mockOscillatorsState,
      oscillatorTypes: mockOscillatorTypesState,
      getClientOscillator: mockGetClientOscillator, // Though not directly used in OscillatorControl
      isLoading: false,
      error: null,
    });
    mockUpdateOscillator.mockResolvedValue(undefined); // Default mock implementation
  });

  it('renders all controls correctly', () => {
    render(<OscillatorControl config={baseConfig} showLabel={true} />);
    expect(screen.getByTestId(`value-control-${baseConfig.oscillator_id}-frequency`)).toBeInTheDocument();
    expect(screen.getByTestId(`value-control-${baseConfig.oscillator_id}-note`)).toBeInTheDocument();
    expect(screen.getByTestId(`value-control-${baseConfig.oscillator_id}-amplitude`)).toBeInTheDocument();
    expect(screen.getByTestId(`select-control-${baseConfig.oscillator_id}-type`)).toBeInTheDocument();
  });

  it('calls updateOscillator with correct frequency when frequency ValueControl changes', async () => {
    render(<OscillatorControl config={baseConfig} />);
    const frequencyInput = screen.getByTestId(`value-control-${baseConfig.oscillator_id}-frequency`).querySelector('input')!;
    
    await act(async () => {
      fireEvent.change(frequencyInput, { target: { value: '500' } });
    });

    expect(mockUpdateOscillator).toHaveBeenCalledWith(
      expect.objectContaining({ oscillator_id: baseConfig.oscillator_id, frequency: 500, amplitude: baseConfig.amplitude }),
      'server' // Default type from mockOscillatorTypesState
    );
  });

  it('calls updateOscillator with correct amplitude when amplitude ValueControl changes', async () => {
    render(<OscillatorControl config={baseConfig} />);
    const amplitudeInput = screen.getByTestId(`value-control-${baseConfig.oscillator_id}-amplitude`).querySelector('input')!;

    await act(async () => {
      fireEvent.change(amplitudeInput, { target: { value: '75' } });
    });
    
    expect(mockUpdateOscillator).toHaveBeenCalledWith(
      expect.objectContaining({ oscillator_id: baseConfig.oscillator_id, amplitude: 75, frequency: baseConfig.frequency }),
      'server' 
    );
  });

  it('calls updateOscillator with correct type and wave_type when SelectControl changes type to client-sine', async () => {
    render(<OscillatorControl config={baseConfig} />);
    
    // OSCILLATOR_TYPES imported by oscillator-control.tsx:
    // { value: 'server', label: 'Server Default' }, -> index 0
    // { value: 'client-sine', label: 'Client Sine' }, -> index 1
    // ...
    const targetTypeIndex = 1; // Index for 'client-sine'
    const expectedCombinedType = 'client-sine'; // OSCILLATOR_TYPES[targetTypeIndex].value

    await act(async () => {
      // Simulate SelectControl's onChange being triggered
      // The mockSelectControlOnChange is set up to call the props.onChange of the mocked SelectControl
      mockSelectControlOnChange(`${baseConfig.oscillator_id}-type`, targetTypeIndex, true);
    });

    expect(mockUpdateOscillator).toHaveBeenCalledWith(
      expect.objectContaining({ 
        oscillator_id: baseConfig.oscillator_id, 
        wave_type: 'SINE' // from parseCombinedType('client-sine')
      }),
      'client' // from parseCombinedType('client-sine')
    );
  });
  
  it('calls updateOscillator correctly when type is changed to server', async () => {
    // Setup initial state in context to be client
     (useOscillatorContext as jest.Mock).mockReturnValue({
      updateOscillator: mockUpdateOscillator,
      oscillators: [{ ...baseConfig, wave_type: 'SQUARE' as WaveType }], // Current wave_type
      oscillatorTypes: { [baseConfig.oscillator_id]: 'client' },
      getClientOscillator: mockGetClientOscillator,
      isLoading: false,
      error: null,
    });

    render(<OscillatorControl config={{ ...baseConfig, wave_type: 'SQUARE' as WaveType }} />);

    const targetTypeIndex = 0; // Index for 'server'
    
    await act(async () => {
      mockSelectControlOnChange(`${baseConfig.oscillator_id}-type`, targetTypeIndex, true);
    });

    expect(mockUpdateOscillator).toHaveBeenCalledWith(
      expect.objectContaining({
        oscillator_id: baseConfig.oscillator_id,
        wave_type: 'SQUARE', // Should retain current wave_type when switching to server
      }),
      'server'
    );
  });

  it('updates frequency correctly after type has been changed to client', async () => {
    // Initial state is server
    render(<OscillatorControl config={baseConfig} />);
    
    // 1. Change type to client-sawtooth (index 3)
    const clientSawtoothIndex = 3;
    await act(async () => {
      mockSelectControlOnChange(`${baseConfig.oscillator_id}-type`, clientSawtoothIndex, true);
    });

    // Verify the type change call
    expect(mockUpdateOscillator).toHaveBeenCalledWith(
      expect.objectContaining({ wave_type: 'SAWTOOTH' }),
      'client'
    );
    mockUpdateOscillator.mockClear(); // Clear mocks for the next call verification

    // 2. Change frequency
    const frequencyInput = screen.getByTestId(`value-control-${baseConfig.oscillator_id}-frequency`).querySelector('input')!;
    await act(async () => {
      fireEvent.change(frequencyInput, { target: { value: '600' } });
    });
    
    // Expect updateOscillator to be called with 'client' type
    expect(mockUpdateOscillator).toHaveBeenCalledWith(
      expect.objectContaining({ oscillator_id: baseConfig.oscillator_id, frequency: 600, wave_type: 'SAWTOOTH' }),
      'client' 
    );
  });
});

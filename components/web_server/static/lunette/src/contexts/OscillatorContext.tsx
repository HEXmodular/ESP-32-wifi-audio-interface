import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from 'react';
import type { OscillatorConfig } from '@api/oscillatorApi';
import useOscillatorApi from '@api/oscillatorApi';

// Define a type for the client-side oscillator instances
type ClientOscillator = {
  node: OscillatorNode;
  gainNode: GainNode;
};

interface OscillatorContextType {
  oscillators: OscillatorConfig[];
  updateOscillator: (config: OscillatorConfig, type?: 'server' | 'client') => Promise<void>;
  isLoading: boolean;
  error: string | null;
  getClientOscillator: (id: number) => ClientOscillator | undefined;
}

const OscillatorContext = createContext<OscillatorContextType | null>(null);

export const useOscillatorContext = () => {
  const context = useContext(OscillatorContext);
  if (!context) {
    throw new Error('useOscillatorContext must be used within an OscillatorProvider');
  }
  return context;
};

export const OscillatorProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [oscillators, setOscillators] = useState<OscillatorConfig[]>([]);
  // State for client-side oscillator instances
  const [clientOscillators, setClientOscillators] = useState<Record<number, ClientOscillator>>({});
  // State to track oscillator types (server or client)
  const [oscillatorTypes, setOscillatorTypes] = useState<Record<number, 'server' | 'client'>>({});
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const { getOscillators, updateOscillator: updateOscillatorApi } = useOscillatorApi();
  const dataFetching = useRef({ oscillators: false });
  const audioContextRef = useRef<AudioContext | null>(null);

  // Initialize AudioContext
  useEffect(() => {
    if (typeof window !== 'undefined' && !audioContextRef.current) {
      audioContextRef.current = new (window.AudioContext || (window as any).webkitAudioContext)();
    }
    // Cleanup AudioContext on component unmount
    return () => {
      if (audioContextRef.current) {
        audioContextRef.current.close();
        audioContextRef.current = null;
      }
    };
  }, []);

  const fetchOscillators = useCallback(async () => {
    if (dataFetching.current.oscillators) return;

    try {
      dataFetching.current.oscillators = true;
      setIsLoading(true);
      setError(null);
      const data = await getOscillators();
      setOscillators(data);
      // Initialize oscillator types as server-side by default
      const initialTypes: Record<number, 'server' | 'client'> = {};
      data.forEach(osc => {
        initialTypes[osc.oscillator_id] = 'server';
      });
      setOscillatorTypes(initialTypes);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Failed to fetch oscillators from server.';
      console.warn(`${errorMsg} Using default client-side oscillators.`);

      const defaultOscs: OscillatorConfig[] = [
        { oscillator_id: 0, is_active: false, wave_type: 'SINE', frequency: 220, amplitude: 0, phase: 0, detune: 0, pwm: 0 },
        { oscillator_id: 1, is_active: false, wave_type: 'SQUARE', frequency: 330, amplitude: 0, phase: 0, detune: 0, pwm: 0 },
        { oscillator_id: 2, is_active: false, wave_type: 'SAWTOOTH', frequency: 440, amplitude: 0, phase: 0, detune: 0, pwm: 0 },
        { oscillator_id: 3, is_active: false, wave_type: 'TRIANGLE', frequency: 550, amplitude: 0, phase: 0, detune: 0, pwm: 0 },
      ];
      setOscillators(defaultOscs);

      const defaultTypes: Record<number, 'server' | 'client'> = {};
      defaultOscs.forEach(osc => {
        defaultTypes[osc.oscillator_id] = 'client';
      });
      setOscillatorTypes(defaultTypes);
      
      setError(`Defaults loaded: ${errorMsg}`); // Set a non-critical error message
    } finally {
      setIsLoading(false);
    }
  }, [getOscillators]);

  const updateOscillator = useCallback(async (config: OscillatorConfig, type: 'server' | 'client' = 'server') => {
    setOscillatorTypes(prevTypes => ({
      ...prevTypes,
      [config.oscillator_id]: type,
    }));

    if (type === 'client' && audioContextRef.current) {
      const audioCtx = audioContextRef.current;
      setClientOscillators(prevClientOscs => {
        const existingClientOsc = prevClientOscs[config.oscillator_id];
        
        if (existingClientOsc) {
          // Update existing client oscillator
          existingClientOsc.node.frequency.setValueAtTime(config.frequency, audioCtx.currentTime);
          existingClientOsc.gainNode.gain.setValueAtTime(config.amplitude / 100, audioCtx.currentTime); // Assuming amplitude is 0-100
          if (config.is_active && existingClientOsc.node.context.state === 'suspended') {
            existingClientOsc.node.context.resume();
          } else if (!config.is_active && existingClientOsc.node.context.state === 'running') {
            // This is tricky with OscillatorNode, it can't be paused and resumed in the traditional sense.
            // For simplicity, we'll stop and remove it. A more robust solution might involve disconnecting/reconnecting.
            existingClientOsc.node.stop();
            existingClientOsc.node.disconnect();
            existingClientOsc.gainNode.disconnect();
            const newClientOscs = { ...prevClientOscs };
            delete newClientOscs[config.oscillator_id];
            return newClientOscs;
          }
        } else if (config.is_active) {
          // Create new client oscillator
          const oscillatorNode = audioCtx.createOscillator();
          const gainNode = audioCtx.createGain();
          
          oscillatorNode.type = config.wave_type.toLowerCase() as OscillatorType;
          oscillatorNode.frequency.setValueAtTime(config.frequency, audioCtx.currentTime);
          gainNode.gain.setValueAtTime(config.amplitude / 100, audioCtx.currentTime); // Assuming amplitude is 0-100
          
          oscillatorNode.connect(gainNode);
          gainNode.connect(audioCtx.destination);
          oscillatorNode.start();
          
          return {
            ...prevClientOscs,
            [config.oscillator_id]: { node: oscillatorNode, gainNode },
          };
        }
        return prevClientOscs;
      });
       // Update local state for client-side changes
      setOscillators(prev =>
        prev.map(osc =>
          osc.oscillator_id === config.oscillator_id ? config : osc
        )
      );

    } else { // server-side
      try {
        await updateOscillatorApi(config);
        setOscillators(prev =>
          prev.map(osc =>
            osc.oscillator_id === config.oscillator_id ? config : osc
          )
        );
        // If it was client-side, remove it from client oscillators
        setClientOscillators(prevClientOscs => {
          const existingClientOsc = prevClientOscs[config.oscillator_id];
          if (existingClientOsc) {
            existingClientOsc.node.stop();
            existingClientOsc.node.disconnect();
            existingClientOsc.gainNode.disconnect();
            const newClientOscs = { ...prevClientOscs };
            delete newClientOscs[config.oscillator_id];
            return newClientOscs;
          }
          return prevClientOscs;
        });
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Failed to update server-side oscillator');
      }
    }
  }, [updateOscillatorApi]);

  const getClientOscillator = useCallback((id: number) => {
    return clientOscillators[id];
  }, [clientOscillators]);

  // useEffect(() => {
  //   fetchOscillators();
  // }, [fetchOscillators]);

  // Cleanup client oscillators when component unmounts or oscillators are removed
  useEffect(() => {
    return () => {
      if (audioContextRef.current) {
        Object.values(clientOscillators).forEach(clientOsc => {
          clientOsc.node.stop();
          clientOsc.node.disconnect();
          clientOsc.gainNode.disconnect();
        });
        setClientOscillators({});
      }
    };
  }, [clientOscillators]);


  const value = {
    oscillators,
    updateOscillator,
    isLoading,
    error,
    getClientOscillator
  };

  return (
    <OscillatorContext.Provider value={value}>
      {children}
    </OscillatorContext.Provider>
  );
};
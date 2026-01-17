// write useWebSocketAudioInput hook
import audioWorkletUrl from '@worklets/audio-worklet.js?url';
import { useEffect, useRef, useCallback, useState } from 'react';

export const useWebSocketAudioInput = (context: AudioContext | null, wsUrl: string) => {
    const ws = useRef<WebSocket | null>(null);
    const audioContext = useRef<AudioContext | null>(null);
    const audioWorkletNode = useRef<AudioWorkletNode | null>(null);
    const [audioWorkletNodeState, setAudioWorkletNodeState] = useState<AudioWorkletNode | null>(null);

    const initAudio = async() => {
        try {
            console.log('initAudio audioContext',audioContext);
            if (!context) {
                console.error('WebSocketAudioInput: Audio context is not initialized', "initAudio");
                return;
            }
            audioContext.current = context;

            // Load and register our audio worklet
            await audioContext.current.audioWorklet.addModule(audioWorkletUrl);
            console.log('AudioWorklet module loaded.');

            // Create audio worklet node
            audioWorkletNode.current = new AudioWorkletNode(audioContext.current, 'audio-processor');
            setAudioWorkletNodeState(audioWorkletNode.current);

        } catch (error) {
            console.error('Failed to initialize audio:', error);
        }
    };

    const handleAudioData = useCallback(async (blob: Blob) => {
        // if (!audioContext.current || !audioWorkletNode.current) {
        //     await initAudio();
        // }

        const arrayBuffer = await blob.arrayBuffer();
        const int8Array = new Int8Array(arrayBuffer);

        // Convert int8 samples (-128 to 127) to float32 (-1.0 to 1.0)
        const float32Array = new Float32Array(int8Array.length);
        for (let i = 0; i < int8Array.length; i++) {
            float32Array[i] = int8Array[i] / 128.0;
        }

        // Send audio data to the worklet
        if (audioWorkletNode.current) {
            audioWorkletNode.current.port.postMessage({
                audioData: float32Array,
                sampleRate: 10000 // ESP32 sample rate
            });
        }
    }, []);

    useEffect(() => {
        console.log('WebSocketAudioInput:', "useEffect");
        if (!context) {
            console.error('WebSocketAudioInput: Audio context is not initialized', "useEffect");
            return;
        }

        (async () => {
            if (!audioContext.current || !audioWorkletNode.current) {
                await initAudio();
            }
        })();

        // Initialize WebSocket
        ws.current = new WebSocket(wsUrl);

        ws.current.onopen = () => {
            console.log('Connected to WebSocket server');
        };

        ws.current.onmessage = async (event: MessageEvent) => {
            if (event.data instanceof Blob) {
                await handleAudioData(event.data);
            }
        };

        ws.current.onclose = () => {
            console.log('Disconnected from WebSocket server');
        };

        // Cleanup on unmount
        return () => {
            if (ws.current) {
                ws.current.close();
            }
            if (audioWorkletNode.current) {
                audioWorkletNode.current.disconnect();
            }
        };
    }, [context, handleAudioData, wsUrl]);

    return {
        audioWorkletNode: audioWorkletNodeState,
    };
};

export default useWebSocketAudioInput;

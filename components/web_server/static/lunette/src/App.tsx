// import viteLogo from '/vite.svg'
import React from 'react'
import { useEffect, useState } from 'react'
import { useWebSocketAudioInput } from '@audio/webSocketAudioInput'

import { SwipeScreensControl } from '@controls/swipe-screens-control/swipe-screens-control'

import MainScreen from '@screens/main-screen'
import EffectsScreen from '@screens/effects-screen'
import { KeyboardScreen } from '@screens/keyboard-screen'
import { SequencerScreen } from '@screens/sequencer-screen'
import SequencerMagenta from '@/screens/sequencer-magenta/sequencer-magenta'

import { OscillatorProvider } from '@contexts/OscillatorContext'
import { TouchProvider, useTouch } from '@contexts/TouchContext'
import { EffectsProvider } from '@contexts/EffectsContext'

import './App.css'


const AppContent: React.FC = () => {
  const wsUrl = import.meta.env.DEV ? 'https://lunette.local/ws' : '/ws';

  const [audioContext, setAudioContext] = useState<AudioContext | null>(null);
  const [audioEngineStarted, setAudioEngineStarted] = useState(false);
  const { audioWorkletNode } = useWebSocketAudioInput(audioContext, wsUrl);

  const { isTouching } = useTouch();

  useEffect(() => {
    if (isTouching) {
      document.documentElement.style.overflow = 'hidden';
    } else {
      document.documentElement.style.overflow = 'auto';
    }
  }, [isTouching]);

  useEffect(() => {
    console.log('audioEngineStarted', audioEngineStarted);
    if (audioEngineStarted) {
      const audioContextRef = new AudioContext();
      audioContextRef.addEventListener('error', (event) => {
        console.error('AudioContext error', event);
      });
      setAudioContext(audioContextRef);
    }
  }, [audioEngineStarted]);


  return (
    <OscillatorProvider>
      <EffectsProvider audioContext={audioContext}>
      <div className="app">
        <div className="content-block">
          {!audioEngineStarted && <button onClick={() => {
            setAudioEngineStarted(true);
          }}>START AUDIO ENGINE</button>}
        </div>
        <SwipeScreensControl initialScreen={0} onScreenChange={(index) => console.log(`Switched to screen ${index}`)}>
          <SequencerMagenta />
          <KeyboardScreen />
          <MainScreen />
          <SequencerScreen />
          <div>
              <EffectsScreen inputNode={audioWorkletNode} outputNode={audioContext?.destination} audioContext={audioContext} />
          </div>
        </SwipeScreensControl>
      </div>
      </EffectsProvider>
    </OscillatorProvider>
  );
};

const App: React.FC = () => {
  return (
    <TouchProvider>
      <AppContent />
    </TouchProvider>
  );
};

export default App;

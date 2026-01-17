// Web Audio API
// make room reverb using two delay nodes
// after each delay node add allpass filters
// summ allpass filters outputs
// add lowpass filter to summ output
// add feedback to input of reverb

class ReverbProcessor extends AudioWorkletProcessor {
    constructor() {
        super();
        
        // Initialize buffers for delays
        this.delayBuffer1 = new Float32Array(48000); // 1 second at 48kHz
        this.delayBuffer2 = new Float32Array(48000);
        this.delayIndex1 = 0;
        this.delayIndex2 = 0;
        
        // Initialize allpass filter states
        this.allpass1 = { x1: 0, y1: 0 };
        this.allpass2 = { x1: 0, y1: 0 };
        this.allpass3 = { x1: 0, y1: 0 };
        this.allpass4 = { x1: 0, y1: 0 };
        
        // Initialize lowpass filter state
        this.lowpass = { y1: 0 };
        
        // Default parameters
        this.parameters = {
            delayTime1: 0.1,
            delayTime2: 0.15,
            allpassFreq1: 1000,
            allpassFreq2: 2000,
            allpassFreq3: 3000,
            allpassFreq4: 4000,
            lowpassFreq: 2000,
            feedbackGain: 0.5
        };
        
        // Calculate delay buffer sizes
        this.delaySize1 = Math.floor(this.parameters.delayTime1 * sampleRate);
        this.delaySize2 = Math.floor(this.parameters.delayTime2 * sampleRate);
        
        // Calculate filter coefficients
        this.updateFilterCoefficients();
        
        // Handle parameter updates from main thread
        this.port.onmessage = (event) => {
            if (event.data.type === 'parameters') {
                this.parameters = { ...this.parameters, ...event.data.parameters };
                this.updateFilterCoefficients();
                this.delaySize1 = Math.floor(this.parameters.delayTime1 * sampleRate);
                this.delaySize2 = Math.floor(this.parameters.delayTime2 * sampleRate);
            }
        };
    }
    
    updateFilterCoefficients() {
        // Calculate allpass filter coefficients
        this.allpassCoeffs = [
            this.calculateAllpassCoeff(this.parameters.allpassFreq1),
            this.calculateAllpassCoeff(this.parameters.allpassFreq2),
            this.calculateAllpassCoeff(this.parameters.allpassFreq3),
            this.calculateAllpassCoeff(this.parameters.allpassFreq4)
        ];
        
        // Calculate lowpass filter coefficient
        this.lowpassCoeff = this.calculateLowpassCoeff(this.parameters.lowpassFreq);
    }
    
    calculateAllpassCoeff(frequency) {
        const w0 = 2 * Math.PI * frequency / sampleRate;
        const alpha = Math.sin(w0) / (2 * Math.cos(w0));
        return (1 - alpha) / (1 + alpha);
    }
    
    calculateLowpassCoeff(frequency) {
        const w0 = 2 * Math.PI * frequency / sampleRate;
        return Math.exp(-w0);
    }
    
    processAllpass(input, coeff, state) {
        const output = coeff * input + state.x1 - coeff * state.y1;
        state.x1 = input;
        state.y1 = output;
        return output;
    }
    
    processLowpass(input, state) {
        const output = this.lowpassCoeff * state.y1 + (1 - this.lowpassCoeff) * input;
        state.y1 = output;
        return output;
    }
    
    process(inputs, outputs, parameters) {
        const input = inputs[0];
        const output = outputs[0];
        
        for (let channel = 0; channel < input.length; channel++) {
            const inputChannel = input[channel];
            const outputChannel = output[channel];
            
            for (let i = 0; i < inputChannel.length; i++) {
                // Process input through delays
                const delayed1 = this.delayBuffer1[this.delayIndex1];
                const delayed2 = this.delayBuffer2[this.delayIndex2];
                
                // Update delay buffers
                this.delayBuffer1[this.delayIndex1] = inputChannel[i];
                this.delayBuffer2[this.delayIndex2] = inputChannel[i];
                
                // Update delay indices
                this.delayIndex1 = (this.delayIndex1 + 1) % this.delaySize1;
                this.delayIndex2 = (this.delayIndex2 + 1) % this.delaySize2;
                
                // Process through allpass filters
                const allpassOut1 = this.processAllpass(delayed1, this.allpassCoeffs[0], this.allpass1);
                const allpassOut2 = this.processAllpass(delayed2, this.allpassCoeffs[1], this.allpass2);
                const allpassOut3 = this.processAllpass(allpassOut1, this.allpassCoeffs[2], this.allpass3);
                const allpassOut4 = this.processAllpass(allpassOut2, this.allpassCoeffs[3], this.allpass4);
                
                // Sum allpass outputs
                const summed = allpassOut3 + allpassOut4;
                
                // Process through lowpass filter
                const lowpassed = this.processLowpass(summed, this.lowpass);
                
                // Apply feedback
                const feedback = lowpassed * this.parameters.feedbackGain;
                
                // Mix dry and wet signals
                outputChannel[i] = inputChannel[i] + lowpassed;
            }
        }
        
        return true;
    }
}

// Register the processor
registerProcessor('reverb-processor', ReverbProcessor);

// Helper class for the main thread
class ReverbWorklet {
    constructor(audioContext) {
        this.context = audioContext;
        this.node = null;
        this.init();
    }
    
    async init() {
        await this.context.audioWorklet.addModule('audio-effects.js');
        this.node = new AudioWorkletNode(this.context, 'reverb-processor');
    }
    
    setParameters(params) {
        if (this.node) {
            this.node.port.postMessage({
                type: 'parameters',
                parameters: params
            });
        }
    }
    
    connect(source) {
        if (this.node) {
            source.connect(this.node);
            return this.node;
        }
        return null;
    }
    
    disconnect() {
        if (this.node) {
            this.node.disconnect();
        }
    }
}

export default ReverbWorklet;










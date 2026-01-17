class AudioProcessor extends AudioWorkletProcessor {
    constructor() {
        super();
        this.buffer = new Float32Array(0);
        this.inputSampleRate = 10000; // ESP32 sample rate
        this.port.onmessage = this.handleMessage.bind(this);
    }

    handleMessage(event) {
        const { audioData, sampleRate } = event.data;
        this.inputSampleRate = sampleRate;
        
        // Append new data to buffer
        const newBuffer = new Float32Array(this.buffer.length + audioData.length);
        newBuffer.set(this.buffer);
        newBuffer.set(audioData, this.buffer.length);
        this.buffer = newBuffer;
    }

    process(inputs, outputs, parameters) {
        const output = outputs[0];
        const channel = output[0];
        const outputSampleRate = 48000; // Standard Web Audio sample rate
        const ratio = this.inputSampleRate / outputSampleRate;

        if (this.buffer.length < channel.length * ratio) {
            // Not enough data in buffer
            return true;
        }

        // Linear interpolation for sample rate conversion
        for (let i = 0; i < channel.length; i++) {
            const inputIndex = Math.floor(i * ratio);
            const fraction = i * ratio - inputIndex;
            
            const sample1 = this.buffer[inputIndex] || 0;
            const sample2 = this.buffer[inputIndex + 1] || sample1;
            
            channel[i] = sample1 + fraction * (sample2 - sample1);
        }

        // Remove processed samples from buffer
        const samplesUsed = Math.ceil(channel.length * ratio);
        this.buffer = this.buffer.slice(samplesUsed);

        return true;
    }
}

registerProcessor('audio-processor', AudioProcessor); 
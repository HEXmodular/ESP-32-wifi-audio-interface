export interface SequencerOptions {
    sequenceLength: number;
    timeUnit: number; // in milliseconds
    onStepChange?: (step: number) => void;
}

export class Sequencer {
    private currentStep: number;
    private sequenceLength: number;
    private timeUnit: number;
    private timer: number | null;
    private isPlaying: boolean;
    private onStepChange?: (step: number) => void;

    constructor(options: SequencerOptions) {
        this.currentStep = 0;
        this.sequenceLength = options.sequenceLength;
        this.timeUnit = options.timeUnit;
        this.onStepChange = options.onStepChange;
        this.timer = null;
        this.isPlaying = false;
    }

    public start(): void {
        if (this.isPlaying) return;
        
        this.isPlaying = true;
        this.timer = window.setInterval(() => {
            this.nextStep();
        }, this.timeUnit);
    }

    public stop(): void {
        if (!this.isPlaying) return;

        if (this.timer) {
            window.clearInterval(this.timer);
            this.timer = null;
        }
        this.isPlaying = false;
    }

    public reset(): void {
        this.stop();
        this.currentStep = 0;
        this.onStepChange?.(this.currentStep);
    }

    public nextStep(): void {
        this.currentStep = (this.currentStep + 1) % this.sequenceLength;
        this.onStepChange?.(this.currentStep);
    }

    public setStep(step: number): void {
        if (step >= 0 && step < this.sequenceLength) {
            this.currentStep = step;
            this.onStepChange?.(this.currentStep);
        }
    }

    public getCurrentStep(): number {
        return this.currentStep;
    }

    public isRunning(): boolean {
        return this.isPlaying;
    }

    public setTimeUnit(timeUnit: number): void {
        this.timeUnit = timeUnit;
        if (this.isPlaying) {
            this.stop();
            this.start();
        }
    }

    public destroy(): void {
        this.stop();
    }
} 
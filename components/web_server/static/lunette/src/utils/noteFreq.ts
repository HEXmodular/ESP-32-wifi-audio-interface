export const NOTES = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B'];
export const A4_OCTAVE = 4;
export const A4_MIDI_NOTE = 69;
export const A4_FREQUENCY = 440;

export const getNoteFrequency = (note: string, octave?: number): number => {
    const noteIndex = NOTES.indexOf(note);
    const octaveDiff = octave ? octave - A4_OCTAVE : 0;
    const noteDiff = noteIndex - NOTES.indexOf('A');
    return A4_FREQUENCY * Math.pow(2, octaveDiff + noteDiff / 12);
};

export const getMIDINoteFrequency = (note: number): number => {
    return A4_FREQUENCY * Math.pow(2, (note - A4_MIDI_NOTE) / 12);
};

export const getNoteFromFrequency = (frequency?: number): string => {
    if (!frequency) return '';
    const noteDiff = Math.round(12 * Math.log2(frequency / A4_FREQUENCY));
    const octaveDiff = Math.floor(noteDiff / 12);
    const noteIndex = (noteDiff % 12 + 12) % 12;
    const octave = A4_OCTAVE + octaveDiff;
    return `${NOTES[noteIndex]}${octave}`;
};
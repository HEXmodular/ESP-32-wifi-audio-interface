export class Envelope {
    constructor() {}

    startEnvelope(
        param: AudioParam,
        attackTime: number,
        peakValue: number,
        decayTime: number,
        sustainLevel: number,
        audioContext: AudioContext
    ): void {
    const now = audioContext.currentTime; // Получаем текущее время аудио-контекста
    const paramValue = param.value;
    const paramMaxValue = param.maxValue;

    // --- Шаг 1: Отменить все предыдущие запланированные изменения с текущего момента ---
    // Это ГЛАВНЫЙ шаг для предотвращения конфликтов.
    // Гарантирует, что новая автоматизация начнется чисто, не продолжая старые рампы.
    param.cancelScheduledValues(now);

    // --- Шаг 2: Установить начальное значение в текущий момент ---
    // Часто устанавливают 0 для гейна при начале атаки, но можно и начать от текущего значения,
    // если логика требует плавного перехода от предыдущего состояния.
    // В типичном ADSR гейн начинается с 0.
    param.setValueAtTime(0.0, now); // Установить гейн в 0 в точное время старта

    // --- Шаг 3: Планирование сегмента Атаки ---
    // Линейный или экспоненциальный переход от начального значения (0) к peakValue
    const attackEndTime = now + attackTime;
    param.linearRampToValueAtTime(paramValue + paramMaxValue * peakValue, attackEndTime);
    // param.exponentialRampToValueAtTime(peakValue, attackEndTime); // Альтернатива для экспоненциального роста

    // --- Шаг 4: Планирование сегмента Спада (Decay) ---
    // Линейный или экспоненциальный переход от peakValue к sustainLevel.
    // Начинается сразу после завершения атаки (во время attackEndTime).
    const decayEndTime = attackEndTime + decayTime;
    param.linearRampToValueAtTime(paramValue, decayEndTime);
    // param.exponentialRampToValueAtTime(sustainLevel, decayEndTime); // Альтернатива для экспоненциального спада
    // Или с использованием setTargetAtTime для более естественного спада:
    // param.setTargetAtTime(sustainLevel, attackEndTime, decayTime / 3); // timeConstant примерно 1/3 от желаемой длительности спада
    // При использовании setTargetAtTime decayEndTime как такового нет, значение асимптотически приближается к sustainLevel.

    // --- Шаг 5: Сегмент Удержания (Sustain) ---
    // Если это полная ADSR огибающая, значение теперь будет оставаться на sustainLevel
    // до тех пор, пока не будет вызвана функция "Note Off" или "Release".
    // Никаких дополнительных методов планирования здесь не требуется,
    // значение будет держаться на sustainLevel до следующего вызова методов автоматизации.
}

    stopEnvelope(
        param: AudioParam,
        releaseTime: number,
        audioContext: AudioContext
    ): void {
    const now = audioContext.currentTime; // Получаем текущее время отпускания

    // --- Шаг 1: Отменить все предыдущие запланированные изменения с текущего момента ---
    // Это важно, если нота была отпущена во время атаки или спада,
    // чтобы прервать их и начать сегмент Release с ТЕКУЩЕГО значения.
    param.cancelScheduledValues(now);

     // --- Шаг 2: Установить текущее значение как начальное для Release ---
     // Это гарантирует, что сегмент Release начнется плавно с того значения,
     // которое параметр имел в момент отпускания ноты.
     param.setValueAtTime(param.value, now);

    // --- Шаг 3: Планирование сегмента Спада (Release) ---
    // Линейный или экспоненциальный переход от текущего значения (param.value в момент now) к 0
    const releaseEndTime = now + releaseTime;
    param.linearRampToValueAtTime(0.0, releaseEndTime);
    // param.exponentialRampToValueAtTime(0.00001, releaseEndTime); // Экспоненциальный спад к очень малому числу, чтобы избежать деления на 0
    // Или с использованием setTargetAtTime для более естественного спада:
    // param.setTargetAtTime(0.0, now, releaseTime / 3); // timeConstant примерно 1/3 от желаемой длительности спада
    }
}


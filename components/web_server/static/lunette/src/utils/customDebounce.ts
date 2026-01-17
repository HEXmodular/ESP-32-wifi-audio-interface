type DebouncedFunction<T extends (...args: any[]) => any> = {
  (...args: Parameters<T>): ReturnType<T> | undefined;
  cancel: () => void;
  flush: () => ReturnType<T> | undefined;
};

interface DebounceOptions {
  maxWait?: number; // Maximum wait time before forced execution
}

function customDebounce<T extends (...args: any[]) => any>(
  func: T,
  delay: number,
  options: DebounceOptions = {}
): DebouncedFunction<T> {
  let timeoutId: ReturnType<typeof setTimeout> | null = null;
  let maxWaitTimeoutId: ReturnType<typeof setTimeout> | null = null;
  let lastArgs: Parameters<T> | null = null;
  let lastThis: ThisParameterType<T> | null = null;
  let lastResult: ReturnType<T> | undefined;
  let lastCallTime: number = 0; // Time of last `debounced` function call

  const maxWait = options.maxWait || Infinity; // If maxWait not specified, use infinity

  // Helper function to execute the original function
  function invokeFunc(): void {
    const args = lastArgs!;
    const context = lastThis!;

    lastArgs = null;
    lastThis = null;
    lastResult = func.apply(context, args); // Call original function

    // Reset timers and state after execution
    debounced.cancel(); // Clears both timers
    lastCallTime = 0; // Reset last call time so new series starts clean
  }

  // Function called after main `delay` expires
  function trailingCall(): void {
    invokeFunc();
  }

  // Function called after `maxWait` expires
  function maxWaitCall(): void {
    if (lastArgs) {
      invokeFunc();
    }
  }

  const debounced = function (this: ThisParameterType<T>, ...args: Parameters<T>): ReturnType<T> | undefined {
    const now = Date.now();
    lastArgs = args;
    lastThis = this;

    // If first call in series or enough time passed since last call
    if (lastCallTime === 0 || (now - lastCallTime >= maxWait)) {
      lastCallTime = now;
      if (maxWaitTimeoutId) clearTimeout(maxWaitTimeoutId);
      if (maxWait !== Infinity) {
        maxWaitTimeoutId = setTimeout(maxWaitCall, maxWait);
      }
    }

    // Reset main delay timer
    if (timeoutId) clearTimeout(timeoutId);
    timeoutId = setTimeout(trailingCall, delay);

    return lastResult; // Return result from previous successful call
  } as DebouncedFunction<T>;

  // Method to cancel all pending calls
  debounced.cancel = function (): void {
    if (timeoutId) {
      clearTimeout(timeoutId);
      timeoutId = null;
    }
    if (maxWaitTimeoutId) {
      clearTimeout(maxWaitTimeoutId);
      maxWaitTimeoutId = null;
    }
    lastArgs = null;
    lastThis = null;
    lastCallTime = 0;
  };

  // Method to immediately execute pending call
  debounced.flush = function (): ReturnType<T> | undefined {
    if (lastArgs) {
      invokeFunc();
      return lastResult;
    }
    return undefined;
  };

  return debounced;
}

export default customDebounce;
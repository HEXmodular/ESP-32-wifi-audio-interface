import React, { createContext, useContext, useState, useCallback, type ReactNode } from 'react';

interface TouchContextType {
  isTouching: boolean;
  isLongPressing: boolean;
  touchStartTime: number | null;
  handleTouchStart: (event: React.TouchEvent) => void;
  handleTouchEnd: (event: React.TouchEvent) => void;
  handleTouchMove: (event: React.TouchEvent) => void;
}

const TouchContext = createContext<TouchContextType | undefined>(undefined);

const LONG_PRESS_DURATION = 500; // milliseconds

interface TouchProviderProps {
  children: ReactNode;
}

export const TouchProvider: React.FC<TouchProviderProps> = ({ children }) => {
  const [isTouching, setIsTouching] = useState(false);
  const [isLongPressing, setIsLongPressing] = useState(false);
  const [touchStartTime, setTouchStartTime] = useState<number | null>(null);
  const [longPressTimer, setLongPressTimer] = useState<NodeJS.Timeout | null>(null);

  const handleTouchStart = useCallback(() => {
    setIsTouching(true);
    const startTime = Date.now();
    setTouchStartTime(startTime);

    const timer = setTimeout(() => {
      setIsLongPressing(true);
    }, LONG_PRESS_DURATION);

    setLongPressTimer(timer);
  }, []);

  const handleTouchEnd = useCallback(() => {
    setIsTouching(false);
    setIsLongPressing(false);
    setTouchStartTime(null);
    
    if (longPressTimer) {
      clearTimeout(longPressTimer);
      setLongPressTimer(null);
    }
  }, [longPressTimer]);

  const handleTouchMove = useCallback(() => {
    // Reset long press if user moves their finger
    if (isLongPressing) {
      setIsLongPressing(false);
      if (longPressTimer) {
        clearTimeout(longPressTimer);
        setLongPressTimer(null);
      }
    }
  }, [isLongPressing, longPressTimer]);

  const value = {
    isTouching,
    isLongPressing,
    touchStartTime,
    handleTouchStart,
    handleTouchEnd,
    handleTouchMove,
  };

  return (
    <TouchContext.Provider value={value}>
      {children}
    </TouchContext.Provider>
  );
};

export const useTouch = () => {
  const context = useContext(TouchContext);
  if (context === undefined) {
    throw new Error('useTouch must be used within a TouchProvider');
  }
  return context;
};

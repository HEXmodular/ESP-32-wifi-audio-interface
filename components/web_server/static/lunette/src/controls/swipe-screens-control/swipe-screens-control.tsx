// компонент для свайпа между экранами
// как на мобильных устройствах

import React, { useState, useRef } from 'react';
import type { TouchEvent } from 'react';
import { useTouch } from '@contexts/TouchContext';

import './swipe-screens-control.css';

interface SwipeScreensProps {
  children: React.ReactNode[];
  onScreenChange?: (index: number) => void;
  initialScreen?: number;
}

export const SwipeScreensControl: React.FC<SwipeScreensProps> = ({
  children,
  onScreenChange,
  initialScreen = 0,
}) => {
  const { isTouching } = useTouch();
  const [currentScreen, setCurrentScreen] = useState(initialScreen);
  const [touchStart, setTouchStart] = useState<number | null>(null);
  const [touchEnd, setTouchEnd] = useState<number | null>(null);
  const containerRef = useRef<HTMLDivElement>(null);

  const minSwipeDistance = 50;

  const handleTouchStart = (e: TouchEvent) => {
    if (isTouching) return; // Если touch уже используется другим контролом, игнорируем
    setTouchEnd(null);
    setTouchStart(e.targetTouches[0].clientX);
  };

  const handleTouchMove = (e: TouchEvent) => {
    if (isTouching) return; // Если touch уже используется другим контролом, игнорируем
    setTouchEnd(e.targetTouches[0].clientX);
  };

  const handleTouchEnd = () => {
    if (isTouching) return; // Если touch уже используется другим контролом, игнорируем
    
    if (!touchStart || !touchEnd) return;

    const distance = touchStart - touchEnd;
    const isLeftSwipe = distance > minSwipeDistance;
    const isRightSwipe = distance < -minSwipeDistance;

    if (isLeftSwipe && currentScreen < React.Children.count(children) - 1) {
      const newScreen = currentScreen + 1;
      setCurrentScreen(newScreen);
      onScreenChange?.(newScreen);
    } else if (isRightSwipe && currentScreen > 0) {
      const newScreen = currentScreen - 1;
      setCurrentScreen(newScreen);
      onScreenChange?.(newScreen);
    }
  };

  return (
    <div
      ref={containerRef}
      className="swipe-screens-container"
      onTouchStart={handleTouchStart}
      onTouchMove={handleTouchMove}
      onTouchEnd={handleTouchEnd}
    >
      {React.Children.map(children, (child, index) => (
        <div
          key={index}
          className="swipe-screen"
          style={{
            width: '100%',
            height: currentScreen === index ? '100%' : 0,
            opacity: currentScreen === index ? 1 : 0,
            transition: 'opacity 0.3s ease-in-out',
            pointerEvents: currentScreen === index ? 'auto' : 'none'
          }}
        >
          {child}
        </div>
      ))}
    </div>
  );
};



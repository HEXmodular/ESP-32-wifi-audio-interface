import React from 'react';
import { render, fireEvent, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import { SelectControl } from './select-control'; // Adjust path as necessary

describe('SelectControl', () => {
  const mockOnChange = jest.fn();
  const defaultLabels = ['Label 1', 'Label 2', 'Label 3'];
  const defaultId = 'test-select';

  beforeEach(() => {
    mockOnChange.mockClear();
    jest.spyOn(console, 'error').mockImplementation(() => {}); // Suppress console.error output during tests
  });

  afterEach(() => {
    (console.error as jest.Mock).mockRestore();
  });

  describe('Resiliency (labels prop validation)', () => {
    it('should render null and log an error if labels is undefined', () => {
      const { container } = render(
        <SelectControl id={defaultId} labels={undefined as any} onChange={mockOnChange} />
      );
      expect(container.firstChild).toBeNull();
      expect(console.error).toHaveBeenCalledWith(
        `SelectControl (id: ${defaultId}): 'labels' prop is missing, not an array, or empty. Rendering null.`
      );
    });

    it('should render null and log an error if labels is null', () => {
      const { container } = render(
        <SelectControl id={defaultId} labels={null as any} onChange={mockOnChange} />
      );
      expect(container.firstChild).toBeNull();
      expect(console.error).toHaveBeenCalledWith(
        `SelectControl (id: ${defaultId}): 'labels' prop is missing, not an array, or empty. Rendering null.`
      );
    });

    it('should render null and log an error if labels is not an array', () => {
      const { container } = render(
        <SelectControl id={defaultId} labels={'not-an-array' as any} onChange={mockOnChange} />
      );
      expect(container.firstChild).toBeNull();
      expect(console.error).toHaveBeenCalledWith(
        `SelectControl (id: ${defaultId}): 'labels' prop is missing, not an array, or empty. Rendering null.`
      );
    });

    it('should render null and log an error if labels is an empty array', () => {
      const { container } = render(
        <SelectControl id={defaultId} labels={[]} onChange={mockOnChange} />
      );
      expect(container.firstChild).toBeNull();
      expect(console.error).toHaveBeenCalledWith(
        `SelectControl (id: ${defaultId}): 'labels' prop is missing, not an array, or empty. Rendering null.`
      );
    });
  });

  describe('Rendering and Basic Functionality', () => {
    it('should render buttons for each label', () => {
      render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} />);
      defaultLabels.forEach(label => {
        expect(screen.getByText(label)).toBeInTheDocument();
      });
    });

    it('should initialize with no selection if value is not provided (single mode)', () => {
      render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} mode="single" />);
      defaultLabels.forEach(label => {
        expect(screen.getByText(label)).not.toHaveClass('active');
      });
    });

    it('should initialize with correct selection based on "value" prop (single mode)', () => {
      render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} mode="single" value={1} />);
      expect(screen.getByText('Label 1')).not.toHaveClass('active');
      expect(screen.getByText('Label 2')).toHaveClass('active');
      expect(screen.getByText('Label 3')).not.toHaveClass('active');
    });
    
    it('should handle disabled state', () => {
      render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} disabled={true} />);
      fireEvent.click(screen.getByText('Label 1'));
      expect(mockOnChange).not.toHaveBeenCalled();
    });
  });

  describe('onChange callback (single mode)', () => {
    it('should call onChange with correct parameters when a button is clicked', () => {
      render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} mode="single" />);
      fireEvent.click(screen.getByText('Label 2'));
      // For single mode, isSelected is true, and the fourth param is the full boolean array
      expect(mockOnChange).toHaveBeenCalledWith(defaultId, 1, true, [false, true, false]);
    });

    it('should update active state correctly on click (single mode)', () => {
      render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} mode="single" value={0} />);
      expect(screen.getByText('Label 1')).toHaveClass('active');
      
      fireEvent.click(screen.getByText('Label 2'));
      
      expect(screen.getByText('Label 1')).not.toHaveClass('active');
      expect(screen.getByText('Label 2')).toHaveClass('active');
      expect(mockOnChange).toHaveBeenCalledWith(defaultId, 1, true, [false, true, false]);
    });
  });
  
  describe('onChange callback (multiple mode)', () => {
    it('should call onChange with correct parameters (multiple mode)', () => {
        render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} mode="multiple" />);
        const button1 = screen.getByText('Label 1');
        fireEvent.click(button1); // Select Label 1
        expect(mockOnChange).toHaveBeenCalledWith(defaultId, 0, true, [true, false, false]);
        
        mockOnChange.mockClear();
        fireEvent.click(button1); // Deselect Label 1
        expect(mockOnChange).toHaveBeenCalledWith(defaultId, 0, false, [false, false, false]);
    });

    it('should toggle active state correctly on click (multiple mode)', () => {
        render(<SelectControl id={defaultId} labels={defaultLabels} onChange={mockOnChange} mode="multiple" values={[0]} />);
        const button1 = screen.getByText('Label 1');
        const button2 = screen.getByText('Label 2');
        expect(button1).toHaveClass('active');
        expect(button2).not.toHaveClass('active');

        fireEvent.click(button2); // Select Label 2
        expect(button1).toHaveClass('active'); // Still active
        expect(button2).toHaveClass('active');
        expect(mockOnChange).toHaveBeenCalledWith(defaultId, 1, true, [true, true, false]);
        
        mockOnChange.mockClear();
        fireEvent.click(button1); // Deselect Label 1
        expect(button1).not.toHaveClass('active');
        expect(button2).toHaveClass('active'); // Still active
        expect(mockOnChange).toHaveBeenCalledWith(defaultId, 0, false, [false, true, false]);
    });
  });
});

# Value Control Component

A reusable touch-enabled value control component for web applications.

## Features

- Touch and mouse interaction support
- Smooth animations and visual feedback
- Customizable value range
- Responsive design
- Event-based value updates

## Usage

1. Include the component files in your project:
```html
<link rel="stylesheet" href="value-control.css">
<script src="value-control.js"></script>
```

2. Create a container element:
```html
<div id="myValueControl"></div>
```

3. Initialize the component:
```javascript
const valueControl = new ValueControl({
    min: 0,
    max: 100,
    initialValue: 50
});

// Mount to container
valueControl.mount('#myValueControl');

// Listen for value changes
valueControl.onValueChange = (value) => {
    console.log('New value:', value);
};
```

## Configuration Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| min | number | 0 | Minimum value |
| max | number | 100 | Maximum value |
| initialValue | number | 50 | Initial value |

## Methods

| Method | Description |
|--------|-------------|
| mount(selector) | Mounts the component to the specified container |
| unmount() | Removes the component from the DOM |
| getValue() | Returns the current value |
| setValue(value) | Sets a new value |

## Events

| Event | Description |
|-------|-------------|
| onValueChange | Called when the value changes |

## Styling

The component uses CSS classes for styling:
- `.value-control` - Main container
- `.value-display` - Value display area
- `.value-slider` - Interactive slider area

Custom styles can be added by overriding these classes.

## Browser Support

- Chrome (latest)
- Firefox (latest)
- Safari (latest)
- Edge (latest)
- Mobile browsers (iOS Safari, Android Chrome)

## License

MIT License 
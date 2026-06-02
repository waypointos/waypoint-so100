import '@testing-library/jest-dom';

// Enables React's act(...) support outside of @testing-library/react's render().
(globalThis as unknown as { IS_REACT_ACT_ENVIRONMENT: boolean }).IS_REACT_ACT_ENVIRONMENT = true;

// jsdom lacks ResizeObserver, which @react-three/fiber's <Canvas> needs to mount.
if (!('ResizeObserver' in globalThis)) {
  (globalThis as unknown as { ResizeObserver: unknown }).ResizeObserver = class {
    observe() {}
    unobserve() {}
    disconnect() {}
  };
}

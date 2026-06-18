import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { BridgeProvider } from './bridge';
import { CalibratePanel } from './CalibratePanel';

function renderWithBridge(publish = vi.fn()) {
  return render(
    <BridgeProvider value={{ roverId: 'r1', subscribe: () => () => {}, publish }}>
      <CalibratePanel />
    </BridgeProvider>,
  );
}

describe('CalibratePanel', () => {
  it('publishes a command on the module command subject when starting', () => {
    const publish = vi.fn();
    renderWithBridge(publish);
    fireEvent.click(screen.getByRole('button', { name: /start calibration/i }));
    expect(publish).toHaveBeenCalledWith(
      'waypoint.r1.module.so100.command',
      expect.any(Uint8Array),
    );
  });

  it('renders six joint rows', () => {
    renderWithBridge();
    expect(screen.getAllByTestId(/joint-row-/).length).toBe(6);
  });
});

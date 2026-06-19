import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { BridgeProvider } from './bridge';
import { ArmPanel } from './ArmPanel';

function renderWithBridge(publish = vi.fn()) {
  return render(
    <BridgeProvider value={{ roverId: 'r1', subscribe: () => () => {}, publish }}>
      <ArmPanel />
    </BridgeProvider>,
  );
}

describe('ArmPanel', () => {
  it('publishes a command on the module command subject when starting calibration', () => {
    const publish = vi.fn();
    renderWithBridge(publish);
    fireEvent.click(screen.getByRole('button', { name: /start calibration/i }));
    expect(publish).toHaveBeenCalledWith(
      'waypoint.r1.module.so100.command',
      expect.any(Uint8Array),
    );
  });

  it('renders six calibration joint rows', () => {
    renderWithBridge();
    expect(screen.getAllByTestId(/joint-row-/).length).toBe(6);
  });

  it('renders the MOTOR DETAIL table with a row per servo', () => {
    const { container } = renderWithBridge();
    const table = container.querySelector('[data-arm-motor-detail]');
    expect(table).not.toBeNull();
    expect(container.querySelectorAll('[data-servo]').length).toBe(6);
  });

  it('shows the feed as awaiting telemetry before any joints arrive', () => {
    const { container } = renderWithBridge();
    expect(container.querySelector('[data-arm-feed]')!.textContent).toContain('awaiting');
  });
});

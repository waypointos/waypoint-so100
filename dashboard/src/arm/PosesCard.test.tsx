import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent, within } from '@testing-library/react';
import { BridgeProvider } from '../bridge';
import { ArmCommand, PoseState, PoseSlot } from '../proto/so100_pb';
import { PosesCard } from './PosesCard';

function state(slots: Array<{ slot: string; name?: string; assigned: boolean }>) {
  return new PoseState({ slots: slots.map((s) => new PoseSlot({ slot: s.slot, name: s.name ?? '', assigned: s.assigned })) });
}

function renderCard(poses: PoseState | null, publish = vi.fn()) {
  render(
    <BridgeProvider value={{ roverId: 'r1', subscribe: () => () => {}, publish }}>
      <PosesCard poses={poses} />
    </BridgeProvider>,
  );
  return publish;
}

// Decode the last published command's action case.
function lastAction(publish: ReturnType<typeof vi.fn>) {
  const [subject, bytes] = publish.mock.calls.at(-1)!;
  expect(subject).toBe('waypoint.r1.module.so100.command');
  return ArmCommand.fromBinary(bytes as Uint8Array).action;
}

describe('PosesCard', () => {
  it('publishes set_torque false / true from the torque buttons', () => {
    const publish = renderCard(null);
    fireEvent.click(screen.getByRole('button', { name: /torque off/i }));
    expect(lastAction(publish)).toEqual({ case: 'setTorque', value: false });
    fireEvent.click(screen.getByRole('button', { name: /torque on/i }));
    expect(lastAction(publish)).toEqual({ case: 'setTorque', value: true });
  });

  it('publishes capture_pose with the slot when assigning', () => {
    const publish = renderCard(state([{ slot: 'share', assigned: false }]));
    const row = screen.getByTestId('pose-slot-share');
    fireEvent.click(within(row).getByRole('button', { name: /assign/i }));
    const action = lastAction(publish);
    expect(action.case).toBe('capturePose');
    expect((action.value as { slot: string }).slot).toBe('share');
  });

  it('publishes delete_pose when clearing an assigned slot', () => {
    const publish = renderCard(state([{ slot: 'options', name: 'stow', assigned: true }]));
    const row = screen.getByTestId('pose-slot-options');
    fireEvent.click(within(row).getByRole('button', { name: /clear/i }));
    expect(lastAction(publish)).toEqual({ case: 'deletePose', value: 'options' });
  });

  it('disables Clear for an unassigned slot', () => {
    renderCard(state([{ slot: 'share', assigned: false }]));
    const row = screen.getByTestId('pose-slot-share');
    expect(within(row).getByRole('button', { name: /clear/i })).toBeDisabled();
  });
});

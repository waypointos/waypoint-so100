import { describe, it, expect, vi } from 'vitest';
import { act } from 'react';
import teleop from './teleop';
import { ArmCommand, JointAngles, PoseSlot, PoseState } from './proto/so100_pb';

function mountWith(deliverRef: { fn: ((b: Uint8Array) => void) | null }) {
  const subjects: string[] = [];
  const off = vi.fn();
  const ctx = {
    roverId: 'r1',
    tokens: {},
    subscribe: (subject: string, onBytes: (b: Uint8Array) => void) => {
      subjects.push(subject);
      deliverRef.fn = onBytes;
      return off;
    },
    publish: vi.fn(),
  };
  const container = document.createElement('div');
  document.body.appendChild(container);
  let teardown: () => void = () => {};
  act(() => {
    teardown = teleop.mount(container, ctx);
  });
  return { subjects, off, container, teardown };
}

describe('teleop render window', () => {
  it('subscribes to module.<rover>.joints and renders a delivered payload without throwing', () => {
    const deliver: { fn: ((b: Uint8Array) => void) | null } = { fn: null };
    const { subjects, off, teardown } = mountWith(deliver);

    expect(subjects).toContain('waypoint.r1.module.so100.joints');

    const payload = new JointAngles({
      joints: [
        { id: 1, angleRad: 0.3 },
        { id: 2, naReason: 'uncalibrated' }, // absent angle => N/A
      ],
    }).toBinary();
    act(() => {
      deliver.fn?.(payload);
    });

    act(() => {
      teardown();
    });
    expect(off).toHaveBeenCalled();
  });

  it('renders the joint rail with N/A reasons instead of fake zeros', () => {
    const deliver: { fn: ((b: Uint8Array) => void) | null } = { fn: null };
    const { container, teardown } = mountWith(deliver);

    // before any telemetry: every joint awaits
    expect(container.textContent).toContain('awaiting telemetry');

    const payload = new JointAngles({
      joints: [
        { id: 1, angleRad: 0.3 },
        { id: 5, naReason: 'uncalibrated' },
      ],
    }).toBinary();
    act(() => {
      deliver.fn?.(payload);
    });

    const rail = container.querySelector('[data-joint-rail]');
    expect(rail).not.toBeNull();
    const j1 = container.querySelector('[data-joint="1"]')!;
    expect(j1.textContent).toContain('+17.2°'); // 0.3 rad
    const j5 = container.querySelector('[data-joint="5"]')!;
    expect(j5.textContent).toContain('N/A');
    expect(j5.textContent).toContain('uncalibrated');

    act(() => teardown());
  });

  it('shows the feed pill as awaiting before telemetry and live after', () => {
    const deliver: { fn: ((b: Uint8Array) => void) | null } = { fn: null };
    const { container, teardown } = mountWith(deliver);

    expect(container.querySelector('[data-arm-feed]')!.textContent).toContain('awaiting');

    act(() => {
      deliver.fn?.(new JointAngles({ joints: [{ id: 1, angleRad: 0 }] }).toBinary());
    });
    expect(container.querySelector('[data-arm-feed]')!.textContent).toContain('live');

    act(() => teardown());
  });

  it('switches the viewport between 3d and 2d', () => {
    const deliver: { fn: ((b: Uint8Array) => void) | null } = { fn: null };
    const { container, teardown } = mountWith(deliver);

    const btn2d = Array.from(container.querySelectorAll('button')).find((b) => b.textContent === '2d')!;
    act(() => { btn2d.click(); });
    expect(container.querySelector('[data-arm-2d]')).not.toBeNull();

    act(() => teardown());
  });
});

// mountRich captures subscribe callbacks per subject so the poses subject can be
// delivered independently of the joints subject.
function mountRich() {
  const cbs = new Map<string, (b: Uint8Array) => void>();
  const publish = vi.fn();
  const ctx = {
    roverId: 'r1',
    subscribe: (subject: string, onBytes: (b: Uint8Array) => void) => {
      cbs.set(subject, onBytes);
      return () => {};
    },
    publish,
  };
  const container = document.createElement('div');
  document.body.appendChild(container);
  let teardown: () => void = () => {};
  act(() => { teardown = teleop.mount(container, ctx); });
  return { cbs, publish, container, teardown };
}

describe('teleop pose recall', () => {
  it('recalls a pose on the command subject when the on-screen button is clicked', () => {
    const { cbs, publish, container, teardown } = mountRich();
    act(() => {
      cbs.get('waypoint.r1.module.so100.poses')?.(
        new PoseState({ slots: [new PoseSlot({ slot: 'share', name: 'reach', assigned: true })] }).toBinary(),
      );
    });

    const btn = container.querySelector<HTMLButtonElement>('[data-testid="pose-recall-share"]')!;
    expect(btn.disabled).toBe(false);
    act(() => { btn.click(); });

    const call = publish.mock.calls.find((c) => c[0] === 'waypoint.r1.module.so100.command')!;
    expect(call).toBeTruthy();
    expect(ArmCommand.fromBinary(call[1] as Uint8Array).action).toEqual({ case: 'recallPose', value: 'share' });

    act(() => teardown());
  });

  it('disables the recall button for an unassigned slot', () => {
    const { cbs, container, teardown } = mountRich();
    act(() => {
      cbs.get('waypoint.r1.module.so100.poses')?.(
        new PoseState({ slots: [new PoseSlot({ slot: 'options', assigned: false })] }).toBinary(),
      );
    });
    const btn = container.querySelector<HTMLButtonElement>('[data-testid="pose-recall-options"]')!;
    expect(btn.disabled).toBe(true);
    act(() => teardown());
  });
});

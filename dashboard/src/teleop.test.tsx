import { describe, it, expect, vi } from 'vitest';
import { act } from 'react';
import teleop from './teleop';
import { JointAngles } from './proto/so100_pb';

describe('teleop render window', () => {
  it('subscribes to module.<rover>.joints and renders a delivered payload without throwing', () => {
    const subjects: string[] = [];
    let deliver: ((b: Uint8Array) => void) | null = null;
    const off = vi.fn();
    const ctx = {
      roverId: 'r1',
      tokens: {},
      subscribe: (subject: string, onBytes: (b: Uint8Array) => void) => {
        subjects.push(subject);
        deliver = onBytes;
        return off;
      },
      publish: vi.fn(),
    };

    const container = document.createElement('div');
    let teardown: () => void = () => {};
    act(() => {
      teardown = teleop.mount(container, ctx);
    });

    expect(subjects).toContain('waypoint.r1.module.so100.joints');

    const payload = new JointAngles({
      joints: [
        { id: 1, angleRad: 0.3 },
        { id: 2, naReason: 'uncalibrated' }, // absent angle => N/A
      ],
    }).toBinary();
    act(() => {
      deliver?.(payload);
    });

    act(() => {
      teardown();
    });
    expect(off).toHaveBeenCalled();
  });
});

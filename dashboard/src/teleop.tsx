// mount(container, ctx): renders the SO-100 arm from live module.so100.joints.
// ctx: { roverId, tokens, subscribe(subject, onBytes), publish(subject, bytes) }
import { createRoot } from 'react-dom/client';
import { JointAngles } from './proto/so100_pb';
import { ArmOverlay } from './arm/ArmOverlay';
import { SO100_JOINTS, type ServoId } from './arm/joints';
import './ui/tokens.css';

type ModuleContext = {
  roverId: string;
  subscribe: (subject: string, onBytes: (b: Uint8Array) => void) => () => void;
  publish?: (subject: string, bytes: Uint8Array) => void;
};

const REST_POSE = Object.fromEntries(SO100_JOINTS.map((j) => [j.id, 0])) as Record<ServoId, number>;

export default {
  mount(container: HTMLElement, ctx: ModuleContext): () => void {
    const root = createRoot(container);
    let pose: Record<ServoId, number> = { ...REST_POSE };
    const render = () => root.render(<ArmOverlay joints={pose} />);
    render();

    const off = ctx.subscribe(`waypoint.${ctx.roverId}.module.so100.joints`, (b) => {
      const ja = JointAngles.fromBinary(b);
      const next: Record<ServoId, number> = { ...pose };
      // Absent angle is N/A; the URDF render holds that joint at rest (0).
      for (const j of ja.joints) next[j.id as ServoId] = j.angleRad ?? 0;
      pose = next;
      render();
    });

    return () => {
      off();
      root.unmount();
    };
  },
};

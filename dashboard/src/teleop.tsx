// mount(container, ctx): renders the SO-100 arm window from live
// module.so100.joints. The panel fills the host window frame; absent angles
// surface as N/A with their reason instead of silently posing at rest.
// ctx: { roverId, tokens, subscribe(subject, onBytes), publish(subject, bytes) }
import { useEffect, useState } from 'react';
import { createRoot } from 'react-dom/client';
import { JointAngles } from './proto/so100_pb';
import { ArmTeleop, type Readings } from './arm/ArmTeleop';
import { SO100_JOINTS, type ServoId } from './arm/joints';
import './ui/tokens.css';

type ModuleContext = {
  roverId: string;
  subscribe: (subject: string, onBytes: (b: Uint8Array) => void) => () => void;
  publish?: (subject: string, bytes: Uint8Array) => void;
};

const AWAITING: Readings = Object.fromEntries(
  SO100_JOINTS.map((j) => [j.id, { angleRad: null, naReason: 'awaiting telemetry' }]),
) as Readings;

function ArmTeleopWindow({ ctx }: { ctx: ModuleContext }) {
  const [readings, setReadings] = useState<Readings>(AWAITING);
  const [lastAtMs, setLastAtMs] = useState<number | null>(null);

  useEffect(() => {
    return ctx.subscribe(`waypoint.${ctx.roverId}.module.so100.joints`, (b) => {
      const ja = JointAngles.fromBinary(b);
      setReadings((prev) => {
        const next = { ...prev };
        for (const j of ja.joints) {
          next[j.id as ServoId] = j.angleRad !== undefined
            ? { angleRad: j.angleRad, naReason: null }
            : { angleRad: null, naReason: j.naReason || 'no data' };
        }
        return next;
      });
      setLastAtMs(Date.now());
    });
  }, [ctx]);

  return <ArmTeleop readings={readings} lastAtMs={lastAtMs} />;
}

export default {
  mount(container: HTMLElement, ctx: ModuleContext): () => void {
    const root = createRoot(container);
    root.render(<ArmTeleopWindow ctx={ctx} />);
    return () => root.unmount();
  },
};

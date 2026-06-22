// POSES card for the Arm tab. Saves the arm's current physical pose into the
// Share / Options slots so the operator can snap back to it from teleop (the
// gamepad Share/Options buttons, or the on-screen buttons there). Workflow:
// torque OFF, pose the arm by hand, Assign to a slot, torque ON to hold.
// Recall drives joints 1-5 only; the gripper is left where it is.
import { useState } from 'react';
import { useBridge } from '../bridge';
import { ArmCommand, PoseCapture, type PoseState } from '../proto/so100_pb';
import { Panel } from '../ui/Panel';
import styles from './PosesCard.module.css';

const SLOTS = [
  { slot: 'share', label: 'Share' },
  { slot: 'options', label: 'Options' },
] as const;

export function PosesCard({ poses }: { poses: PoseState | null }) {
  const { roverId, publish } = useBridge();
  // No torque-enable telemetry exists, so reflect the operator's last intent.
  const [torque, setTorque] = useState<boolean | null>(null);

  const send = (action: ArmCommand['action']) => {
    publish(`waypoint.${roverId}.module.so100.command`, new ArmCommand({ action }).toBinary());
  };
  const setTorqueAll = (on: boolean) => {
    send({ case: 'setTorque', value: on });
    setTorque(on);
  };
  const assign = (slot: string) => send({ case: 'capturePose', value: new PoseCapture({ slot, name: '' }) });
  const clear = (slot: string) => send({ case: 'deletePose', value: slot });

  const bySlot = new Map((poses?.slots ?? []).map((s) => [s.slot, s]));

  const torqueControls = (
    <>
      <button
        className={styles.btn}
        aria-pressed={torque === false}
        onClick={() => setTorqueAll(false)}
      >
        Torque off
      </button>
      <button
        className={styles.btn}
        aria-pressed={torque === true}
        onClick={() => setTorqueAll(true)}
      >
        Torque on
      </button>
    </>
  );

  return (
    <Panel title="POSES" note={torque === null ? '' : torque ? 'TORQUE ON' : 'TORQUE OFF'} action={torqueControls}>
      <p className={styles.hint}>
        Turn <b>torque off</b>, move the arm by hand to the pose you want, then <b>Assign</b> it to
        Share or Options. Turn <b>torque on</b> to lock it. In teleop, the Share/Options buttons (or
        the on-screen buttons) snap the arm back. The gripper is left where it is.
      </p>

      <ul className={styles.slots}>
        {SLOTS.map(({ slot, label }) => {
          const s = bySlot.get(slot);
          const assigned = s?.assigned ?? false;
          return (
            <li key={slot} className={styles.slot} data-testid={`pose-slot-${slot}`}>
              <span className={styles.slotName}>{label}</span>
              <span className={assigned ? styles.set : styles.unset}>
                {assigned ? (s?.name?.trim() || 'assigned') : 'unassigned'}
              </span>
              <span className={styles.actions}>
                <button className={styles.btn} onClick={() => assign(slot)}>
                  {assigned ? 'Reassign' : 'Assign'}
                </button>
                <button className={styles.clear} onClick={() => clear(slot)} disabled={!assigned}>
                  Clear
                </button>
              </span>
            </li>
          );
        })}
      </ul>
    </Panel>
  );
}

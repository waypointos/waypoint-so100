// CALIBRATION card for the Arm tab. Holds the range-recording workflow that
// used to be the entire tab: start/finish/abort controls in the card header,
// the per-joint captured-range table below. Torque is off during recording, so
// the hint warns the operator to support the arm.
import { useBridge } from '../bridge';
import { ArmCommand, type CalibrationState } from '../proto/so100_pb';
import { SO100_JOINTS } from './joints';
import { Panel } from '../ui/Panel';
import styles from './CalibrationCard.module.css';

export function CalibrationCard({ cal }: { cal: CalibrationState | null }) {
  const { roverId, publish } = useBridge();

  const send = (action: 'runCalibration' | 'finishCalibration' | 'abort') => {
    const cmd = new ArmCommand({ action: { case: action, value: true } });
    publish(`waypoint.${roverId}.module.so100.command`, cmd.toBinary());
  };

  const byId = new Map((cal?.joints ?? []).map((j) => [j.id, j]));
  const phase = cal?.phase ?? 'idle';
  const recording = phase === 'recording';

  const controls = recording ? (
    <>
      <button className={styles.run} onClick={() => send('finishCalibration')}>Done — save</button>
      <button className={styles.abort} onClick={() => send('abort')}>Abort</button>
    </>
  ) : (
    <button className={styles.run} onClick={() => send('runCalibration')}>Start calibration</button>
  );

  return (
    <Panel title="CALIBRATION" note={phase.toUpperCase()} action={controls}>
      {recording && (
        <p className={styles.hint}>
          Torque is off — support the arm so it doesn’t drop, then move every joint slowly through
          its full range until it meets each hard stop (open and close the gripper too). Click
          “Done — save” when finished.
        </p>
      )}

      <table className={styles.joints}>
        <tbody>
          {SO100_JOINTS.map((spec) => {
            const j = byId.get(spec.id);
            const active = cal?.activeJoint === spec.id;
            return (
              <tr key={spec.id} data-testid={`joint-row-${spec.id}`} className={active ? styles.active : undefined}>
                <td className={styles.jointName}>{spec.label}</td>
                <td className={styles.range}>
                  {j?.rawMin !== undefined && j?.rawMax !== undefined
                    ? `${j.rawMin}..${j.rawMax}`
                    : <span className={styles.na}>N/A — not measured</span>}
                </td>
                <td className={styles.status}>
                  {!j ? <span className={styles.na}>N/A</span>
                    : j.ok ? <span className={styles.ok}>OK</span>
                    : <span className={styles.flag}>{j.flagReason || 'not calibrated'}</span>}
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </Panel>
  );
}

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

  const send = (action: 'runCalibration' | 'setHome' | 'finishCalibration' | 'abort') => {
    const cmd = new ArmCommand({ action: { case: action, value: true } });
    publish(`waypoint.${roverId}.module.so100.command`, cmd.toBinary());
  };

  const byId = new Map((cal?.joints ?? []).map((j) => [j.id, j]));
  const phase = cal?.phase ?? 'idle';
  const recording = phase === 'recording';
  const homeSet = cal?.homeSet ?? false;

  const controls = recording ? (
    <>
      <button className={styles.run} onClick={() => send('setHome')}>
        {homeSet ? 'Re-set home' : 'Set home'}
      </button>
      <button className={styles.run} onClick={() => send('finishCalibration')} disabled={!homeSet}>
        Done — save
      </button>
      <button className={styles.abort} onClick={() => send('abort')}>Abort</button>
    </>
  ) : (
    <button className={styles.run} onClick={() => send('runCalibration')}>Start calibration</button>
  );

  const note = recording ? `${phase.toUpperCase()} · ${homeSet ? 'HOME SET' : 'NO HOME'}` : phase.toUpperCase();

  return (
    <Panel title="CALIBRATION" note={note} action={controls}>
      {recording && (
        <p className={styles.hint}>
          Torque is off — support the arm so it doesn’t drop. <b>1.</b> Move the arm to its neutral
          (straight / URDF-zero) pose and click <b>Set home</b>. <b>2.</b> Move every joint slowly
          through its full range to both limits (open and close the gripper too). <b>3.</b> Click
          <b> Done — save</b>.
        </p>
      )}

      <table className={styles.joints}>
        <thead>
          <tr>
            <th>Joint</th>
            <th>Range</th>
            <th>Status</th>
          </tr>
        </thead>
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

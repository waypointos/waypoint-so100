import { useBridge } from './bridge';
import { useCalibration } from './useCalibration';
import { ArmOverlay } from './arm/ArmOverlay';
import { ArmCommand } from './proto/so100_pb';
import { SO100_JOINTS } from './arm/joints';
import styles from './CalibratePanel.module.css';

export function CalibratePanel() {
  const { roverId, publish } = useBridge();
  const cal = useCalibration();

  const send = (action: 'runCalibration' | 'finishCalibration' | 'abort') => {
    const cmd = new ArmCommand({ action: { case: action, value: true } });
    publish(`waypoint.${roverId}.module.so100.command`, cmd.toBinary());
  };

  const byId = new Map((cal?.joints ?? []).map((j) => [j.id, j]));
  const phase = cal?.phase ?? 'idle';
  const recording = phase === 'recording';

  return (
    <div className={styles.wrap} data-testid="panel-m-so100">
      <header className={styles.header}>
        <span className={styles.title}>ARM CALIBRATION</span>
        <span className={styles.phase}>{phase.toUpperCase()}</span>
        {recording ? (
          <>
            <button className={styles.run} onClick={() => send('finishCalibration')}>
              Done — save
            </button>
            <button className={styles.abort} onClick={() => send('abort')}>
              Abort
            </button>
          </>
        ) : (
          <button className={styles.run} onClick={() => send('runCalibration')}>
            Start calibration
          </button>
        )}
      </header>

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

      <ArmOverlay />
    </div>
  );
}

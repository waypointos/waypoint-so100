import { useBridge } from './bridge';
import { useCalibration } from './useCalibration';
import { ArmOverlay } from './arm/ArmOverlay';
import { ArmCommand } from './proto/so100_pb';
import { SO100_JOINTS } from './arm/joints';
import styles from './CalibratePanel.module.css';

export function CalibratePanel() {
  const { roverId, publish } = useBridge();
  const cal = useCalibration();

  const runCalibration = () => {
    const cmd = new ArmCommand({ action: { case: 'runCalibration', value: true } });
    publish(`waypoint.${roverId}.module.so100.command`, cmd.toBinary());
  };

  const byId = new Map((cal?.joints ?? []).map((j) => [j.id, j]));
  const phase = cal?.phase ?? 'idle';

  return (
    <div className={styles.wrap} data-testid="panel-m-so100">
      <header className={styles.header}>
        <span className={styles.title}>ARM CALIBRATION</span>
        <span className={styles.phase}>{phase.toUpperCase()}</span>
        <button className={styles.run} onClick={runCalibration} disabled={phase === 'running'}>
          Run calibration
        </button>
      </header>

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

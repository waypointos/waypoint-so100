// Dense raw-telemetry table for the Arm tab — the arm counterpart to the host
// OVERVIEW's MOTOR DETAIL. One row per SO-100 servo straight off the
// module.so100.stats stream: position/speed/load/current ticks, bus volts,
// temperature, and a live status cell. A servo that did not respond renders
// every cell as "—" (ok=false) rather than a fake zero.
import { SO100_JOINTS, type ServoId } from './joints';
import type { Stats } from '../useArmTelemetry';
import styles from './MotorDetail.module.css';

const SHORT: Record<ServoId, string> = {
  1: 'pan', 2: 'lift', 3: 'elbow', 4: 'w.flex', 5: 'w.roll', 6: 'grip',
};

const num = (v: number | undefined): string => (v == null ? '—' : String(v));
const volts = (deci: number | undefined): string => (deci == null ? '—' : (deci / 10).toFixed(1));

function firstBusVolts(stats: Stats): string {
  for (const spec of SO100_JOINTS) {
    const v = stats[spec.id]?.voltageDeci;
    if (v != null) return `${(v / 10).toFixed(1)} V`;
  }
  return '— V';
}

function StatusCell({ s }: { s: Stats[ServoId] }) {
  if (!s || !s.ok) return <span className={styles.stOff}>no rd</span>;
  if (s.overcurrentTripped) return <span className={styles.stFault}>OC!</span>;
  if (s.moving) return <span className={styles.stMove}>mov</span>;
  return <span className={styles.stOk}>ok</span>;
}

export function MotorDetail({ stats }: { stats: Stats }) {
  return (
    <table className={styles.detail} data-arm-motor-detail>
      <thead>
        <tr>
          <th>ID</th>
          <th>Pos</th>
          <th>Spd</th>
          <th>Load</th>
          <th>I</th>
          <th>V</th>
          <th>°C</th>
          <th>st</th>
        </tr>
      </thead>
      <tbody>
        {SO100_JOINTS.map((spec) => {
          const s = stats[spec.id];
          return (
            <tr key={spec.id} data-servo={spec.id}>
              <td className={styles.detailId}>
                {String(spec.id).padStart(2, '0')} · {SHORT[spec.id]}
              </td>
              <td>{num(s?.positionRaw)}</td>
              <td>{num(s?.speedRaw)}</td>
              <td>{num(s?.loadRaw)}</td>
              <td>{num(s?.currentRaw)}</td>
              <td>{volts(s?.voltageDeci)}</td>
              <td>{num(s?.temperatureC)}</td>
              <td className={styles.st}><StatusCell s={s} /></td>
            </tr>
          );
        })}
      </tbody>
      <tfoot>
        <tr>
          <td colSpan={8} className={styles.detailFoot}>
            Bus: {firstBusVolts(stats)} · pos/spd/load/I raw ticks · V volts · °C
          </td>
        </tr>
      </tfoot>
    </table>
  );
}

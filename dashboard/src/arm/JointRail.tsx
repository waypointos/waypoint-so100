// dashboard/src/arm/JointRail.tsx
//
// Live joint readout rail: angle, position-in-limits bar, and a first-class
// N/A row (reason shown, no fake zero) per the platform telemetry rules.
// Values pad to a fixed width so the rail never shifts as digits change.
import { SO100_JOINTS, radToDeg, type ServoId } from './joints';
import type { Readings } from './ArmTeleop';
import styles from './JointRail.module.css';

const NEAR_LIMIT_DEG = 8;
const NBSP = ' ';

function fixedDeg(rad: number): string {
  const v = radToDeg(rad);
  return `${(v >= 0 ? '+' : '') + v.toFixed(1)}°`.padStart(7, NBSP);
}

export function JointRail({ readings }: { readings: Readings }) {
  return (
    <div className={styles.rail} data-joint-rail>
      <div className={styles.head}><span>joints</span><span>deg</span></div>
      {SO100_JOINTS.map((spec) => {
        const r = readings[spec.id as ServoId];
        const rad = r?.angleRad ?? null;
        if (rad === null) {
          return (
            <div key={spec.id} className={styles.row} data-joint={spec.id}>
              <span className={styles.id}>{String(spec.id).padStart(2, '0')}</span>
              <span className={styles.name}>{spec.label}</span>
              <span className={styles.na}>N/A</span>
              <span className={styles.reason}>{r?.naReason ?? 'no data'}</span>
            </div>
          );
        }
        const lowDeg = radToDeg(spec.lowerRad);
        const highDeg = radToDeg(spec.upperRad);
        const deg = radToDeg(rad);
        const frac = Math.max(0, Math.min(1, (deg - lowDeg) / (highDeg - lowDeg)));
        const zeroFrac = Math.max(0, Math.min(1, (0 - lowDeg) / (highDeg - lowDeg)));
        const hot = deg - lowDeg < NEAR_LIMIT_DEG || highDeg - deg < NEAR_LIMIT_DEG;
        return (
          <div key={spec.id} className={styles.row} data-joint={spec.id}>
            <span className={styles.id}>{String(spec.id).padStart(2, '0')}</span>
            <span className={styles.name}>{spec.label}</span>
            <span className={hot ? `${styles.val} ${styles.hot}` : styles.val}>{fixedDeg(rad)}</span>
            <span className={styles.bar}>
              <span className={styles.zero} style={{ left: `${zeroFrac * 100}%` }} />
              <span className={hot ? `${styles.mark} ${styles.markHot}` : styles.mark}
                style={{ left: `${frac * 100}%` }} />
            </span>
          </div>
        );
      })}
    </div>
  );
}

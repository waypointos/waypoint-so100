// MOTORS card grid — the at-a-glance per-servo view, copied from the host
// OVERVIEW's MotorCard grid. Pairs with MOTOR DETAIL (the raw-data companion):
// this shows current, temperature, and speed per servo as compact cards, with
// a thermal-amber accent when a servo runs hot. A servo that did not respond is
// dimmed with "—" values.
import { SO100_JOINTS, SHORT_LABEL, type ServoId } from './joints';
import type { Stats } from '../useArmTelemetry';
import styles from './MotorCards.module.css';

const HOT_AT_C = 55;

function MotorCard({ id, s }: { id: ServoId; s: Stats[ServoId] }) {
  const offline = !s || !s.ok;
  const hot = !!s && s.temperatureC != null && s.temperatureC >= HOT_AT_C;
  const cls = [styles.card, hot && styles.hot, offline && styles.off].filter(Boolean).join(' ');
  return (
    <div className={cls} data-motor-card={id}>
      <div className={styles.id}>{String(id).padStart(2, '0')}<br />{SHORT_LABEL[id]}</div>
      <div className={styles.vals}>
        <div className={styles.a}>
          {s?.currentRaw == null ? '—' : s.currentRaw}
          <span className={styles.u}>I·raw</span>
        </div>
        <div className={styles.t}>{s?.temperatureC == null ? '—' : `${s.temperatureC} °C`}</div>
        <div className={styles.vel}>
          <span className={styles.velLbl}>spd</span>
          <span className={styles.velVal}>{s?.speedRaw == null ? '—' : s.speedRaw}</span>
          <span className={styles.velUnit}>raw</span>
        </div>
      </div>
    </div>
  );
}

export function MotorCards({ stats }: { stats: Stats }) {
  return (
    <div className={styles.grid}>
      {SO100_JOINTS.map((spec) => (
        <MotorCard key={spec.id} id={spec.id} s={stats[spec.id]} />
      ))}
    </div>
  );
}

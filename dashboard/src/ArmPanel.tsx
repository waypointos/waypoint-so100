// dashboard/src/ArmPanel.tsx
//
// The SO-100 Arm tab ([ui.static] surface). Composed of cards in the host
// dashboard's language (Panel + 1fr/320px split, like OVERVIEW):
//   center → ARM 3D hero, MOTOR DETAIL (live servo telemetry), CALIBRATION
//   rail   → JOINTS live readout, STATUS (feed / cal / grip)
// All telemetry is live off the module's private subjects; absent values show
// N/A with their reason rather than fake zeros.
import { useMemo } from 'react';
import { ArmHeroCard } from './arm/ArmHeroCard';
import { JointRail } from './arm/JointRail';
import { MotorCards } from './arm/MotorCards';
import { MotorDetail } from './arm/MotorDetail';
import { CalibrationCard } from './arm/CalibrationCard';
import { PosesCard } from './arm/PosesCard';
import { Panel } from './ui/Panel';
import { useJoints, useStats, useAgeMs } from './useArmTelemetry';
import { useCalibration } from './useCalibration';
import { usePoses } from './usePoses';
import { SO100_JOINTS, type ServoId } from './arm/joints';
import styles from './ArmPanel.module.css';

const STALE_AFTER_MS = 2_000;

export function ArmPanel() {
  const { readings, lastAtMs } = useJoints();
  const { stats } = useStats();
  const cal = useCalibration();
  const poses = usePoses();
  const ageMs = useAgeMs(lastAtMs);

  // 3D render holds N/A joints at rest; the JOINTS card carries the reason.
  const pose = useMemo(() => {
    const out = {} as Record<ServoId, number>;
    for (const j of SO100_JOINTS) out[j.id] = readings[j.id]?.angleRad ?? 0;
    return out;
  }, [readings]);

  const uncal = SO100_JOINTS.filter((j) => readings[j.id]?.naReason === 'uncalibrated').length;
  const gripper = SO100_JOINTS.find((j) => j.id === 6);
  const gripRad = readings[6]?.angleRad;
  const gripPct = gripper && gripRad != null
    ? Math.round(((gripRad - gripper.lowerRad) / (gripper.upperRad - gripper.lowerRad)) * 100)
    : null;

  const liveMotors = SO100_JOINTS.filter((j) => stats[j.id]?.ok).length;
  const stale = ageMs !== null && ageMs > STALE_AFTER_MS;
  const feed = ageMs === null
    ? { cls: styles.feedOff, text: 'awaiting telemetry' }
    : stale
      ? { cls: styles.feedStale, text: `stale · ${(ageMs / 1000).toFixed(0)}s` }
      : { cls: styles.feedLive, text: 'live' };

  return (
    <div className={styles.layout} data-arm-panel data-testid="panel-m-so100">
      <div className={styles.center}>
        <ArmHeroCard pose={pose} readings={readings} />
        <Panel title="MOTORS" note={`${liveMotors}/${SO100_JOINTS.length} live · STS3215`}>
          <MotorCards stats={stats} />
        </Panel>
        <Panel title="MOTOR DETAIL" note="raw telemetry">
          <MotorDetail stats={stats} />
        </Panel>
        <CalibrationCard cal={cal} />
        <PosesCard poses={poses} />
      </div>

      <aside className={styles.rail}>
        <Panel title="JOINTS" note="deg">
          <JointRail readings={readings} variant="card" />
        </Panel>
        <Panel title="STATUS">
          <div className={styles.status}>
            <div className={styles.statusRow}>
              <span className={styles.statusLbl}>feed</span>
              <span className={`${styles.feed} ${feed.cls}`} data-arm-feed>
                <span className={styles.dot} />{feed.text}
              </span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusLbl}>cal</span>
              <span className={uncal > 0 ? styles.warn : styles.statusVal}>
                {SO100_JOINTS.length - uncal}/{SO100_JOINTS.length}
              </span>
            </div>
            <div className={styles.statusRow}>
              <span className={styles.statusLbl}>grip</span>
              <span className={styles.statusVal}>{gripPct === null ? 'N/A' : `${gripPct}%`}</span>
            </div>
          </div>
        </Panel>
      </aside>
    </div>
  );
}

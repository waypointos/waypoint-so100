// dashboard/src/arm/ArmTeleop.tsx
//
// Teleop window panel for the SO-100 arm. Fills the host window frame
// (TeleopWindowHost owns the title bar, drag, resize, close): viewport with
// 3D URDF / 2D schematic modes on the left, live joint rail on the right,
// status footer beneath. N/A joints render at rest in the viewport while the
// rail says why; the footer feed pill turns stale when telemetry stops.
import { useEffect, useMemo, useState } from 'react';
import { Arm3DScene } from './Arm3DScene';
import { Arm2DScene, type FlatView } from './Arm2DScene';
import { JointRail } from './JointRail';
import { CAMERA_PRESETS, DEFAULT_PRESET, type PresetKey } from './cameraPresets';
import { SO100_JOINTS, radToDeg, type ServoId } from './joints';
import styles from './ArmTeleop.module.css';

export type JointReading = { angleRad: number | null; naReason: string | null };
export type Readings = Record<ServoId, JointReading>;

type ViewMode = '3d' | '2d';

const STALE_AFTER_MS = 2_000;

function useAgeMs(lastAtMs: number | null): number | null {
  const [, bump] = useState(0);
  useEffect(() => {
    const id = setInterval(() => bump((n) => n + 1), 500);
    return () => clearInterval(id);
  }, []);
  return lastAtMs === null ? null : Date.now() - lastAtMs;
}

export function ArmTeleop({ readings, lastAtMs }: { readings: Readings; lastAtMs: number | null }) {
  const [mode, setMode] = useState<ViewMode>('3d');
  const [preset, setPreset] = useState<PresetKey>(DEFAULT_PRESET);
  const [flat, setFlat] = useState<FlatView>('side');
  const [opacity, setOpacity] = useState(1);
  const ageMs = useAgeMs(lastAtMs);

  // URDF render holds N/A joints at rest; the rail carries the reason.
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

  const stale = ageMs !== null && ageMs > STALE_AFTER_MS;
  const feed = ageMs === null
    ? { cls: styles.feedOff, text: 'awaiting telemetry' }
    : stale
      ? { cls: styles.feedStale, text: `stale · ${(ageMs / 1000).toFixed(0)}s` }
      : { cls: styles.feedLive, text: 'live' };

  return (
    <div className={styles.panel} data-arm-teleop>
      <div className={styles.main}>
        <div className={styles.viewport}>
          {mode === '3d'
            ? <Arm3DScene joints={pose} preset={preset} opacity={opacity} />
            : <Arm2DScene readings={readings} view={flat} />}
          <span className={styles.tag}>
            {mode === '3d' ? '3d · drag to orbit' : flat === 'plan' ? '2d · plan view' : '2d · side elevation'}
          </span>
          <div className={styles.modeRow}>
            {(['3d', '2d'] as const).map((m) => (
              <button key={m} type="button" className={styles.vpBtn}
                aria-pressed={mode === m} onClick={() => setMode(m)}>{m}</button>
            ))}
          </div>
          <div className={styles.presetRow}>
            {mode === '3d'
              ? CAMERA_PRESETS.map((p) => (
                  <button key={p.key} type="button" className={styles.vpBtn}
                    aria-pressed={preset === p.key} onClick={() => setPreset(p.key)}>{p.label}</button>
                ))
              : (['side', 'plan'] as const).map((v) => (
                  <button key={v} type="button" className={styles.vpBtn}
                    aria-pressed={flat === v} onClick={() => setFlat(v)}>{v}</button>
                ))}
          </div>
        </div>
        <JointRail readings={readings} />
      </div>
      <footer className={styles.foot}>
        <span className={`${styles.feed} ${feed.cls}`} data-arm-feed>
          <span className={styles.feedDot} />{feed.text}
        </span>
        <span className={uncal > 0 ? styles.calWarn : undefined}>cal {SO100_JOINTS.length - uncal}/{SO100_JOINTS.length}</span>
        <span>grip {gripPct === null ? 'N/A' : `${gripPct}%`}</span>
        <span className={styles.footSpacer} />
        <label className={styles.alpha}>
          opacity
          <input type="range" min={30} max={100} value={Math.round(opacity * 100)}
            onChange={(e) => setOpacity(Number(e.target.value) / 100)} />
          <span className={styles.alphaVal}>{Math.round(opacity * 100)}</span>
        </label>
      </footer>
    </div>
  );
}

/** Degrees label used by tests and the rail; exported for reuse. */
export function degLabel(angleRad: number): string {
  const v = radToDeg(angleRad);
  return `${v >= 0 ? '+' : ''}${v.toFixed(1)}°`;
}

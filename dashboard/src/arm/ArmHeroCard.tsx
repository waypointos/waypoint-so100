// ARM hero card — the live arm render inlined as the tab's lead card. Mirrors
// the teleop viewport (the known-good layout): 3D URDF or 2D schematic with a
// mode toggle, camera/flat presets, and an opacity control, all as overlays in
// the stage. Replaces the old draggable ArmOverlay window (that floating
// pattern belongs to the teleop console, not a static tab).
import { useState } from 'react';
import { Arm3DScene } from './Arm3DScene';
import { Arm2DScene, type FlatView } from './Arm2DScene';
import { CAMERA_PRESETS, DEFAULT_PRESET, type PresetKey } from './cameraPresets';
import { type ServoId } from './joints';
import type { Readings } from './ArmTeleop';
import { Panel } from '../ui/Panel';
import styles from './ArmHeroCard.module.css';

type ViewMode = '3d' | '2d';

export function ArmHeroCard({ pose, readings }: {
  pose: Record<ServoId, number>;
  readings: Readings;
}) {
  const [mode, setMode] = useState<ViewMode>('3d');
  const [preset, setPreset] = useState<PresetKey>(DEFAULT_PRESET);
  const [flat, setFlat] = useState<FlatView>('side');
  const [opacity, setOpacity] = useState(1);

  const note = mode === '3d'
    ? '3d · drag to orbit'
    : flat === 'plan' ? '2d · plan view' : '2d · side elevation';

  return (
    <Panel title="ARM" note={note}>
      <div className={styles.stage}>
        {mode === '3d'
          ? <Arm3DScene joints={pose} preset={preset} opacity={opacity} />
          : <Arm2DScene readings={readings} view={flat} />}

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

        {mode === '3d' && (
          <label className={styles.opacity}>
            opacity
            <input type="range" min={30} max={100} value={Math.round(opacity * 100)}
              onChange={(e) => setOpacity(Number(e.target.value) / 100)} />
            <span className={styles.opacityVal}>{Math.round(opacity * 100)}</span>
          </label>
        )}
      </div>
    </Panel>
  );
}

// ARM hero card — the live arm render as the tab's lead card. Two panes side by
// side: the 3D URDF scene (left, camera presets + opacity) and the 2D schematic
// (right, side/plan toggle). Replaces the old draggable ArmOverlay window (that
// floating pattern belongs to the teleop console, not a static tab).
import { useState } from 'react';
import { Arm3DScene } from './Arm3DScene';
import { Arm2DScene, type FlatView } from './Arm2DScene';
import { CAMERA_PRESETS, DEFAULT_PRESET, type PresetKey } from './cameraPresets';
import { type ServoId } from './joints';
import type { Readings } from './ArmTeleop';
import { Panel } from '../ui/Panel';
import styles from './ArmHeroCard.module.css';

export function ArmHeroCard({ pose, readings }: {
  pose: Record<ServoId, number>;
  readings: Readings;
}) {
  const [preset, setPreset] = useState<PresetKey>(DEFAULT_PRESET);
  const [flat, setFlat] = useState<FlatView>('side');
  const [opacity, setOpacity] = useState(1);

  return (
    <Panel title="ARM" note="live render">
      <div className={styles.stages}>
        <div className={styles.stage}>
          <span className={styles.tag}>3d · drag to orbit</span>
          <Arm3DScene joints={pose} preset={preset} opacity={opacity} />
          <div className={styles.presetRow}>
            {CAMERA_PRESETS.map((p) => (
              <button key={p.key} type="button" className={styles.vpBtn}
                aria-pressed={preset === p.key} onClick={() => setPreset(p.key)}>{p.label}</button>
            ))}
          </div>
          <label className={styles.opacity}>
            opacity
            <input type="range" min={30} max={100} value={Math.round(opacity * 100)}
              onChange={(e) => setOpacity(Number(e.target.value) / 100)} />
            <span className={styles.opacityVal}>{Math.round(opacity * 100)}</span>
          </label>
        </div>

        <div className={styles.stage}>
          <span className={styles.tag}>2d · {flat === 'plan' ? 'plan view' : 'side elevation'}</span>
          <Arm2DScene readings={readings} view={flat} />
          <div className={styles.presetRow}>
            {(['side', 'plan'] as const).map((v) => (
              <button key={v} type="button" className={styles.vpBtn}
                aria-pressed={flat === v} onClick={() => setFlat(v)}>{v}</button>
            ))}
          </div>
        </div>
      </div>
    </Panel>
  );
}

// ARM hero card — the live 3D URDF render inlined as the tab's lead card.
// This replaces the old draggable ArmOverlay window (that floating pattern
// belongs to the teleop console, not a static tab). Camera presets sit in the
// card header; opacity is a small overlay control in the corner of the stage.
import { useState } from 'react';
import { Arm3DScene } from './Arm3DScene';
import { CAMERA_PRESETS, DEFAULT_PRESET, type PresetKey } from './cameraPresets';
import { type ServoId } from './joints';
import { Panel } from '../ui/Panel';
import styles from './ArmHeroCard.module.css';

export function ArmHeroCard({ pose }: { pose: Record<ServoId, number> }) {
  const [preset, setPreset] = useState<PresetKey>(DEFAULT_PRESET);
  const [opacity, setOpacity] = useState(1);

  const presets = (
    <span className={styles.presetRow}>
      {CAMERA_PRESETS.map((p) => (
        <button key={p.key} type="button" className={styles.presetBtn}
          aria-pressed={preset === p.key} onClick={() => setPreset(p.key)}>{p.label}</button>
      ))}
    </span>
  );

  return (
    <Panel title="ARM" note="3d · drag to orbit" action={presets}>
      <div className={styles.stage}>
        <Arm3DScene joints={pose} preset={preset} opacity={opacity} />
        <label className={styles.opacity}>
          opacity
          <input type="range" min={30} max={100} value={Math.round(opacity * 100)}
            onChange={(e) => setOpacity(Number(e.target.value) / 100)} />
          <span className={styles.opacityVal}>{Math.round(opacity * 100)}</span>
        </label>
      </div>
    </Panel>
  );
}

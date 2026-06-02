// dashboard/src/views/ArmDemo/ArmOverlay.tsx
//
// Transparent, draggable/resizable arm overlay. The 3D floats with no chrome
// until hover, which reveals the drag bar, camera presets, and resize handle.
import { useState } from 'react';
import { BracketCorners } from './BracketCorners';
import { useDragResize } from './useDragResize';
import { Arm3DScene } from './Arm3DScene';
import { OverlayChrome } from './OverlayChrome';
import { SO100_JOINTS, type ServoId } from './joints';
import { DEFAULT_PRESET, type PresetKey } from './cameraPresets';
import styles from './ArmOverlay.module.css';

const REST_POSE = Object.fromEntries(SO100_JOINTS.map((j) => [j.id, 0])) as Record<ServoId, number>;

export function ArmOverlay({ onClose, joints = REST_POSE }: {
  onClose?: () => void;
  joints?: Record<ServoId, number>;
}) {
  const { pos, size, startMove, startResize } = useDragResize({
    initialPosition: { x: 96, y: 96 },
    initialSize: { width: 420, height: 460 },
    minSize: { width: 240, height: 260 },
  });
  const [preset, setPreset] = useState<PresetKey>(DEFAULT_PRESET);
  const [opacity, setOpacity] = useState(0.6);

  return (
    <section
      className={styles.overlay}
      style={{ left: pos.x, top: pos.y, width: size.width, height: size.height }}
    >
      <Arm3DScene joints={joints} preset={preset} opacity={opacity} />
      <div className={styles.chrome}>
        <BracketCorners full color="var(--color-fg-4)" />
        <OverlayChrome
          activePreset={preset}
          onPreset={setPreset}
          opacity={opacity}
          onOpacity={setOpacity}
          onClose={onClose}
          onMoveStart={startMove}
          onResizeStart={startResize}
        />
      </div>
    </section>
  );
}

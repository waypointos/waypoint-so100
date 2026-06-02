// dashboard/src/views/ArmDemo/OverlayChrome.tsx
//
// Hover-revealed controls for the transparent arm overlay: a drag bar, camera
// preset buttons (top-left), close, and a resize handle. Presentational only —
// no canvas — so it stays unit-testable.
import React from 'react';
import { X } from 'lucide-react';
import { CAMERA_PRESETS, type PresetKey } from './cameraPresets';
import styles from './OverlayChrome.module.css';

type Props = {
  activePreset: PresetKey;
  onPreset: (key: PresetKey) => void;
  opacity: number;
  onOpacity: (value: number) => void;
  onClose?: () => void;
  onMoveStart: (e: React.PointerEvent) => void;
  onResizeStart: (e: React.PointerEvent) => void;
};

export function OverlayChrome({
  activePreset, onPreset, opacity, onOpacity, onClose, onMoveStart, onResizeStart,
}: Props) {
  const pct = Math.round(opacity * 100);
  return (
    <>
      <header className={styles.bar} onPointerDown={onMoveStart}>
        <span className={styles.title}>
          <span className={styles.bracket}>[</span>SO-100 ARM<span className={styles.bracket}>]</span>
        </span>
        {onClose && (
          <button type="button" className={styles.close} aria-label="close" onClick={onClose}>
            <X size={12} />
          </button>
        )}
      </header>

      <div className={styles.presets} role="group" aria-label="camera presets">
        {CAMERA_PRESETS.map((p) => (
          <button
            key={p.key}
            type="button"
            className={styles.preset}
            aria-pressed={p.key === activePreset}
            onClick={() => onPreset(p.key)}
          >
            {p.label}
          </button>
        ))}
      </div>

      <div className={styles.opacity}>
        <span className={styles.olabel}>Opacity</span>
        <input
          type="range"
          className={styles.oslider}
          min={15}
          max={100}
          step={1}
          value={pct}
          aria-label="mesh opacity"
          onChange={(e) => onOpacity(Number(e.target.value) / 100)}
        />
        <span className={styles.oval}>{pct}%</span>
      </div>

      <span
        className={styles.resize}
        aria-label="resize"
        role="separator"
        onPointerDown={onResizeStart}
      />
    </>
  );
}

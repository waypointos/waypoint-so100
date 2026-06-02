// dashboard/src/views/ArmDemo/cameraPresets.ts
//
// Orbit-camera presets for the arm overlay. Positions are in metres, framed on
// the arm's mid-height; the rig in Arm3DScene eases the camera toward these.
export type PresetKey = 'front' | 'side' | 'top' | 'iso';

export interface CameraPreset {
  key: PresetKey;
  label: string;
  pos: [number, number, number];
  target: [number, number, number];
  /** Vertical FOV (deg). A narrow FOV from far away flattens perspective into
   *  a near-orthographic elevation; omit for the default lens. */
  fov?: number;
}

const TARGET: [number, number, number] = [0, 0.15, 0];

export const CAMERA_PRESETS: readonly CameraPreset[] = [
  { key: 'front', label: 'Front', pos: [0.6, 0.18, 0], target: TARGET },
  { key: 'side',  label: 'Side',  pos: [0, 0.18, 7], target: TARGET, fov: 5 },
  { key: 'top',   label: 'Top',   pos: [0, 0.75, 0.0001], target: TARGET },
  { key: 'iso',   label: '3/4',   pos: [0.42, 0.38, 0.42], target: TARGET },
];

export const DEFAULT_PRESET: PresetKey = 'iso';

export function presetByKey(key: PresetKey): CameraPreset {
  const preset = CAMERA_PRESETS.find((p) => p.key === key);
  if (!preset) throw new Error(`unknown camera preset: ${key}`);
  return preset;
}

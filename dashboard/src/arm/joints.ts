// dashboard/src/views/ArmDemo/joints.ts
//
// SO-100/101 servo-id → URDF joint mapping. `name` must match so101.urdf
// exactly; limits are the URDF's revolute <limit> values in radians.
export type ServoId = 1 | 2 | 3 | 4 | 5 | 6;

export interface JointSpec {
  id: ServoId;
  name: string;
  label: string;
  lowerRad: number;
  upperRad: number;
}

export const SO100_JOINTS: readonly JointSpec[] = [
  { id: 1, name: 'shoulder_pan',  label: 'Shoulder pan',  lowerRad: -1.91986, upperRad: 1.91986 },
  { id: 2, name: 'shoulder_lift', label: 'Shoulder lift', lowerRad: -1.74533, upperRad: 1.74533 },
  { id: 3, name: 'elbow_flex',    label: 'Elbow flex',    lowerRad: -1.69,    upperRad: 1.69 },
  { id: 4, name: 'wrist_flex',    label: 'Wrist flex',    lowerRad: -1.65806, upperRad: 1.65806 },
  { id: 5, name: 'wrist_roll',    label: 'Wrist roll',    lowerRad: -2.74385, upperRad: 2.84121 },
  { id: 6, name: 'gripper',       label: 'Gripper',       lowerRad: -0.17453, upperRad: 1.74533 },
];

export const degToRad = (deg: number): number => (deg * Math.PI) / 180;
export const radToDeg = (rad: number): number => (rad * 180) / Math.PI;

export function jointById(id: ServoId): JointSpec {
  const spec = SO100_JOINTS.find((j) => j.id === id);
  if (!spec) throw new Error(`unknown servo id: ${id}`);
  return spec;
}

export function clampRad(spec: JointSpec, rad: number): number {
  return Math.min(spec.upperRad, Math.max(spec.lowerRad, rad));
}

// dashboard/src/views/ArmDemo/Arm3DScene.tsx
import { useCallback, useEffect, useRef, useState, type ComponentRef, type RefObject } from 'react';
import * as THREE from 'three';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import { useUrdfRobot } from './useUrdfRobot';
import { applyMonochrome, setArmOpacity } from './applyMonochrome';
import { SO100_JOINTS, type ServoId } from './joints';
import { presetByKey, type PresetKey } from './cameraPresets';

const URDF_URL = '/models/so101/so101.urdf';

const cssVar = (name: string) =>
  getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const DEFAULT_FOV = 45;

// Module-level stable references. TeleopView re-renders at telemetry rate;
// passing fresh object/array literals on every render makes R3F's Canvas
// re-reconcile its renderer prefs each frame, which on Safari can race with
// WebRTC video decode and surface as "WebGLRenderer: Context Lost".
const CAMERA_PROPS = { position: [0.42, 0.38, 0.42] as const, fov: 45 };
const DPR_RANGE: [number, number] = [1, 1.5];
const GL_PROPS = {
  alpha: true,
  antialias: true,
  powerPreference: 'low-power' as const,
  failIfMajorPerformanceCaveat: false,
  stencil: false,
  depth: true,
  preserveDrawingBuffer: false,
};
const CANVAS_STYLE = { background: 'transparent' as const };

type OrbitImpl = ComponentRef<typeof OrbitControls>;

// Eases the orbit camera toward the active preset, then releases so the user
// can orbit freely until the next preset change. Self-drives invalidate() so
// it animates under frameloop="demand".
function CameraRig({ preset, controlsRef }: {
  preset: PresetKey;
  controlsRef: RefObject<OrbitImpl>;
}) {
  const camera = useThree((s) => s.camera) as THREE.PerspectiveCamera;
  const invalidate = useThree((s) => s.invalidate);
  const moving = useRef(false);
  const goalPos = useRef(new THREE.Vector3());
  const goalTarget = useRef(new THREE.Vector3());
  const goalFov = useRef(DEFAULT_FOV);

  useEffect(() => {
    const p = presetByKey(preset);
    goalPos.current.set(...p.pos);
    goalTarget.current.set(...p.target);
    goalFov.current = p.fov ?? DEFAULT_FOV;
    moving.current = true;
    invalidate();
  }, [preset, invalidate]);

  useFrame(() => {
    if (!moving.current) return;
    camera.position.lerp(goalPos.current, 0.18);
    camera.fov += (goalFov.current - camera.fov) * 0.18;
    camera.updateProjectionMatrix();
    const c = controlsRef.current;
    if (c) {
      c.target.lerp(goalTarget.current, 0.18);
      c.update();
    }
    if (camera.position.distanceTo(goalPos.current) < 0.004 && Math.abs(camera.fov - goalFov.current) < 0.05) {
      camera.fov = goalFov.current;
      camera.updateProjectionMatrix();
      moving.current = false;
      return;
    }
    invalidate();
  });

  return null;
}

// Lives inside Canvas so it can pull `invalidate` from the R3F store and push
// the URDF into the scene with frame-on-change semantics under "demand".
function RobotInScene({ robot, joints, opacity }: {
  robot: ReturnType<typeof useUrdfRobot>['robot'];
  joints: Record<ServoId, number>;
  opacity: number;
}) {
  const invalidate = useThree((s) => s.invalidate);

  useEffect(() => {
    if (!robot) return;
    // fg-2 base: legible over the window surface and over dark video alike.
    applyMonochrome(robot, { base: cssVar('--color-fg-2'), accent: cssVar('--color-fg') });
    invalidate();
  }, [robot, invalidate]);

  useEffect(() => {
    if (!robot) return;
    setArmOpacity(robot, opacity);
    invalidate();
  }, [robot, opacity, invalidate]);

  useEffect(() => {
    if (!robot) return;
    for (const spec of SO100_JOINTS) robot.setJointValue(spec.name, joints[spec.id] ?? 0);
    invalidate();
  }, [robot, joints, invalidate]);

  if (!robot) return null;
  return <primitive object={robot} />;
}

// Treats the GPU as fallible: prevents default on context loss so the browser
// will try to restore, and bumps a generation so the parent remounts the
// Canvas (and re-uploads the URDF) when restored. Also points the camera at
// the arm centroid once, after the GL context exists, so this work is not on
// the hot reconcile path (a fresh onCreated handler each render is enough to
// make R3F treat the Canvas as dirty).
function CanvasInit({ onLost, onRestored }: { onLost: () => void; onRestored: () => void }) {
  const gl = useThree((s) => s.gl);
  const camera = useThree((s) => s.camera) as THREE.PerspectiveCamera;
  useEffect(() => {
    camera.lookAt(0, 0.15, 0);
  }, [camera]);
  useEffect(() => {
    const canvas = gl.domElement;
    const lost = (e: Event) => { e.preventDefault(); onLost(); };
    const restored = () => onRestored();
    canvas.addEventListener('webglcontextlost', lost);
    canvas.addEventListener('webglcontextrestored', restored);
    return () => {
      canvas.removeEventListener('webglcontextlost', lost);
      canvas.removeEventListener('webglcontextrestored', restored);
    };
  }, [gl, onLost, onRestored]);
  return null;
}

export function Arm3DScene({ joints, preset, opacity }: {
  joints: Record<ServoId, number>;
  preset: PresetKey;
  opacity: number;
}) {
  const { robot, error } = useUrdfRobot(URDF_URL);
  const controls = useRef<OrbitImpl>(null!);

  // Bumping `generation` remounts Canvas after a context-restored event.
  const [generation, setGeneration] = useState(0);
  const onLost = useCallback(() => {
    if (typeof console !== 'undefined') console.warn('[arm3d] WebGL context lost');
  }, []);
  const onRestored = useCallback(() => {
    if (typeof console !== 'undefined') console.info('[arm3d] WebGL context restored — remounting canvas');
    setGeneration((g) => g + 1);
  }, []);

  if (error) {
    return (
      <div style={{
        position: 'absolute', inset: 0, display: 'flex', alignItems: 'center',
        justifyContent: 'center', textAlign: 'center', padding: '0 16px',
        fontFamily: 'var(--font-mono)', fontSize: 11, letterSpacing: '0.04em',
        color: 'var(--color-fg-4)',
      }}>
        3D model unavailable — {error.message || 'failed to load'}. Switch to 2d.
      </div>
    );
  }

  return (
    <Canvas
      key={generation}
      camera={CAMERA_PROPS}
      dpr={DPR_RANGE}
      gl={GL_PROPS}
      frameloop="demand"
      style={CANVAS_STYLE}
    >
      <CanvasInit onLost={onLost} onRestored={onRestored} />
      <ambientLight intensity={0.7} />
      <directionalLight position={[3, 5, 2]} intensity={1.1} />
      <RobotInScene robot={robot} joints={joints} opacity={opacity} />
      <OrbitControls ref={controls} target={[0, 0.15, 0]} enableDamping makeDefault />
      <CameraRig preset={preset} controlsRef={controls} />
    </Canvas>
  );
}

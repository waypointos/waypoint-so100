// dashboard/src/views/ArmDemo/useUrdfRobot.ts
//
// Loads a URDF (with STL meshes) into a THREE object once per url. The URDF is
// Z-up (ROS); three is Y-up, so the robot is rotated to stand upright. The
// robot is published only after the manager reports all meshes loaded, so
// callers' material overrides see every mesh (meshes attach async, after the
// loader's own onComplete). On unmount, all geometries and materials are
// disposed so repeated open/close of the overlay doesn't leak GPU memory.
import { useEffect, useState } from 'react';
import * as THREE from 'three';
import URDFLoader, { type URDFRobot } from 'urdf-loader';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

export interface UrdfState {
  robot: URDFRobot | null;
  error: Error | null;
}

function disposeRobot(robot: THREE.Object3D) {
  robot.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!('isMesh' in mesh) || !(mesh as THREE.Mesh).isMesh) return;
    mesh.geometry?.dispose();
    const m = mesh.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(m)) m.forEach((x) => x.dispose());
    else m?.dispose();
  });
}

export function useUrdfRobot(url: string): UrdfState {
  const [state, setState] = useState<UrdfState>({ robot: null, error: null });

  useEffect(() => {
    let cancelled = false;
    let built: URDFRobot | null = null;
    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.loadMeshCb = (path, _m, done) => {
      new STLLoader(manager).load(
        path,
        (geom) => {
          // Grey at creation; wrap in a Group so urdf-loader leaves the material
          // alone (it forces the URDF's <material> colour only onto a bare Mesh).
          const grey = getComputedStyle(document.documentElement)
            .getPropertyValue('--color-fg-3').trim();
          const mesh = new THREE.Mesh(
            geom,
            new THREE.MeshStandardMaterial({ color: new THREE.Color(grey), metalness: 0.1, roughness: 0.6 }),
          );
          const wrapper = new THREE.Group();
          wrapper.add(mesh);
          done(wrapper);
        },
        undefined,
        (err) => done(null as unknown as THREE.Object3D, err as Error),
      );
    };
    manager.onLoad = () => {
      if (!cancelled && built) setState({ robot: built, error: null });
    };
    loader.load(
      url,
      (robot) => {
        robot.rotation.x = -Math.PI / 2;
        built = robot;
      },
      undefined,
      (err) => { if (!cancelled) setState({ robot: null, error: err as Error }); },
    );
    return () => {
      cancelled = true;
      if (built) disposeRobot(built);
    };
  }, [url]);

  return state;
}

// dashboard/src/views/ArmDemo/applyMonochrome.ts
//
// Override all mesh materials under a loaded robot with flat monochrome
// standard materials. Colors are passed in (resolved from design tokens by the
// caller) so this stays free of inline hex.
import * as THREE from 'three';

export interface MonochromePalette {
  base: string;
  accent: string;
}

export function applyMonochrome(
  root: THREE.Object3D,
  palette: MonochromePalette,
  accentLinks: Set<string> = new Set(),
): void {
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) return;
    const color = isUnderAccentLink(mesh, accentLinks) ? palette.accent : palette.base;
    mesh.material = new THREE.MeshStandardMaterial({
      color: new THREE.Color(color),
      metalness: 0.1,
      roughness: 0.6,
    });
  });
}

// Update the opacity of every mesh material in place (no re-create). depthWrite
// stays on even when transparent so each mesh shows only its (backface-culled)
// outer shell — without it, inner faces render unsorted and the arm z-fights.
export function setArmOpacity(root: THREE.Object3D, opacity: number): void {
  const transparent = opacity < 1;
  root.traverse((obj) => {
    const mesh = obj as THREE.Mesh;
    if (!mesh.isMesh) return;
    const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const mat of mats) {
      mat.transparent = transparent;
      mat.opacity = opacity;
      mat.depthWrite = true;
      mat.needsUpdate = true;
    }
  });
}

function isUnderAccentLink(obj: THREE.Object3D, accentLinks: Set<string>): boolean {
  let cur: THREE.Object3D | null = obj;
  while (cur) {
    if (cur.name && accentLinks.has(cur.name)) return true;
    cur = cur.parent;
  }
  return false;
}

// dashboard/src/arm/Arm2DScene.tsx
//
// Canvas schematic of the arm: side elevation (reach plane foreshortened by
// shoulder pan) or top-down plan. Cheap alternative to the URDF scene; N/A
// joints draw at rest with a muted "?" marker at the affected joint.
import { useEffect, useRef } from 'react';
import { SO100_JOINTS, type ServoId } from './joints';
import type { Readings } from './ArmTeleop';
import styles from './Arm2DScene.module.css';

export type FlatView = 'side' | 'plan';

// Link lengths in metres, schematic-true to SO-100 proportions.
const L = { base: 0.055, l1: 0.112, l2: 0.108, l3: 0.085, jaw: 0.05 };

const cssVar = (name: string) =>
  getComputedStyle(document.documentElement).getPropertyValue(name).trim();

type Chain = { S: [number, number]; E: [number, number]; W: [number, number]; EE: [number, number]; a3: number };

function chain(pose: Record<ServoId, number>): Chain {
  const S: [number, number] = [0, L.base];
  const a1 = Math.PI / 2 - pose[2];
  const E: [number, number] = [S[0] + L.l1 * Math.cos(a1), S[1] + L.l1 * Math.sin(a1)];
  const a2 = a1 + pose[3];
  const W: [number, number] = [E[0] + L.l2 * Math.cos(a2), E[1] + L.l2 * Math.sin(a2)];
  const a3 = a2 + pose[4];
  const EE: [number, number] = [W[0] + L.l3 * Math.cos(a3), W[1] + L.l3 * Math.sin(a3)];
  return { S, E, W, EE, a3 };
}

function draw(cv: HTMLCanvasElement, readings: Readings, view: FlatView) {
  const ctx = cv.getContext('2d');
  if (!ctx) return;
  const dpr = window.devicePixelRatio || 1;
  const vw = cv.clientWidth, vh = cv.clientHeight;
  cv.width = Math.max(1, vw * dpr);
  cv.height = Math.max(1, vh * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, vw, vh);

  const C = {
    border: cssVar('--color-border'), fg2: cssVar('--color-fg-2'),
    fg3: cssVar('--color-fg-3'), fg4: cssVar('--color-fg-4'),
    accent: cssVar('--color-accent'), caution: cssVar('--color-caution'),
  };
  const mono = cssVar('--font-mono');

  const pose = {} as Record<ServoId, number>;
  for (const j of SO100_JOINTS) pose[j.id] = readings[j.id]?.angleRad ?? 0;
  const naIds = SO100_JOINTS.filter((j) => readings[j.id]?.angleRad == null).map((j) => j.id);

  // grid
  ctx.strokeStyle = C.border; ctx.globalAlpha = 0.55; ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = (vw / 2) % 26; x < vw; x += 26) { ctx.moveTo(x, 0); ctx.lineTo(x, vh); }
  for (let y = vh - 24; y > 0; y -= 26) { ctx.moveTo(0, y); ctx.lineTo(vw, y); }
  ctx.stroke(); ctx.globalAlpha = 1;

  const scale = Math.min(vw, vh) * 1.05;
  const k = chain(pose);

  if (view === 'plan') {
    const cx = vw * 0.45, cy = vh / 2;
    ctx.strokeStyle = C.fg4; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, 0.034 * scale, 0, Math.PI * 2); ctx.stroke();
    const reach = k.EE[0];
    const ex = cx + reach * scale * Math.cos(pose[1]);
    const ey = cy + reach * scale * Math.sin(pose[1]);
    ctx.strokeStyle = C.fg2; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke();
    ctx.strokeStyle = C.accent; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.arc(ex, ey, 5, 0, Math.PI * 2); ctx.stroke();
    return;
  }

  // side elevation, reach foreshortened by pan
  const gy = vh - 24;
  ctx.strokeStyle = C.fg4;
  ctx.beginPath(); ctx.moveTo(14, gy); ctx.lineTo(vw - 14, gy); ctx.stroke();

  const ox = vw * 0.42;
  const fore = Math.cos(pose[1]);
  const P = (x: number, y: number): [number, number] => [ox + x * fore * scale, gy - y * scale];

  const [bx, by] = [ox, gy];
  ctx.strokeStyle = C.fg3; ctx.lineWidth = 1.4;
  ctx.strokeRect(bx - 17, by - 10, 34, 10);

  ctx.strokeStyle = C.fg2; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(ox, gy);
  for (const p of [k.S, k.E, k.W, k.EE]) ctx.lineTo(...P(p[0], p[1]));
  ctx.stroke();

  const spread = 0.07 + Math.max(0, pose[6]) * 0.35;
  ctx.lineWidth = 1.4;
  for (const s of [+1, -1]) {
    const a = k.a3 + s * spread;
    ctx.beginPath();
    ctx.moveTo(...P(k.EE[0], k.EE[1]));
    ctx.lineTo(...P(k.EE[0] + L.jaw * Math.cos(a), k.EE[1] + L.jaw * Math.sin(a)));
    ctx.stroke();
  }

  for (const p of [k.S, k.E, k.W]) {
    const [jx, jy] = P(p[0], p[1]);
    ctx.beginPath(); ctx.arc(jx, jy, 4.5, 0, Math.PI * 2);
    ctx.fillStyle = cssVar('--color-bg'); ctx.fill();
    ctx.strokeStyle = C.fg2; ctx.lineWidth = 1.2; ctx.stroke();
  }

  if (naIds.length > 0) {
    const [wx, wy] = P(k.W[0], k.W[1]);
    ctx.fillStyle = C.fg4; ctx.font = `10px ${mono}`;
    ctx.fillText(`${naIds.length} N/A`, wx + 8, wy - 8);
  }

  const [ex, ey] = P(k.EE[0], k.EE[1]);
  ctx.strokeStyle = C.accent; ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(ex - 7, ey); ctx.lineTo(ex - 2, ey); ctx.moveTo(ex + 2, ey); ctx.lineTo(ex + 7, ey);
  ctx.moveTo(ex, ey - 7); ctx.lineTo(ex, ey - 2); ctx.moveTo(ex, ey + 2); ctx.lineTo(ex, ey + 7);
  ctx.stroke();
}

export function Arm2DScene({ readings, view }: { readings: Readings; view: FlatView }) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const cv = ref.current;
    if (!cv) return;
    draw(cv, readings, view);
  }, [readings, view]);

  useEffect(() => {
    const cv = ref.current;
    if (!cv || typeof ResizeObserver === 'undefined') return;
    const ro = new ResizeObserver(() => draw(cv, readings, view));
    ro.observe(cv);
    return () => ro.disconnect();
    // Redraw-on-resize only; data redraws come from the effect above.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return <canvas ref={ref} className={styles.canvas} data-arm-2d />;
}

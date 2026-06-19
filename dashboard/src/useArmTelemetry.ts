// Live telemetry hooks for the Arm tab (the [ui.static] panel surface).
//
// The tab subscribes to two private module subjects:
//   • module.so100.joints — calibrated joint angles (N/A + reason when a joint
//     is uncalibrated or unread), feeding the JOINTS and 3D cards.
//   • module.so100.stats  — full per-servo telemetry (position/speed/load/
//     current/voltage/temp + flags), feeding the MOTOR DETAIL card.
// Both carry their own "absent" semantics; we never substitute fake zeros.
import { useEffect, useState } from 'react';
import { useBridge } from './bridge';
import { JointAngles, ServoStats } from './proto/so100_pb';
import { SO100_JOINTS, type ServoId } from './arm/joints';
import type { Readings } from './arm/ArmTeleop';

const AWAITING: Readings = Object.fromEntries(
  SO100_JOINTS.map((j) => [j.id, { angleRad: null, naReason: 'awaiting telemetry' }]),
) as Readings;

export function useJoints(): { readings: Readings; lastAtMs: number | null } {
  const { roverId, subscribe } = useBridge();
  const [readings, setReadings] = useState<Readings>(AWAITING);
  const [lastAtMs, setLastAtMs] = useState<number | null>(null);
  useEffect(() => {
    return subscribe(`waypoint.${roverId}.module.so100.joints`, (b) => {
      let ja: JointAngles;
      try { ja = JointAngles.fromBinary(b); } catch { return; }
      setReadings((prev) => {
        const next = { ...prev };
        for (const j of ja.joints) {
          next[j.id as ServoId] = j.angleRad !== undefined
            ? { angleRad: j.angleRad, naReason: null }
            : { angleRad: null, naReason: j.naReason || 'no data' };
        }
        return next;
      });
      setLastAtMs(Date.now());
    });
  }, [roverId, subscribe]);
  return { readings, lastAtMs };
}

/** One servo's live registers; optional scalars are absent (→ N/A) on a bad read. */
export type ServoSample = {
  positionRaw?: number;
  speedRaw?: number;
  loadRaw?: number;
  currentRaw?: number;
  voltageDeci?: number;
  temperatureC?: number;
  moving: boolean;
  overcurrentTripped: boolean;
  ok: boolean;
};

export type Stats = Partial<Record<ServoId, ServoSample>>;

export function useStats(): { stats: Stats; lastAtMs: number | null } {
  const { roverId, subscribe } = useBridge();
  const [stats, setStats] = useState<Stats>({});
  const [lastAtMs, setLastAtMs] = useState<number | null>(null);
  useEffect(() => {
    return subscribe(`waypoint.${roverId}.module.so100.stats`, (b) => {
      let st: ServoStats;
      try { st = ServoStats.fromBinary(b); } catch { return; }
      const next: Stats = {};
      for (const s of st.servos) {
        next[s.servoId as ServoId] = {
          positionRaw: s.positionRaw,
          speedRaw: s.speedRaw,
          loadRaw: s.loadRaw,
          currentRaw: s.currentRaw,
          voltageDeci: s.voltageDeci,
          temperatureC: s.temperatureC,
          moving: s.moving,
          overcurrentTripped: s.overcurrentTripped,
          ok: s.ok,
        };
      }
      setStats(next);
      setLastAtMs(Date.now());
    });
  }, [roverId, subscribe]);
  return { stats, lastAtMs };
}

/** Re-renders on a 500 ms tick so feed-age labels stay current between frames. */
export function useAgeMs(lastAtMs: number | null): number | null {
  const [, bump] = useState(0);
  useEffect(() => {
    const id = setInterval(() => bump((n) => n + 1), 500);
    return () => clearInterval(id);
  }, []);
  return lastAtMs === null ? null : Date.now() - lastAtMs;
}

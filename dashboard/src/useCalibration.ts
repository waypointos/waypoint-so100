import { useEffect, useState } from 'react';
import { useBridge } from './bridge';
import { CalibrationState } from './proto/so100_pb';

export function useCalibration() {
  const { roverId, subscribe } = useBridge();
  const [state, setState] = useState<CalibrationState | null>(null);
  useEffect(() => {
    return subscribe(`waypoint.${roverId}.module.so100.calibration`, (b) => {
      try { setState(CalibrationState.fromBinary(b)); } catch { /* ignore malformed */ }
    });
  }, [roverId, subscribe]);
  return state;
}

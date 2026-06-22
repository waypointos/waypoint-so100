import { useEffect, useState } from 'react';
import { useBridge } from './bridge';
import { PoseState } from './proto/so100_pb';

// usePoses subscribes to the module's poses subject, reporting which slots
// ("share" / "options") currently hold a saved pose. Mirrors useCalibration.
export function usePoses() {
  const { roverId, subscribe } = useBridge();
  const [state, setState] = useState<PoseState | null>(null);
  useEffect(() => {
    return subscribe(`waypoint.${roverId}.module.so100.poses`, (b) => {
      try { setState(PoseState.fromBinary(b)); } catch { /* ignore malformed */ }
    });
  }, [roverId, subscribe]);
  return state;
}

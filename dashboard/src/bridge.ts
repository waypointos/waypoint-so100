import { createContext, useContext } from 'react';

export type Subscribe = (subject: string, onBytes: (b: Uint8Array) => void) => () => void;
export type Publish = (subject: string, bytes: Uint8Array) => void;

type Bridge = { roverId: string; subscribe: Subscribe; publish: Publish };
const BridgeContext = createContext<Bridge | null>(null);
export const BridgeProvider = BridgeContext.Provider;

export function useBridge(): Bridge {
  const b = useContext(BridgeContext);
  if (!b) throw new Error('useBridge must be used inside a mounted module');
  return b;
}

import { createRoot } from 'react-dom/client';
import { BridgeProvider } from './bridge';
import { ArmPanel } from './ArmPanel';
import './ui/tokens.css';

type ModuleContext = {
  roverId: string;
  subscribe: (subject: string, onBytes: (b: Uint8Array) => void) => () => void;
  publish: (subject: string, bytes: Uint8Array) => void;
  session?: { role: string };
};

export default {
  mount(container: HTMLElement, ctx: ModuleContext): () => void {
    const root = createRoot(container);
    root.render(
      <BridgeProvider value={{ roverId: ctx.roverId, subscribe: ctx.subscribe, publish: ctx.publish }}>
        <ArmPanel />
      </BridgeProvider>,
    );
    return () => root.unmount();
  },
};

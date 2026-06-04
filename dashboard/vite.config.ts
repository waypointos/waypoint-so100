/// <reference types="vitest/config" />
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import cssInjectedByJsPlugin from 'vite-plugin-css-injected-by-js';
import { fileURLToPath, URL } from 'node:url';

// Each module entry is loaded standalone by the host via a single dynamic
// import(), so every output must be a self-contained ESM bundle (no shared
// chunks). Vite's lib mode can only inline dynamic imports for one entry at a
// time, so a build pass targets one entry chosen by WAYPOINT_ENTRY.
const ENTRIES: Record<string, string> = { panel: './src/mount.tsx', teleop: './src/teleop.tsx' };
const entry = process.env.WAYPOINT_ENTRY ?? 'panel';

export default defineConfig(({ command }) => ({
  plugins: [react(), cssInjectedByJsPlugin()],
  // Vite library mode does not substitute process.env.NODE_ENV the way app
  // builds do. The panel is loaded standalone via dynamic import(), so bundled
  // React's raw process.env.NODE_ENV reads would throw in the browser. Define
  // it for builds only (not the vitest run, which needs development React).
  ...(command === 'build'
    ? { define: { 'process.env.NODE_ENV': JSON.stringify('production') } }
    : {}),
  build: {
    outDir: 'dist',
    emptyOutDir: false, // two passes write panel.js and teleop.js side by side
    cssCodeSplit: false,
    lib: {
      entry: fileURLToPath(new URL(ENTRIES[entry], import.meta.url)),
      formats: ['es'],
      fileName: () => `${entry}.js`,
    },
    rollupOptions: { output: { inlineDynamicImports: true } },
  },
  test: { environment: 'jsdom', globals: true, setupFiles: ['./src/test-setup.ts'] },
}));

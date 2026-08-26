import { resolve } from 'path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// base:'./' → relative asset paths so the built bundle works served from the backend at '/'.
// The dev proxy lets `npm run dev` (:5173) reach the backend (:8000) with same-origin URLs.
//
// TWO entry points. `xr/` is the Quest's WebXR page, deliberately built as its own bundle:
// it is loaded by a headset over a separate TLS listener, has no React and no shared UI, and
// must stay small and dependency-free. Emits dist/xr/index.html, served at /xr.
export default defineConfig({
  plugins: [react()],
  base: './',
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'index.html'),
        xr: resolve(__dirname, 'xr/index.html'),
      },
    },
  },
  server: {
    port: 5173,
    strictPort: true,
    proxy: {
      '/api': 'http://localhost:8000',
      '/auth': 'http://localhost:8000',
      '/ws': { target: 'ws://localhost:8000', ws: true },
    },
  },
})

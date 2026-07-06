import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// base:'./' → relative asset paths so the built bundle works served from the backend at '/'.
// The dev proxy lets `npm run dev` (:5173) reach the backend (:8000) with same-origin URLs.
export default defineConfig({
  plugins: [react()],
  base: './',
  build: { outDir: 'dist', emptyOutDir: true },
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

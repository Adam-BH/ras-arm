/// <reference types="vitest" />
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// base './' → the built site works from any folder (GitHub Pages)
export default defineConfig({
  base: './',
  plugins: [react()],
  test: { environment: 'node' },
});

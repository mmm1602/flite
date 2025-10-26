// vite.config.js
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  // Use a relative base so built HTML references ./assets/... — works on Vercel and GitHub Pages.
  // If you need an absolute repo path for GitHub Pages, set it at build time instead.
  base: './',
})
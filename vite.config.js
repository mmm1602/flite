// vite.config.js
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  base: '/<repo-name>/', // e.g. '/flite/' if this is NOT mmm1602.github.io
})

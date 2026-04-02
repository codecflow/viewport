import { defineConfig } from 'vite';

export default defineConfig({
  server: { port: 5200 },
  resolve: {
    alias: {
      'three/addons': 'three/examples/jsm',
    }
  }
});

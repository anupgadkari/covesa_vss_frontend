import { defineConfig } from '@playwright/test';

export default defineConfig({
  testDir: '.',
  timeout: 20_000,
  retries: 0,
  use: {
    baseURL: 'http://localhost:3000',
    headless: true,
  },
  // Serve the HMI files via a simple static HTTP server so CDN scripts
  // (React, Babel standalone) can load correctly.
  webServer: {
    command: 'npx serve -l 3000 ../.. --no-clipboard',
    port: 3000,
    reuseExistingServer: true,
    timeout: 10_000,
  },
  projects: [
    {
      name: 'chromium',
      use: { browserName: 'chromium' },
    },
  ],
});

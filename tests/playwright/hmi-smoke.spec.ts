/**
 * Tier 3 — Playwright smoke tests for the VSS HMI.
 *
 * Prerequisites:
 *   1. vss-bridge running on localhost:8080
 *   2. vss-hmi-body-sensors.html served (or opened via file://)
 *
 * These tests verify that the HMI binds the right VSS signals and
 * reacts to state changes.  They do NOT assert blink cadence —
 * timing-sensitive checks are handled by Tier 1 (virtual time) and
 * Tier 2 (WS client with wall-clock tolerances).
 *
 * Run:
 *   cd tests/playwright
 *   npm install && npx playwright install chromium
 *   npm test
 */

import { test, expect } from '@playwright/test';
import { WebSocket } from 'ws';
import * as path from 'path';

const WS_URL = 'ws://127.0.0.1:8080';
const HMI_URL = '/vss-hmi-body-sensors.html';

/** Send a sensor message to the bridge via a raw WS connection. */
function sendSensor(ws: WebSocket, signalPath: string, value: unknown): Promise<void> {
  return new Promise((resolve, reject) => {
    const msg = JSON.stringify({ type: 'sensor', path: signalPath, value });
    ws.send(msg, (err) => (err ? reject(err) : resolve()));
  });
}

/** Wait until the bridge WS is reachable (up to 3 s). */
async function waitForBridge(): Promise<WebSocket> {
  const deadline = Date.now() + 3000;
  while (Date.now() < deadline) {
    try {
      const ws = new WebSocket(WS_URL);
      await new Promise<void>((resolve, reject) => {
        ws.once('open', resolve);
        ws.once('error', reject);
      });
      return ws;
    } catch {
      await new Promise((r) => setTimeout(r, 100));
    }
  }
  throw new Error(`Bridge not reachable at ${WS_URL}`);
}

test.describe('HMI Smoke Tests', () => {
  let controlWs: WebSocket;

  test.beforeAll(async () => {
    controlWs = await waitForBridge();
  });

  test.afterAll(async () => {
    controlWs?.close();
  });

  test('HMI loads and renders the Actuator Outputs section', async ({ page }) => {
    await page.goto(HMI_URL);
    // Wait for React to render (the HMI uses Babel standalone, may take a moment).
    await page.waitForTimeout(2000);
    await expect(page.locator('text=Actuator Outputs')).toBeVisible();
  });

  test('hazard switch turns on both indicator lamp groups', async ({ page }) => {
    await page.goto(HMI_URL);
    await page.waitForTimeout(2000);

    // Engage hazard via control WS
    await sendSensor(controlWs, 'Body.Switches.Hazard.IsEngaged', true);

    // Wait for the lamps to start blinking — within 1s we should see
    // at least one amber circle (background-color change).
    // The Front lamp circle for the left side should go amber.
    await page.waitForTimeout(1500);

    // Check that the LEFT label shows active (not "idle")
    const leftStatus = page.locator('text=1.5 Hz').first();
    await expect(leftStatus).toBeVisible({ timeout: 3000 });
  });

  test('hazard off returns lamps to idle', async ({ page }) => {
    await page.goto(HMI_URL);
    await page.waitForTimeout(2000);

    // Engage then disengage
    await sendSensor(controlWs, 'Body.Switches.Hazard.IsEngaged', true);
    await page.waitForTimeout(1000);
    await sendSensor(controlWs, 'Body.Switches.Hazard.IsEngaged', false);
    await page.waitForTimeout(1000);

    // Both sides should show "idle"
    const idleLabels = page.locator('text=idle');
    await expect(idleLabels.first()).toBeVisible({ timeout: 3000 });
  });

  test('defect toggle changes label to show defect', async ({ page }) => {
    await page.goto(HMI_URL);
    await page.waitForTimeout(2000);

    // Set ignition ON + stalk LEFT via control WS
    await sendSensor(controlWs, 'Vehicle.LowVoltageSystemState', 'ON');
    await sendSensor(controlWs, 'Body.Switches.TurnIndicator.Direction', 'LEFT');
    await page.waitForTimeout(500);

    // Inject a front lamp defect
    await sendSensor(
      controlWs,
      'Body.Lights.DirectionIndicator.Left.Lamp.Front.IsDefect',
      true,
    );
    await page.waitForTimeout(500);

    // The left side label should now say "DEFECT · 3 Hz"
    const defectLabel = page.locator('text=DEFECT');
    await expect(defectLabel.first()).toBeVisible({ timeout: 3000 });
  });
});

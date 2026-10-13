import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import ManualDrivePanel from './ManualDrivePanel';

// HH_261007 - Keep block/freshness feedback visible in the folded panel while
// retaining the real deadman/zero/disarm controls and backend-owned limits.
let host, root, sockets, originalWebSocket, now;
class TestSocket {
  static OPEN = 1;
  constructor() { this.readyState = 1; this.send = jest.fn(); this.close = jest.fn(); sockets.push(this); }
}
const emit = async changes => {
  await act(async () => sockets[0].onmessage({ data: JSON.stringify({ type: 'state', manual_drive: {
    available: true, connected: true, armed: true, holding: false,
    limits: { linear_x_mps: 2, lateral_y_mps: 1, angular_z_radps: 0.7853 },
    ...changes,
  } }) }));
};
const gate = changes => ({ fresh: true, blocked: true, level: 1,
  operating_state: 'ROUTE_SAFETY_HOLD', receipt_age_s: 0.1, source_age_s: 0.1,
  freshness_timeout_s: 1.5, reasons: ['lanelet_physical_body_cost'], ...changes });

beforeEach(async () => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  now = 10;
  jest.spyOn(performance, 'now').mockImplementation(() => now);
  sockets = [];
  originalWebSocket = global.WebSocket; global.WebSocket = TestSocket;
  host = document.createElement('div'); document.body.appendChild(host);
  root = createRoot(host);
  await act(async () => root.render(<ManualDrivePanel />));
});
afterEach(() => {
  act(() => root.unmount()); host.remove();
  global.WebSocket = originalWebSocket;
  delete global.IS_REACT_ACT_ENVIRONMENT;
  jest.restoreAllMocks(); jest.useRealTimers();
});

test('folded ARMED panel shows boundary restriction without disabling zero or disarm', async () => {
  await emit({ safety_gate: gate({ recovery_reason: 'manual_projected_recovery' }) });
  expect(host.querySelector('[data-ui="manual-drive-state"]').textContent).toContain('ARMED · 경계 탈출 제한');
  expect(host.querySelector('[data-ui="manual-drive-gate-message"]').textContent).toContain('안전한 탈출 입력만 허용');
  expect(host.querySelector('[data-ui="manual-drive-zero"]').disabled).toBe(false);
  expect(host.querySelector('[data-ui="manual-drive-disarm"]').disabled).toBe(false);
  expect(host.querySelector('[data-ui="manual-drive-panel"]').dataset.linearLimitMps).toBe('2');
  expect(sockets[0].send).not.toHaveBeenCalled();
});

test('fresh admission expires locally while the socket remains connected', async () => {
  await emit({ safety_gate: gate({ blocked: false, level: 0, operating_state: 'ENABLED', reasons: [] }) });
  expect(host.querySelector('[data-ui="manual-drive-state"]').textContent).toContain('명령 허용');
  now = 2010;
  await act(async () => jest.advanceTimersByTime(250));
  expect(host.querySelector('[data-ui="manual-drive-state"]').textContent).toContain('상태 미확인');
  expect(host.querySelector('[data-ui="manual-drive-panel"]').dataset.connected).toBe('true');
  for (const [frame] of sockets[0].send.mock.calls) {
    const command = JSON.parse(frame);
    expect(command.type).toBe('drive');
    expect([command.forward, command.crab, command.turn]).toEqual([0, 0, 0]);
  }
});

test('legacy or absent gate data cannot make ARMED look permitted', async () => {
  await emit({});
  expect(host.querySelector('[data-ui="manual-drive-state"]').textContent).toContain('상태 미확인');
  await emit({ armed: false });
  expect(host.querySelector('[data-ui="manual-drive-state"]').textContent).toBe('STANDBY');
  expect(host.querySelector('[data-ui="manual-drive-gate-message"]')).toBeNull();
});

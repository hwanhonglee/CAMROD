import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import DrivingMonitor from './DrivingMonitor';

jest.mock('./RangerNavigationScene', () => () => null);

test('the monitor only reads driving data and cannot stop or return a mission', async () => {
  const originalFetch = global.fetch;
  const socket = jest.spyOn(window, 'WebSocket');
  const host = document.createElement('div');
  const parentControl = document.createElement('button');
  const root = createRoot(host);
  document.body.append(parentControl, host);
  parentControl.focus();
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  const body = { schema_version: 1, connected: true,
    mission: { active: true, generation: 42, site: 'B7', intent: 'delivery',
      service_state_name: 'MOVING_TO_SITE', phase: 'DRIVING' },
    motion: { speed_mps: 0.5 } };
  global.fetch = jest.fn().mockResolvedValue({ ok: true, json: async () => body });
  try {
    await act(async () => root.render(<DrivingMonitor theme="dark" />));
    const display = host.querySelector('[data-testid="driving-display"]');
    expect(display.classList.contains('dd-theme-dark')).toBe(true);
    expect(display.textContent).toContain('B7');
    expect(display.textContent).toContain('1.8');
    expect(display.textContent).toContain('보기 전용');
    expect(display.querySelector('.dd-stop-action, .dd-back-action')).toBeNull();
    expect(document.activeElement).toBe(parentControl);
    expect(socket).not.toHaveBeenCalled();
    const signal = fetch.mock.calls[0][1].signal;

    fetch.mockResolvedValue({ ok: true, json: async () => ({ ...body, connected: false }) });
    await act(async () => jest.advanceTimersByTime(100));
    expect(display.textContent).toContain('연결 끊김');
    expect(display.querySelector('[data-testid="driving-speed"]').textContent).not.toContain('1.8');
    expect(fetch.mock.calls).toHaveLength(2);
    for (const [url, request] of fetch.mock.calls) {
      expect(url).toBe('/api/driving');
      expect(request.method).toBe('GET');
    }
    await act(async () => root.render(null));
    expect(signal.aborted).toBe(false);
    expect(fetch.mock.calls[1][1].signal.aborted).toBe(true);
    await act(async () => jest.advanceTimersByTime(1000));
    expect(fetch.mock.calls).toHaveLength(2);
  } finally {
    act(() => root.unmount());
    host.remove();
    parentControl.remove();
    socket.mockRestore();
    global.fetch = originalFetch;
    delete global.IS_REACT_ACT_ENVIRONMENT;
    jest.useRealTimers();
  }
});

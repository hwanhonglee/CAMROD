import React from 'react';
import { createRoot } from 'react-dom/client';
import { act as legacyAct } from 'react-dom/test-utils';
import { DiagnosticsMonitor } from './App';
import { ADMIN_DIAGNOSTIC_CATEGORIES, getAdminDiagnosticView } from './AdminNavigation';

// HH_261026 - Keep navigation tests independent of the live telemetry stream and GPU.
jest.mock('./RangerNavigationScene', () => function MockNavigationScene() {
  return <div />;
});
jest.mock('./TelemetryWorkspace', () => ({
  ...jest.requireActual('./TelemetryWorkspace'),
  __esModule: true,
  default: function MockTelemetryWorkspace({ activeTab, adminLoginId }) {
    return <div data-ui="test-telemetry-view" data-tab={activeTab} data-admin={adminLoginId} />;
  },
}));
jest.mock('./SnapshotControl', () => function MockSnapshotControl() {
  return <div data-ui="test-snapshot-view">스냅샷 저장</div>;
});

const act = React.act || legacyAct;
const tabs = ['system', 'gnss', 'proximity', 'camera', 'trajectory', 'perception', 'safety', 'docking', 'snapshot'];
const diagnostics = [
  { level: 2, name: 'sensor/error', message: '연결을 확인하세요', values: [{ key: 'source', value: 'front' }] },
  { level: 1, name: 'sensor/warn', message: '수신 지연' },
  { level: 0, name: 'sensor/ok', message: '정상' },
];
let host, root, originalFetch;

async function render(props = {}) {
  await act(async () => root.render(<DiagnosticsMonitor adminLoginId="operator" adminLoginPw="unchanged"
    serviceStateName="DROP_ZONE_WAIT" {...props} />));
}

async function tap(element) {
  expect(element).not.toBeNull();
  await act(async () => element.click());
}

const tabButton = id => host.querySelector(`[data-ui="operator-diagnostic-tab-${id}"]`);
const posts = () => fetch.mock.calls.filter(([, options]) => options?.method === 'POST');
const diagnosticReads = () => fetch.mock.calls.filter(([url]) => url === '/api/diagnostics');

beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  jest.useFakeTimers();
  originalFetch = global.fetch;
  global.fetch = jest.fn(async url => ({
    ok: true,
    json: async () => url === '/api/diagnostics' ? { status: diagnostics }
      : url === '/ui/platform_tuning' ? { success: true, available: true, steering_transition_rate_radps: 1 }
        : { success: true },
  }));
  host = document.createElement('div');
  document.body.appendChild(host);
  root = createRoot(host);
});

afterEach(() => {
  act(() => root.unmount());
  host.remove();
  global.fetch = originalFetch;
  jest.useRealTimers();
  delete global.IS_REACT_ACT_ENVIRONMENT;
});

test('five categories retain every existing diagnostic ID exactly once', async () => {
  await render();
  expect(ADMIN_DIAGNOSTIC_CATEGORIES.map(category => category.label)).toEqual([
    '주행·안전', '센서', '지도·인지', '주차·도킹', '시스템',
  ]);
  expect(ADMIN_DIAGNOSTIC_CATEGORIES.flatMap(category => category.tabIds).sort()).toEqual([...tabs].sort());
  tabs.forEach(id => expect(host.querySelectorAll(`[data-ui="operator-diagnostic-tab-${id}"]`)).toHaveLength(1));
  expect(tabButton('system').getAttribute('aria-selected')).toBe('true');
  expect(host.querySelector('.admin-view-heading h2').textContent).toBe('시스템 상태');
  expect(host.querySelector('.admin-system-controls').textContent).toContain('복귀 · 자동 주차');
  expect(posts()).toHaveLength(0);
});

test('every category selects its existing view and title without sending a command', async () => {
  await render();
  for (const id of ['safety', 'trajectory', 'gnss', 'proximity', 'camera', 'perception', 'docking', 'snapshot', 'system']) {
    await tap(tabButton(id));
    expect(host.querySelector('.admin-view-heading h2').textContent).toBe(getAdminDiagnosticView(id).title);
    expect(tabButton(id).getAttribute('aria-selected')).toBe('true');
    expect(host.querySelectorAll('[role="tab"][aria-selected="true"]')).toHaveLength(1);
    if (id === 'snapshot') {
      expect(host.querySelector('[data-ui="test-snapshot-view"]')).not.toBeNull();
    } else if (id !== 'system') {
      const view = host.querySelector('[data-ui="test-telemetry-view"]');
      expect(view.getAttribute('data-tab')).toBe(id);
      expect(view.getAttribute('data-admin')).toBe('operator');
    }
  }
  expect(posts()).toHaveLength(0);
});

test('system details remain available, selectable and collapsible with the original refresh policy', async () => {
  await render();
  const details = host.querySelector('.admin-diagnostic-details');
  expect(details.open).toBe(true);
  expect(details.querySelector('summary').textContent).toContain('오류 1');
  await tap([...host.querySelectorAll('.diag-item')].find(button => button.textContent.includes('sensor/error')));
  expect(host.querySelector('.diag-detail').textContent).toContain('연결을 확인하세요');
  expect(host.querySelector('.diag-kv-table').textContent).toContain('sourcefront');
  await tap(details.querySelector('summary'));
  expect(details.open).toBe(false);
  await act(async () => jest.advanceTimersByTime(500));
  expect(diagnosticReads()).toHaveLength(2);
  expect(details.open).toBe(false);
  await tap(tabButton('camera'));
  await act(async () => jest.advanceTimersByTime(1500));
  expect(diagnosticReads()).toHaveLength(2);
  expect(posts()).toHaveLength(0);
});

test('keyboard navigation focuses and selects existing views without a command', async () => {
  await render();
  await act(async () => tabButton('safety').dispatchEvent(new KeyboardEvent('keydown', {
    key: 'ArrowDown', bubbles: true, cancelable: true,
  })));
  expect(document.activeElement).toBe(tabButton('trajectory'));
  expect(tabButton('trajectory').getAttribute('aria-selected')).toBe('true');
  await act(async () => tabButton('trajectory').dispatchEvent(new KeyboardEvent('keydown', {
    key: 'End', bubbles: true, cancelable: true,
  })));
  expect(document.activeElement).toBe(tabButton('snapshot'));
  expect(host.querySelector('[data-ui="test-snapshot-view"]')).not.toBeNull();
  expect(posts()).toHaveLength(0);
});

test('system light and return controls retain their explicit handlers', async () => {
  const onToggleHeadlight = jest.fn();
  await render({ onToggleHeadlight, headlightState: true });
  expect(host.querySelector('.light-control-bar').textContent).toContain('전조등 켜짐');
  await tap(host.querySelector('.headlight-command-btn'));
  expect(onToggleHeadlight).toHaveBeenCalledTimes(1);
  expect(posts()).toHaveLength(0);
  await tap([...host.querySelectorAll('.manual-return-btn')].find(button => button.textContent === '복귀 · 자동 주차'));
  expect(posts()).toEqual([['/ui/manual_return', { method: 'POST' }]]);
});

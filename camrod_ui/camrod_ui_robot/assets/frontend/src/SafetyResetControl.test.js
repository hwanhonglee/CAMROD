import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import SafetyResetControl from './SafetyResetControl';

// HH_261024 - Exercise acknowledgement only: no test may send a motion,
// engage, manual ARM, automatic resume, or mission admission request.
let host, root, originalFetch;
const response = body => ({ ok: true, json: async () => body });
const ready = () => response({ success: true, available: true, message: '초기화 가능' });
const render = async (props = { username: 'admin', password: '1234' }) => {
  await act(async () => root.render(<SafetyResetControl {...props} />));
};
const requestButton = () => host.querySelector('[data-ui="admin-safety-reset-request"]');
const confirmButton = () => host.querySelector('[data-ui="admin-safety-reset-confirm"]');
const tap = async element => { await act(async () => element.click()); };
const posts = () => fetch.mock.calls.filter(([, options]) => options.method === 'POST');

beforeEach(() => {
  global.IS_REACT_ACT_ENVIRONMENT = true;
  originalFetch = global.fetch;
  global.fetch = jest.fn(async () => ready());
  host = document.createElement('div');
  document.body.appendChild(host);
  root = createRoot(host);
});
afterEach(() => {
  act(() => root.unmount());
  host.remove();
  global.fetch = originalFetch;
  delete global.IS_REACT_ACT_ENVIRONMENT;
});

test('reads authenticated status once without automatic reset or motion', async () => {
  await render();
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(fetch).toHaveBeenCalledWith('/api/admin/safety/reset', {
    method: 'GET', headers: { Authorization: `Basic ${btoa('admin:1234')}`, 'Content-Type': 'application/json' },
  });
  expect(requestButton().disabled).toBe(false);
  expect(host.textContent).toContain('현재 미션은 취소');
  expect(host.textContent).toContain('비상정지·장애물 감지');
  await render();
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(posts()).toHaveLength(0);
});

test('missing credentials never call the endpoint or enable reset', async () => {
  await render({ username: '', password: '' });
  expect(fetch).not.toHaveBeenCalled();
  expect(requestButton().disabled).toBe(true);
});

test('first click opens confirmation; cancellation makes no POST', async () => {
  await render();
  await tap(requestButton());
  expect(host.querySelector('[role="alertdialog"]')).not.toBeNull();
  expect(posts()).toHaveLength(0);
  const cancel = [...host.querySelectorAll('button')].find(button => button.textContent === '아니요');
  await tap(cancel);
  expect(host.querySelector('[role="alertdialog"]')).toBeNull();
  expect(posts()).toHaveLength(0);
});

test('second explicit confirmation sends strict boolean once, never starts motion', async () => {
  await render();
  await tap(requestButton());
  fetch.mockResolvedValue(response({ success: true, available: true,
    requires_explicit_start: true, message: '정지 상태로 초기화 완료' }));
  await tap(confirmButton());
  expect(posts()).toEqual([['/api/admin/safety/reset', {
    method: 'POST', headers: { Authorization: `Basic ${btoa('admin:1234')}`, 'Content-Type': 'application/json' },
    body: '{"confirm":true}',
  }]]);
  expect(host.querySelector('[role="status"]').textContent).toContain('정지 상태로 초기화 완료');
  expect(host.textContent).toContain('새 미션 시작을 별도로 요청');
  expect(host.querySelector('[role="alertdialog"]')).toBeNull();
});

test('unavailable status disables reset and explains active blockers', async () => {
  fetch.mockResolvedValue(response({ available: false, message: '비상정지 입력 유지',
    blockers: ['ESTOP_ACTIVE', { message: '로봇을 먼저 정지해주세요.' }] }));
  await render();
  expect(requestButton().disabled).toBe(true);
  expect(host.textContent).toContain('ESTOP_ACTIVE');
  expect(host.textContent).toContain('로봇을 먼저 정지');
  await tap(requestButton());
  expect(confirmButton()).toBeNull();
  expect(posts()).toHaveLength(0);
});

test('refusal remains visible and never claims successful reset', async () => {
  await render();
  await tap(requestButton());
  fetch.mockResolvedValue({ ok: false, json: async () => ({ success: false, available: false,
    message: '장애물이 아직 남아 있습니다.', blockers: ['OBSTACLE_ACTIVE'] }) });
  await tap(confirmButton());
  expect(host.querySelector('[role="alert"]').textContent).toBe('장애물이 아직 남아 있습니다.');
  expect(host.textContent).toContain('OBSTACLE_ACTIVE');
  expect(requestButton().disabled).toBe(true);
  expect(posts()).toHaveLength(1);
});

test('pending request suppresses double clicks and another status request', async () => {
  await render();
  await tap(requestButton());
  let finish;
  fetch.mockImplementation(() => new Promise(resolve => { finish = resolve; }));
  const confirm = confirmButton();
  await act(async () => { confirm.click(); confirm.click(); });
  expect(posts()).toHaveLength(1);
  expect(requestButton().disabled).toBe(true);
  expect(host.querySelector('.safety-reset-refresh').disabled).toBe(true);
  expect(host.querySelector('[role="status"]').textContent).toContain('요청 중');
  await act(async () => finish(response({ success: true, available: true, message: '초기화 완료' })));
  expect(requestButton().disabled).toBe(false);
});

test('explicit status refresh can re-enable service after network failure', async () => {
  fetch.mockRejectedValue(new Error('연결 끊김'));
  await render();
  expect(host.querySelector('[role="alert"]').textContent).toBe('연결 끊김');
  expect(requestButton().disabled).toBe(true);
  fetch.mockResolvedValue(ready());
  await tap(host.querySelector('.safety-reset-refresh'));
  expect(requestButton().disabled).toBe(false);
  expect(posts()).toHaveLength(0);
});

test('late reset response cannot affect a replacement administrator session', async () => {
  await render();
  await tap(requestButton());
  let finish;
  fetch.mockImplementationOnce(() => new Promise(resolve => { finish = resolve; }));
  await tap(confirmButton());
  fetch.mockResolvedValue(response({ available: false, message: '새 세션은 정지 상태 확인 필요' }));
  await render({ username: 'admin', password: 'new-password' });
  await act(async () => finish(response({ success: true, available: true, message: '이전 세션 응답' })));
  expect(host.textContent).not.toContain('이전 세션 응답');
  expect(host.textContent).toContain('새 세션은 정지 상태 확인 필요');
  expect(requestButton().disabled).toBe(true);
});

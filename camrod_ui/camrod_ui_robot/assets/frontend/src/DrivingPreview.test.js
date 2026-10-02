import React, { act } from 'react';
import { createRoot } from 'react-dom/client';
import DrivingPreview, { makeDrivingPreviewSnapshot, demoRoutePose } from './DrivingPreview';

// HH_261001 - Preview only exercises presentation; no ROS backend can be involved.
jest.mock('./DrivingDisplay', () => function MockDisplay({ onDismiss, demo, snapshot }) {
  return <button data-testid="display" onClick={onDismiss} data-demo={String(demo)}>
    {snapshot.mission.site} {snapshot.mission.intent}
  </button>;
});

describe('local-only driving preview', () => {
  let host;
  let root;
  let request;
  beforeEach(() => {
    global.IS_REACT_ACT_ENVIRONMENT = true;
    jest.useFakeTimers();
    host = document.createElement('div');
    document.body.appendChild(host);
    root = createRoot(host);
    request = global.fetch;
    global.fetch = jest.fn(() => { throw new Error('Preview must not fetch'); });
  });
  afterEach(() => {
    act(() => root.unmount());
    host.remove();
    global.fetch = request;
    jest.useRealTimers();
  });
  test('always labels synthetic fixtures and never sends network requests', () => {
    act(() => root.render(<DrivingPreview />));
    expect(host.textContent).toContain('시연 상태·테마 선택용 개발 패널');
    expect(host.querySelector('[data-testid=display]').dataset.demo).toBe('true');
    act(() => jest.advanceTimersByTime(1500));
    expect(global.fetch).not.toHaveBeenCalled();
  });
  test('touch restores the actual site screen and can reopen preview', () => {
    act(() => root.render(<DrivingPreview />));
    act(() => host.querySelector('[data-testid=display]').click());
    expect(host.querySelector('[data-preview=returned]')).not.toBeNull();
    expect(host.textContent).toContain('배송 목적지 선택');
    expect(host.querySelector('[data-ui=operator-site-B7]')).not.toBeNull();
    act(() => host.querySelector('[data-ui=open-driving-display]').click());
    expect(host.querySelector('[data-testid=display]').textContent).toContain('B7');
    expect(global.fetch).not.toHaveBeenCalled();
  });
  test('the meter-scale trajectory actually turns heading, position and path together', () => {
    const start = makeDrivingPreviewSnapshot('delivery', 0);
    const bend = makeDrivingPreviewSnapshot('delivery', 10);
    const after = makeDrivingPreviewSnapshot('delivery', 20);
    expect(start.pose.yaw).toBe(0);
    expect(bend.pose.yaw).toBeGreaterThan(0.6);
    expect(after.pose.yaw).toBeCloseTo(Math.PI / 2);
    expect(after.pose.y).toBeGreaterThan(bend.pose.y);
    expect(bend.motion.vx_mps).toBe(0.92);
    expect(bend.motion.yaw_rate_rps).toBeGreaterThan(0);
    expect(demoRoutePose(8)).toEqual({x: 8, y: 0, yaw: 0});
  });
  test('the entire fixture boxes stay outside the robot corridor along every route bend', () => {
    // HH_261002 - This passive preview cannot brake; do not visually drive through fixtures.
    const snapshot = makeDrivingPreviewSnapshot('delivery', 0);
    const boxes = snapshot.perception.objects.filter(object => object.dimensions);
    for (let distance = 0; distance <= 35; distance += 0.02) {
      const pose = demoRoutePose(distance);
      for (const object of boxes) {
        const dx = Math.max(0, Math.abs(pose.x - object.bbox.center.x) - object.dimensions.x / 2);
        const dy = Math.max(0, Math.abs(pose.y - object.bbox.center.y) - object.dimensions.y / 2);
        expect(Math.hypot(dx, dy)).toBeGreaterThan(1.2);
      }
    }
  });
  test('stop retains the current return location and disables roll motion', () => {
    const moving = makeDrivingPreviewSnapshot('return', 10);
    const stopped = makeDrivingPreviewSnapshot('stop', 10, 'return');
    expect(stopped.pose).toEqual(moving.pose);
    expect(stopped.motion.speed_mps).toBe(0);
    expect(stopped.motion.yaw_rate_rps).toBe(0);
    expect(stopped.mission.service_state_name).toBe('RETURNING_TO_DROP_ZONE');
  });
  test('the route ends at zero speed and zero ETA instead of silently teleporting', () => {
    const arrived = makeDrivingPreviewSnapshot('delivery', 100);
    expect(arrived.motion.speed_mps).toBe(0);
    expect(arrived.progress.remaining_distance_m).toBe(0);
    expect(arrived.progress.remaining_time_s).toBe(0);
    expect(arrived.progress.completion_pct).toBe(100);
    expect(arrived.mission.phase).toBe('ARRIVED');
    expect(makeDrivingPreviewSnapshot('delivery', 200).pose).toEqual(arrived.pose);
    expect(arrived.sensors.gnss.label).toBe('시연 입력');
  });
  test.each(['delivery', 'recall', 'return', 'stop', 'offline'])('%s is an explicitly deterministic fixture', mode => {
    const snapshot = makeDrivingPreviewSnapshot(mode, 2);
    expect(snapshot).toEqual(makeDrivingPreviewSnapshot(mode, 2));
    expect(snapshot.mission.generation).toBe(42);
    if (mode === 'offline') expect(snapshot.connected).toBe(false);
    if (mode === 'stop') expect(snapshot.progress.remaining_time_s).toBeNull();
  });
});

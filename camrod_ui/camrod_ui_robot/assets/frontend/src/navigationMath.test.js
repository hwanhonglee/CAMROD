// HH_261001 - Verify metric coordinates, heading interpolation, and freshness-gated animation.
import { advanceWheelRoll, buildRouteRibbon, dampHeading, followCamera, interpolateAngle,
  interpolatePose, mapToThree, motionMayAnimate, poseIsFresh, signedTravelDistance } from './navigationMath';

const data = () => ({ connected: true, pose: { x: 0, y: 0, yaw: 0, age_s: 0.1 }, mission: { phase: 'DRIVING', state: 'MOVING_TO_SITE' } });

test('metres stay metres and ROS left/up map to glTF negativeZ/positiveY', () => {
  expect(mapToThree([101.553, 201.12, 1.324], { x: 100, y: 200 })).toEqual({ x: 1.5529999999999973, y: 1.324, z: -1.1200000000000045 });
  expect(mapToThree([0, 2, null])).toEqual({ x: 0, y: 0, z: -2 });
});
test('heading interpolation crosses ±π by the shortest turn', () => {
  expect(Math.abs(interpolateAngle(Math.PI - 0.1, -Math.PI + 0.1, 0.5))).toBeCloseTo(Math.PI);
  expect(interpolatePose({ x: 0, y: 2, yaw: 0 }, { x: 10, y: 6, yaw: Math.PI / 2 }, 0.5)).toMatchObject({ x: 5, y: 4, yaw: Math.PI / 4 });
});
test('pose interpolation never extrapolates outside received endpoints', () => {
  const from = { x: 1, y: 2, yaw: 0 }, to = { x: 4, y: 5, yaw: 1 };
  expect(interpolatePose(from, to, 4)).toMatchObject(to);
  expect(interpolatePose(from, to, -2)).toMatchObject(from);
});
test('stale, disconnected and explicit stopped states stop animation', () => {
  expect(poseIsFresh(data(), 901)).toBe(false);
  expect(poseIsFresh(data(), 900)).toBe(true);
  for (const age_s of [undefined, NaN, -0.1, Infinity]) {
    expect(poseIsFresh({ ...data(), pose: { ...data().pose, age_s } })).toBe(false);
  }
  expect(motionMayAnimate({ ...data(), connected: false })).toBe(false);
  expect(motionMayAnimate({ ...data(), mission: { phase: 'SAFETY_STOP' } })).toBe(false);
  expect(motionMayAnimate({ ...data(), mission: { state: 'GUEST_LOADING_WAIT' } })).toBe(false);
  expect(motionMayAnimate(data(), 400)).toBe(true);
});
test('estimated wheel rotation uses signed confirmed travel and freezes with zero/missing speed', () => {
  const from = { x: 0, y: 0 }, to = { x: 0.3, y: 0.4 };
  expect(signedTravelDistance(from, to, 0.5)).toBeCloseTo(0.5);
  expect(signedTravelDistance(from, to, -0.5)).toBeCloseTo(-0.5);
  expect(signedTravelDistance(from, to, 0)).toBe(0);
  expect(signedTravelDistance(from, to, null)).toBe(0);
  expect(signedTravelDistance(from, { x: 20, y: 0 }, 1)).toBe(0);
  expect(advanceWheelRoll(0, 0.153)).toBeCloseTo(-1);
  expect(advanceWheelRoll(0, -0.153)).toBeCloseTo(1);
  expect(advanceWheelRoll(1, 0.2, 0.153, false)).toBe(1);
});
test('camera follows actual heading with a rear offset and positive lookahead', () => {
  const view = followCamera({ x: 4, y: 7 }, 0, { x: 4, y: 7 });
  expect(view.position.x).toBeLessThan(0);
  expect(view.target.x).toBeGreaterThan(0);
  expect(view.position.y).toBeGreaterThan(0);
  expect(dampHeading(0, Math.PI / 2, 0.016)).toBeGreaterThan(0);
  expect(dampHeading(0, Math.PI / 2, 0.016)).toBeLessThan(Math.PI / 2);
  const detail = followCamera({ x: 4, y: 7 }, 0, { x: 4, y: 7 }, 1);
  expect(Math.abs(detail.position.x)).toBeLessThan(Math.abs(view.position.x));
  expect(detail.target.x).toBe(0);
  expect(detail.position.z).toBeGreaterThan(2.5);
});
test('route corridor retains physically meaningful width and never bridges large gaps', () => {
  const vertices = buildRouteRibbon([[0, 0], [2, 0]], { x: 0, y: 0 }, 0.2);
  expect(vertices).toHaveLength(18);
  expect(Math.abs(vertices[2])).toBeCloseTo(0.1);
  expect(buildRouteRibbon([[0, 0], [20, 0]], { x: 0, y: 0 })).toEqual([]);
});

test('detail angle cycles all four faces by orbiting only the camera', () => {
  const pose = { x: 10, y: 20, yaw: 0 };
  const positions = [0, 1, 2, 3].map((quarter) => followCamera(pose, 0, { x: 10, y: 20 }, 1, quarter * Math.PI / 2));
  expect(positions[0].position.x).toBeLessThan(0);
  expect(positions[0].position.z).toBeGreaterThan(0);
  expect(positions[1].position.x).toBeGreaterThan(0);
  expect(positions[1].position.z).toBeGreaterThan(0);
  expect(positions[2].position.x).toBeGreaterThan(0);
  expect(positions[2].position.z).toBeLessThan(0);
  expect(positions[3].position.x).toBeLessThan(0);
  expect(positions[3].position.z).toBeLessThan(0);
  positions.forEach((view) => {
    expect(view.target.x).toBeCloseTo(0);
    expect(view.target.y).toBeCloseTo(0.62);
    expect(view.target.z).toBeCloseTo(0);
  });
  expect(pose).toEqual({ x: 10, y: 20, yaw: 0 });
  expect(followCamera(pose, 0, { x: 10, y: 20 }, 0, Math.PI)).toEqual(followCamera(pose, 0, { x: 10, y: 20 }));
});

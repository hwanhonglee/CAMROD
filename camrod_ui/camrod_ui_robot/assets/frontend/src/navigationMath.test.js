// HH_261001 - Verify metric coordinates, heading interpolation, and freshness-gated animation.
import { advanceWheelRoll, bodyPoseDelta, buildRouteRibbon, dampHeading, estimateWheelStep, followCamera, interpolateAngle,
  interpolatePose, mapToThree, motionMayAnimate, navigationCameraHeading, navigationMotionKind, poseIsFresh, signedTravelDistance } from './navigationMath';

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
test('stale/disconnected data freeze motion, but service labels never hide a fresh measured pose', () => {
  expect(poseIsFresh(data(), 901)).toBe(false);
  expect(poseIsFresh(data(), 900)).toBe(true);
  for (const age_s of [undefined, NaN, -0.1, Infinity]) {
    expect(poseIsFresh({ ...data(), pose: { ...data().pose, age_s } })).toBe(false);
  }
  expect(motionMayAnimate({ ...data(), connected: false })).toBe(false);
  // HH_261002 - Manual motion/braking can coexist with a stopped mission label.
  expect(motionMayAnimate({ ...data(), mission: { phase: 'SAFETY_STOP' } })).toBe(true);
  expect(motionMayAnimate({ ...data(), mission: { state: 'GUEST_LOADING_WAIT' } })).toBe(true);
  expect(motionMayAnimate(data(), 400)).toBe(true);
});

// HH_261002 - Regression cases reproduce crab (zero forward speed), zero-turn
// (zero translation) and reverse without requiring a controller or CAN fixture.
const wheelPositions = [
  { x: 0.44, y: 0.27 }, { x: 0.44, y: -0.27 },
  { x: -0.44, y: 0.27 }, { x: -0.44, y: -0.27 },
];
const pose = (x = 0, y = 0, yaw = 0) => ({ x, y, yaw, frame_id: 'map' });
test.each([1, -1])('zero-turn %i rolls opposite sides and steers front/rear oppositely', (sign) => {
  const step = bodyPoseDelta(pose(), pose(0, 0, sign * 0.1));
  expect(navigationMotionKind(step)).toBe('zero-turn');
  const wheels = wheelPositions.map(position => estimateWheelStep(step, position));
  expect(wheels.map(w => Math.sign(w.steer))).toEqual([-1, 1, 1, -1]);
  expect(wheels.map(w => Math.sign(w.distance))).toEqual([-sign, sign, -sign, sign]);
  wheels.forEach(w => expect(Math.abs(w.distance)).toBeCloseTo(Math.hypot(0.44, 0.27) * 0.1));
});
test.each([1, -1])('crab %i preserves body yaw and rolls all four lateral wheels', (sign) => {
  const step = bodyPoseDelta(pose(), pose(0, sign * 0.1, 0));
  expect(navigationMotionKind(step)).toBe('crab');
  wheelPositions.forEach(position => {
    const wheel = estimateWheelStep(step, position);
    expect(Math.abs(wheel.steer)).toBeCloseTo(Math.PI / 2);
    expect(wheel.distance * Math.sin(wheel.steer)).toBeCloseTo(sign * 0.1);
    expect(wheel.distance * Math.cos(wheel.steer)).toBeCloseTo(0);
  });
});
test('diagonal crab uses the actual body direction, not map axes or a fixed 90-degree steer', () => {
  const step = bodyPoseDelta(pose(0, 0, Math.PI / 2), pose(-0.1, 0.1, Math.PI / 2));
  expect(navigationMotionKind(step)).toBe('crab');
  expect(estimateWheelStep(step, wheelPositions[0]).steer).toBeCloseTo(Math.PI / 4);
  expect(navigationMotionKind(bodyPoseDelta(pose(0, 0, Math.PI / 2), pose(0, 0.1, Math.PI / 2)))).toBe('forward');
});
test('small longitudinal pose noise cannot flip a lateral wheel by 180 degrees', () => {
  let previous = 0;
  for (const x of [0.001, -0.001, 0, -0.002, 0.002]) {
    const wheel = estimateWheelStep({ x, y: 0.1, yaw: 0 }, wheelPositions[0], previous);
    expect(wheel.steer).toBeCloseTo(Math.PI / 2);
    expect(wheel.distance).toBeGreaterThan(0);
    previous = wheel.steer;
  }
});
test('reverse rolls backwards without turning every wheel around', () => {
  const step = bodyPoseDelta(pose(), pose(-0.1));
  expect(navigationMotionKind(step)).toBe('reverse');
  wheelPositions.forEach(position => {
    const wheel = estimateWheelStep(step, position);
    expect(wheel.steer).toBeCloseTo(0);
    expect(wheel.distance).toBeCloseTo(-0.1);
  });
});
test('curved body motion preserves different inner/outer wheel travel and shortest yaw wrap', () => {
  const angle = 0.1, radius = 2;
  const step = bodyPoseDelta(pose(), pose(radius * Math.sin(angle), radius * (1 - Math.cos(angle)), angle));
  expect(step.x).toBeCloseTo(radius * angle);
  expect(step.y).toBeCloseTo(0);
  expect(estimateWheelStep(step, wheelPositions[0]).distance).toBeLessThan(estimateWheelStep(step, wheelPositions[1]).distance);
  expect(bodyPoseDelta(pose(0, 0, Math.PI - 0.02), pose(0, 0, -Math.PI + 0.02)).yaw).toBeCloseTo(0.04);
});
test('still, invalid, stale-gap/frame-reset increments preserve the last wheel pose', () => {
  expect(bodyPoseDelta(pose(), pose(3))).toBeNull();
  expect(bodyPoseDelta(pose(), { ...pose(), frame_id: 'odom' })).toBeNull();
  expect(bodyPoseDelta(pose(), pose(NaN))).toBeNull();
  expect(navigationMotionKind(null)).toBe('stationary');
  for (const step of [null, bodyPoseDelta(pose(), pose())]) {
    expect(estimateWheelStep(step, wheelPositions[0], 0.75)).toEqual({ steer: 0.75, distance: 0 });
  }
  expect(estimateWheelStep({ x: 0, y: -0.1, yaw: 0 }, wheelPositions[0], -1).steer).toBeCloseTo(-Math.PI / 2);
});
test('camera exposes zero-turn/crab rotation and resumes rear-follow on forward/reverse movement', () => {
  for (const kind of ['zero-turn', 'crab', 'stationary']) {
    expect(navigationCameraHeading(0.3, 1.2, 0.05, kind)).toBe(0.3);
  }
  for (const kind of ['forward', 'reverse']) {
    expect(navigationCameraHeading(0.3, 1.2, 0.05, kind)).toBeGreaterThan(0.3);
    expect(navigationCameraHeading(0.3, 1.2, 0.05, kind)).toBeLessThan(1.2);
  }
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

// HH_261001 - Physical navigation coordinates: ROS map (x forward, y left, z up) metres
// map to Three/glTF (x forward, y up, z right). No visual vehicle scaling.
export const RANGER_WHEEL_RADIUS_M = 0.153;
export const NAVIGATION_STALE_MS = 1000;
export const clamp = (value, minimum, maximum) => Math.max(minimum, Math.min(maximum, value));
export const finiteNumber = (value) => typeof value === 'number' && Number.isFinite(value);

export function wrapAngle(angle) {
  if (!finiteNumber(angle)) return 0;
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

export function interpolateAngle(from, to, fraction) {
  return wrapAngle(from + wrapAngle(to - from) * clamp(fraction, 0, 1));
}

export function interpolatePose(from, to, fraction) {
  if (!from) return to ? { ...to } : null;
  if (!to) return { ...from };
  const t = clamp(fraction, 0, 1);
  return { x: from.x + (to.x - from.x) * t, y: from.y + (to.y - from.y) * t,
    yaw: interpolateAngle(from.yaw, to.yaw, t), frame_id: to.frame_id };
}

export function mapToThree(point, origin = { x: 0, y: 0 }, groundHeight = 0) {
  return { x: point[0] - origin.x, y: finiteNumber(point[2]) ? point[2] : groundHeight,
    z: -(point[1] - origin.y) };
}

export function poseIsFresh(data, elapsedMs = 0) {
  const pose = data?.pose;
  if (!finiteNumber(pose?.age_s) || pose.age_s < 0) return false;
  const age = pose.age_s * 1000;
  return data?.connected === true && pose && finiteNumber(pose.x) && finiteNumber(pose.y)
    && finiteNumber(pose.yaw) && elapsedMs >= 0 && elapsedMs + age <= NAVIGATION_STALE_MS;
}

export function motionMayAnimate(data, elapsedMs = 0) {
  // HH_261002 - A mission/stop label is not a motion measurement. Display fresh
  // received movement even during manual control or braking; never extrapolate.
  return Boolean(poseIsFresh(data, elapsedMs));
}

// HH_261002 - Recover a body-frame planar displacement from two measured poses.
// The midpoint/chord correction handles curved motion without confusing map-Y
// travel with crab. Reject frame changes/teleports instead of spinning the wheels.
export function bodyPoseDelta(from, to) {
  if (!from || !to || ![from.x, from.y, from.yaw, to.x, to.y, to.yaw].every(finiteNumber)
    || from.frame_id !== to.frame_id) return null;
  const dx = to.x - from.x, dy = to.y - from.y;
  if (Math.hypot(dx, dy) > 2) return null;
  const yaw = wrapAngle(to.yaw - from.yaw), midYaw = from.yaw + yaw / 2;
  const correction = Math.abs(yaw) < 1e-6 ? 1 : yaw / (2 * Math.sin(yaw / 2));
  return { x: (dx * Math.cos(midYaw) + dy * Math.sin(midYaw)) * correction,
    y: (-dx * Math.sin(midYaw) + dy * Math.cos(midYaw)) * correction, yaw };
}

// HH_261002 - These labels only select a display camera, never a control mode.
export function navigationMotionKind(step) {
  if (!step || Math.hypot(step.x, step.y, step.yaw) < 1e-6) return 'stationary';
  if (Math.hypot(step.x, step.y) < Math.abs(step.yaw) * 0.2) return 'zero-turn';
  if (Math.abs(step.y) > Math.abs(step.x) * 0.5) return 'crab';
  return step.x < 0 ? 'reverse' : 'forward';
}

// HH_261002 - Rigid-body point velocity gives each wheel its own rolling path:
// (dx - dYaw*y, dy + dYaw*x). Positions come from the metric GLB steer pivots.
// This is a no-slip visualization estimate, NOT measured CAN steering or RPM.
export function estimateWheelStep(step, position, previousSteer = 0) {
  if (!step || !position || ![step.x, step.y, step.yaw, position.x, position.y].every(finiteNumber)) {
    return { steer: previousSteer, distance: 0 };
  }
  const x = step.x - step.yaw * position.y, y = step.y + step.yaw * position.x;
  const travel = Math.hypot(x, y);
  if (travel < 1e-6) return { steer: previousSteer, distance: 0 };
  let steer = Math.atan2(y, x), distance = travel;
  // Equivalent +/-90 degree wheel poses allow reverse travel without turning
  // every wheel 180 degrees. A display-only lateral deadband (~2.9 degrees)
  // prevents tiny longitudinal pose noise from flipping crab steering by 180.
  if (Math.abs(x) < travel * 0.05) {
    steer = previousSteer < 0 ? -Math.PI / 2 : Math.PI / 2;
    distance = y * Math.sign(steer);
  } else if (steer > Math.PI / 2) { steer -= Math.PI; distance = -travel; }
  else if (steer < -Math.PI / 2) { steer += Math.PI; distance = -travel; }
  return { steer, distance };
}

// HH_261002 - Keep an orientation reference during zero-turn/crab so the body
// can visibly rotate/translate sideways. Resume rear-follow on longitudinal travel.
export function navigationCameraHeading(current, target, deltaSeconds, kind) {
  return ['forward', 'reverse'].includes(kind) ? dampHeading(current, target, deltaSeconds, 2.5) : current;
}

// HH_261001 - Wheel rotation is a display estimate from confirmed displacement, not a CAN
// angle measurement. Zero/missing speed, discontinuities and stale data freeze it.
export function signedTravelDistance(from, to, signedSpeed) {
  if (!from || !to || !finiteNumber(signedSpeed) || Math.abs(signedSpeed) < 0.005) return 0;
  const distance = Math.hypot(to.x - from.x, to.y - from.y);
  if (!finiteNumber(distance) || distance > 2) return 0;
  return distance * Math.sign(signedSpeed);
}

export function advanceWheelRoll(angle, distance, radius = RANGER_WHEEL_RADIUS_M, valid = true) {
  if (!valid || !finiteNumber(distance) || !finiteNumber(radius) || radius <= 0) return angle;
  return wrapAngle(angle - distance / radius);
}

export function dampHeading(current, target, deltaSeconds, damping = 2.8) {
  const fraction = 1 - Math.exp(-Math.max(0, damping) * clamp(deltaSeconds, 0, 0.1));
  return interpolateAngle(current, target, fraction);
}

export function followCamera(pose, heading, origin = { x: 0, y: 0 }, detail = 0, detailAngle = 0) {
  const position = mapToThree([pose.x, pose.y, 0], origin);
  const cos = Math.cos(heading), sin = Math.sin(heading);
  const blend = clamp(Number(detail) || 0, 0, 1);
  const rear = 5 - blend * 2.9, side = 0.3 + blend * 2.5;
  const height = 3.6 - blend * 1.5, lookahead = 4 * (1 - blend);
  const orbit = heading + (finiteNumber(detailAngle) ? detailAngle : 0) * blend;
  const orbitCos = Math.cos(orbit), orbitSin = Math.sin(orbit);
  return {
    position: { x: position.x - orbitCos * rear + orbitSin * side, y: height, z: position.z + orbitSin * rear + orbitCos * side },
    target: { x: position.x + cos * lookahead, y: 0.28 + blend * 0.34, z: position.z - sin * lookahead },
  };
}

export function buildRouteRibbon(points, origin, width = 0.12, height = 0.025) {
  const vertices = [];
  for (let index = 1; index < points.length; index += 1) {
    const a = mapToThree(points[index - 1], origin, height);
    const b = mapToThree(points[index], origin, height);
    const dx = b.x - a.x, dz = b.z - a.z;
    const length = Math.hypot(dx, dz);
    if (!length || length > 15) continue;
    const nx = -dz / length * width / 2, nz = dx / length * width / 2;
    vertices.push(a.x + nx, height, a.z + nz, a.x - nx, height, a.z - nz, b.x + nx, height, b.z + nz,
      a.x - nx, height, a.z - nz, b.x - nx, height, b.z - nz, b.x + nx, height, b.z + nz);
  }
  return vertices;
}

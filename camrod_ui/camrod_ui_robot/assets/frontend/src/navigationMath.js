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
  if (!poseIsFresh(data, elapsedMs)) return false;
  if (['SAFETY_STOP', 'STOPPED'].includes(data.mission?.phase)) return false;
  if (['OPERATOR_STOPPED', 'GUEST_LOADING_WAIT', 'UNLOAD_WAIT', 'CHARGING', 'DROP_ZONE_WAIT',
    'WAITING_FOR_RETURN_REQUEST', 'WAITING_FOR_CHARGING', 'SITE_ARRIVED'].includes(data.mission?.state)) return false;
  return true;
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

import React, { useEffect, useMemo, useRef, useState } from 'react';

// HH_260810 - These tabs consolidate the former GNSS, radar, velocity, path,
// and RViz operator viewers without changing the underlying ROS authorities.
export const TELEMETRY_TABS = [
  { id: 'gnss', label: 'GNSS · IMU' },
  { id: 'proximity', label: '레이더 · LiDAR' },
  { id: 'camera', label: '카메라' },
  { id: 'trajectory', label: '주행 궤적' },
  { id: 'perception', label: '지도 · 인지' },
  { id: 'safety', label: '안전 · 제어' },
  { id: 'docking', label: '도킹 · 주차' },
];

const EMPTY_TELEMETRY = {
  stream_rate_hz: 10,
  session_active: false,
  sources: {},
  gnss: { fix: {}, navpvt: {}, covariance: {}, relative_heading: {} },
  imu: {},
  radar: { channels: {} },
  lidar: { points: [], point_count: 0, sample_count: 0, streams: {} },
  cameras: { front: {}, rear: {}, docking: {} },
  localization: { pose: {}, status: {}, trace: [] },
  motion: { velocity: {}, tracking_error: {} },
  paths: { global: {}, local: {}, maneuvers: {} },
  footprint: { points: [] },
  perception: {
    obstacle_cloud: {}, obstacle_boxes: {},
    cost_layers: { lanelet: {}, lidar: {}, radar: {}, inflation: {} },
  },
  safety: { gate: {}, controllers: {}, radar_evidence: '', obstacle_replan: '' },
  docking: { tag_detected: false, tag: {}, is_charging: false, battery_percentage: null },
  mission: {},
};

const RADAR_ORDER = ['front1', 'front2', 'left1', 'left2', 'right1', 'right2', 'rear'];
const RADAR_LABEL = {
  front1: 'Front 1', front2: 'Front 2', left1: 'Left 1', left2: 'Left 2',
  right1: 'Right 1', right2: 'Right 2', rear: 'Rear',
};
const RADAR_MOUNTS = {
  front1: [0.62787, -0.11005, 0],
  front2: [0.62787, 0.11005, 0],
  left1: [0.38, 0.53, Math.PI / 2],
  left2: [-0.38, 0.53, Math.PI / 2],
  right1: [0.38, -0.53, -Math.PI / 2],
  right2: [-0.38, -0.53, -Math.PI / 2],
  rear: [-0.61733, 0, Math.PI],
};

const finite = value => typeof value === 'number' && Number.isFinite(value);

function numberText(value, digits = 2, fallback = '-') {
  return finite(value) ? value.toFixed(digits) : fallback;
}

function wrapDegrees(value) {
  if (!finite(value)) return null;
  return ((value + 180) % 360 + 360) % 360 - 180;
}

function sourceState(telemetry, name, staleAfter = 2.5) {
  const source = telemetry.sources?.[name];
  if (!source) return { state: 'waiting', label: '대기', age: null, rate: null };
  const stale = !finite(source.age_s) || source.age_s > staleAfter;
  return {
    state: stale ? 'stale' : 'live',
    label: stale ? 'STALE' : 'LIVE',
    age: source.age_s,
    rate: source.rate_hz,
  };
}

function SourcePill({ telemetry, source, label, staleAfter = 2.5 }) {
  const status = sourceState(telemetry, source, staleAfter);
  return (
    <div className={`telemetry-source telemetry-source-${status.state}`}>
      <span className="telemetry-source-dot" />
      <strong>{label}</strong>
      <span>{status.label}</span>
      {finite(status.rate) && <span>{status.rate.toFixed(1)} Hz</span>}
      {finite(status.age) && <span>{status.age.toFixed(1)} s</span>}
    </div>
  );
}

function Metric({ label, value, unit = '', tone = '' }) {
  return (
    <div className={`telemetry-metric ${tone ? `telemetry-metric-${tone}` : ''}`}>
      <span className="telemetry-metric-label">{label}</span>
      <span className="telemetry-metric-value">
        {value ?? '-'}{unit && <small>{unit}</small>}
      </span>
    </div>
  );
}

function SectionHeader({ title, meta }) {
  return (
    <div className="telemetry-section-header">
      <h3>{title}</h3>
      {meta && <span>{meta}</span>}
    </div>
  );
}

function HeadingCompass({ gnss, imu, localization }) {
  const rays = [
    { key: 'gnss', label: 'GNSS', value: gnss, color: '#f1b84b' },
    { key: 'imu', label: 'IMU', value: imu, color: '#58c4dd' },
    { key: 'pose', label: 'Pose', value: localization, color: '#72d17d' },
  ];
  const endpoint = (degrees, radius) => {
    const rad = degrees * Math.PI / 180;
    return [130 + Math.sin(rad) * radius, 130 - Math.cos(rad) * radius];
  };
  return (
    <div className="heading-plot">
      <svg viewBox="0 0 260 260" role="img" aria-label="GNSS IMU localization heading comparison">
        <circle cx="130" cy="130" r="103" className="heading-ring" />
        <circle cx="130" cy="130" r="68" className="heading-ring-inner" />
        <line x1="130" y1="18" x2="130" y2="242" className="heading-axis" />
        <line x1="18" y1="130" x2="242" y2="130" className="heading-axis" />
        <text x="130" y="13" textAnchor="middle">0°</text>
        <text x="247" y="134" textAnchor="end">90°</text>
        <text x="130" y="256" textAnchor="middle">180°</text>
        {rays.filter(ray => finite(ray.value)).map((ray, index) => {
          const [x, y] = endpoint(ray.value, 93 - index * 8);
          return (
            <g key={ray.key}>
              <line x1="130" y1="130" x2={x} y2={y} style={{ stroke: ray.color }} className="heading-ray" />
              <circle cx={x} cy={y} r="5" style={{ fill: ray.color }} />
            </g>
          );
        })}
        <circle cx="130" cy="130" r="7" className="heading-center" />
      </svg>
      <div className="heading-legend">
        {rays.map(ray => (
          <div key={ray.key}>
            <span style={{ background: ray.color }} />
            <strong>{ray.label}</strong>
            <em>{finite(ray.value) ? `${wrapDegrees(ray.value).toFixed(1)}°` : '-'}</em>
          </div>
        ))}
      </div>
    </div>
  );
}

function GnssImuView({ telemetry }) {
  const nav = telemetry.gnss?.navpvt || {};
  const fix = telemetry.gnss?.fix || {};
  const rel = telemetry.gnss?.relative_heading || {};
  const cov = telemetry.gnss?.covariance || {};
  const imu = telemetry.imu || {};
  const pose = telemetry.localization?.pose || {};
  const localization = telemetry.localization?.status || {};
  const gnssHeading = rel.heading_valid ? rel.heading_deg : nav.vehicle_heading_deg;
  const headingDelta = finite(gnssHeading) && finite(imu.yaw_deg)
    ? Math.abs(wrapDegrees(gnssHeading - imu.yaw_deg)) : null;
  const latitude = finite(nav.latitude) ? nav.latitude : fix.latitude;
  const longitude = finite(nav.longitude) ? nav.longitude : fix.longitude;
  const fixAccuracyAvailable = finite(fix.east_std_m) || finite(fix.north_std_m);
  const hAccuracy = finite(nav.horizontal_accuracy_m)
    ? nav.horizontal_accuracy_m
    : (fixAccuracyAvailable ? Math.hypot(fix.east_std_m || 0, fix.north_std_m || 0) : null);

  return (
    <div className="telemetry-view telemetry-gnss-view">
      <div className="telemetry-source-row">
        <SourcePill telemetry={telemetry} source="gnss.navpvt" label="NAV-PVT" staleAfter={3} />
        <SourcePill telemetry={telemetry} source="gnss.fix" label="NavSatFix" staleAfter={3} />
        <SourcePill telemetry={telemetry} source="gnss.relpos" label="Moving base" staleAfter={3} />
        <SourcePill telemetry={telemetry} source="imu" label="IMU" />
        <SourcePill telemetry={telemetry} source="localization.pose" label="Localization" />
      </div>
      <div className="telemetry-two-column telemetry-two-column-heading">
        <section className="telemetry-section">
          <SectionHeader title="GNSS solution" meta={nav.carrier_solution || (fix.status >= 0 ? 'FIX' : 'NO FIX')} />
          <div className="telemetry-metric-grid telemetry-metric-grid-4">
            <Metric label="Fix" value={nav.fix_name || (fix.status >= 0 ? 'STANDARD' : 'NO FIX')} tone={nav.gnss_fix_ok || fix.status >= 0 ? 'ok' : 'warn'} />
            <Metric label="Carrier" value={nav.carrier_solution || rel.carrier_solution || '-'} tone={nav.carrier_solution === 'FIXED' || rel.carrier_solution === 'FIXED' ? 'ok' : 'warn'} />
            <Metric label="Satellites" value={nav.satellites ?? '-'} />
            <Metric label="H accuracy" value={numberText(hAccuracy, 3)} unit="m" />
            <Metric label="Latitude" value={numberText(latitude, 7)} />
            <Metric label="Longitude" value={numberText(longitude, 7)} />
            <Metric label="Altitude" value={numberText(fix.altitude_m ?? nav.height_m, 2)} unit="m" />
            <Metric label="Ground speed" value={numberText(nav.ground_speed_mps, 2)} unit="m/s" />
            <Metric label="Position DOP" value={numberText(nav.position_dop, 2)} />
            <Metric label="Baseline" value={numberText(rel.baseline_m, 3)} unit="m" />
            <Metric label="Heading accuracy" value={numberText(rel.heading_accuracy_deg ?? nav.heading_accuracy_deg, 2)} unit="°" />
            <Metric label="GNSS / IMU delta" value={numberText(headingDelta, 1)} unit="°" tone={finite(headingDelta) && headingDelta > 15 ? 'warn' : ''} />
          </div>
          <div className="telemetry-subsection">
            <span>Position covariance</span>
            <div className="telemetry-inline-values">
              <b>N {numberText(cov.north_std_m ?? fix.north_std_m, 3)} m</b>
              <b>E {numberText(cov.east_std_m ?? fix.east_std_m, 3)} m</b>
              <b>D/U {numberText(cov.down_std_m ?? fix.up_std_m, 3)} m</b>
            </div>
          </div>
        </section>
        <section className="telemetry-section telemetry-heading-section">
          <SectionHeader title="Direction consistency" meta={localization.mode_label || 'mode unavailable'} />
          <HeadingCompass gnss={gnssHeading} imu={imu.yaw_deg} localization={pose.yaw_deg} />
          <div className="telemetry-metric-grid telemetry-metric-grid-3">
            <Metric label="Confidence" value={numberText(localization.confidence, 2)} />
            <Metric label="GNSS innovation" value={numberText(localization.gnss_innovation_norm, 2)} />
            <Metric label="Wheel innovation" value={numberText(localization.wheel_innovation_norm, 2)} />
          </div>
          <div className="telemetry-health-flags">
            {['gnss', 'imu', 'wheel'].map(key => (
              <span key={key} className={localization[`${key}_ok`] ? 'ok' : 'bad'}>
                {key.toUpperCase()} {localization[`${key}_ok`] ? 'OK' : 'CHECK'}
              </span>
            ))}
          </div>
        </section>
      </div>
    </div>
  );
}

function radarArcPoints(mount, distance, fov) {
  const [x, y, yaw] = mount;
  const points = [];
  const half = Math.max(0.05, fov || 0.35) / 2;
  for (let index = 0; index <= 18; index += 1) {
    const angle = yaw - half + (2 * half * index / 18);
    points.push([x + Math.cos(angle) * distance, y + Math.sin(angle) * distance]);
  }
  return points;
}

function ProximityPlot({ telemetry }) {
  const channels = telemetry.radar?.channels || {};
  const filtered = telemetry.lidar?.streams?.filtered?.points || telemetry.lidar?.points || [];
  const raw = telemetry.lidar?.streams?.raw?.points || [];
  const planning = telemetry.footprint?.planning_points?.length
    ? telemetry.footprint.planning_points : (telemetry.footprint?.points || []);
  const width = 680;
  const height = 500;
  const scale = 78;
  const toScreen = point => [width / 2 - point[1] * scale, height / 2 - point[0] * scale];
  const pointsText = points => points.map(point => toScreen(point).join(',')).join(' ');
  const fallbackBody = [
    [-0.68323, 0.53505], [0.42, 0.53505], [0.70837, 0.40],
    [0.70837, -0.40], [0.42, -0.53495], [-0.68323, -0.53495],
  ];
  const body = telemetry.footprint?.physical_points?.length
    ? telemetry.footprint.physical_points : fallbackBody;
  return (
    <svg className="proximity-plot" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Radar lidar top down proximity view">
      <defs>
        <pattern id="sensorGrid" width={scale} height={scale} patternUnits="userSpaceOnUse">
          <path d={`M ${scale} 0 L 0 0 0 ${scale}`} className="sensor-grid-line" />
        </pattern>
      </defs>
      <rect width={width} height={height} fill="url(#sensorGrid)" />
      <circle cx={width / 2} cy={height / 2} r={scale} className="sensor-range-ring" />
      <circle cx={width / 2} cy={height / 2} r={scale * 2} className="sensor-range-ring" />
      <circle cx={width / 2} cy={height / 2} r={scale * 3} className="sensor-range-ring" />
      {raw.map((point, index) => {
        const [x, y] = toScreen(point);
        if (x < 0 || x > width || y < 0 || y > height) return null;
        return <circle key={`raw-${index}-${point[0]}`} cx={x} cy={y} r="1.7" className="lidar-raw-point" />;
      })}
      {filtered.map((point, index) => {
        const [x, y] = toScreen(point);
        if (x < 0 || x > width || y < 0 || y > height) return null;
        return <circle key={`filtered-${index}-${point[0]}`} cx={x} cy={y} r="2.3" className="lidar-point" />;
      })}
      {RADAR_ORDER.map(channel => {
        const sample = channels[channel] || {};
        const source = sourceState(telemetry, `radar.${channel}`);
        const configuredMax = finite(sample.max_range_m) ? sample.max_range_m : 3;
        const distance = sample.no_target ? configuredMax : sample.range_m;
        if (!finite(distance)) return null;
        const clipped = Math.min(3.1, Math.max(0.05, distance));
        const arc = radarArcPoints(RADAR_MOUNTS[channel], clipped, sample.field_of_view_rad);
        const colorClass = source.state !== 'live'
          ? 'radar-arc-stale'
          : (sample.no_target || distance > configuredMax ? 'radar-arc-clear' : 'radar-arc-hit');
        const [mx, my] = toScreen(RADAR_MOUNTS[channel]);
        return (
          <g key={channel}>
            <polyline points={pointsText(arc)} className={`radar-arc ${colorClass}`} />
            <circle cx={mx} cy={my} r="5" className={`radar-mount ${colorClass}`} />
          </g>
        );
      })}
      {planning.length >= 3 && <polygon points={pointsText(planning)} className="planning-footprint" />}
      <polygon points={pointsText(body)} className="physical-footprint" />
      <line x1={width / 2} y1={height / 2} x2={width / 2} y2={height / 2 - 72} className="robot-forward-axis" />
      <circle cx={width / 2} cy={height / 2} r="5" className="robot-center-point" />
      <text x={width / 2 + 9} y={height / 2 + 18} className="sensor-plot-label">robot_center_link</text>
    </svg>
  );
}

function ProximityView({ telemetry }) {
  const channels = telemetry.radar?.channels || {};
  const lidarSource = sourceState(telemetry, 'lidar.filtered');
  const filtered = telemetry.lidar?.streams?.filtered || telemetry.lidar || {};
  const raw = telemetry.lidar?.streams?.raw || {};
  return (
    <div className="telemetry-view telemetry-proximity-view">
      <div className="telemetry-source-row">
        {RADAR_ORDER.map(channel => (
          <SourcePill key={channel} telemetry={telemetry} source={`radar.${channel}`} label={RADAR_LABEL[channel]} />
        ))}
        <SourcePill telemetry={telemetry} source="lidar.filtered" label="LiDAR filtered" />
        <SourcePill telemetry={telemetry} source="lidar.raw" label="LiDAR raw" />
        <SourcePill telemetry={telemetry} source="platform.planning_boundary" label="Planning boundary" staleAfter={5} />
      </div>
      <div className="telemetry-proximity-layout">
        <section className="telemetry-section telemetry-plot-section">
          <SectionHeader title="Top-down proximity" meta={`Filtered ${filtered.sample_count || 0} / ${filtered.point_count || 0}`} />
          <ProximityPlot telemetry={telemetry} />
          <div className="telemetry-legend-row">
            <span><i className="legend-lidar" />LiDAR sample</span>
            <span><i className="legend-lidar-raw" />LiDAR raw</span>
            <span><i className="legend-radar-hit" />Radar return</span>
            <span><i className="legend-body" />Physical body</span>
            <span><i className="legend-margin" />Planning boundary</span>
          </div>
        </section>
        <section className="telemetry-section telemetry-radar-table-section">
          <SectionHeader title="Seven-channel radar" meta={lidarSource.state === 'live' ? `LiDAR ${lidarSource.rate?.toFixed(1) || '-'} Hz` : 'LiDAR unavailable'} />
          <div className="telemetry-table-wrap">
            <table className="telemetry-table">
              <thead><tr><th>Channel</th><th>Range</th><th>Window</th><th>Rate</th><th>State</th></tr></thead>
              <tbody>
                {RADAR_ORDER.map(channel => {
                  const sample = channels[channel] || {};
                  const source = sourceState(telemetry, `radar.${channel}`);
                  const state = source.state !== 'live' ? source.label : (sample.no_target ? 'CLEAR' : (finite(sample.range_m) ? 'RETURN' : 'NO DATA'));
                  return (
                    <tr key={channel}>
                      <td>{RADAR_LABEL[channel]}</td>
                      <td>{sample.no_target ? '∞' : numberText(sample.range_m, 2)} m</td>
                      <td>{numberText(sample.min_range_m, 1)} - {numberText(sample.max_range_m, 1)} m</td>
                      <td>{numberText(source.rate, 1)} Hz</td>
                      <td><span className={`table-state table-state-${state.toLowerCase().replace(' ', '-')}`}>{state}</span></td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          </div>
          <div className="telemetry-subsection">
            <span>LiDAR frame</span>
            <div className="telemetry-inline-values">
              <b>{filtered.frame_id || '-'}</b>
              <b>{filtered.sample_count || 0} filtered</b>
              <b>{raw.sample_count || 0} raw</b>
            </div>
          </div>
        </section>
      </div>
    </div>
  );
}

function CameraFeed({ telemetry, camera, label }) {
  const data = telemetry.cameras?.[camera] || {};
  const source = sourceState(telemetry, `camera.${camera}`, 3);
  const [loadFailed, setLoadFailed] = useState(false);
  useEffect(() => setLoadFailed(false), [data.sequence]);
  const canDisplay = data.available && !loadFailed;
  return (
    <section className="telemetry-section camera-feed">
      <SectionHeader title={label} meta={`${source.label}${finite(source.rate) ? ` · ${source.rate.toFixed(1)} Hz` : ''}`} />
      <div className={`camera-frame camera-frame-${source.state}`}>
        {canDisplay ? (
          <img
            src={`/api/camera/${camera}?frame=${data.sequence}`}
            alt={`${label} live ROS camera`}
            onError={() => setLoadFailed(true)}
          />
        ) : (
          <div className="camera-empty">
            <strong>NO FRAME</strong>
            <span>{camera === 'front'
              ? '/sensing/camera/econ_front/image_rect/compressed'
              : (camera === 'docking'
                ? '/perception/apriltag_parking_detector/debug_image/compressed'
                : '/sensing/camera/econ_rear/image_raw (+ compressed when available)')}</span>
          </div>
        )}
      </div>
      <div className="camera-metadata">
        <span>Frame <b>{data.frame_id || '-'}</b></span>
        <span>Format <b>{data.format || '-'}</b></span>
        <span>Source <b>{data.source || '-'}</b></span>
        <span>Sensor target <b>10 Hz</b></span>
        <span>Payload <b>{finite(data.bytes) ? `${(data.bytes / 1024).toFixed(1)} KiB` : '-'}</b></span>
        <span>Age <b>{finite(source.age) ? `${source.age.toFixed(2)} s` : '-'}</b></span>
      </div>
    </section>
  );
}

function CameraView({ telemetry }) {
  return (
    <div className="telemetry-view telemetry-camera-view">
      <div className="telemetry-source-row">
        <SourcePill telemetry={telemetry} source="camera.front" label="Front · target 10 Hz" staleAfter={3} />
        <SourcePill telemetry={telemetry} source="camera.rear" label="Rear · target 10 Hz" staleAfter={3} />
      </div>
      <div className="camera-grid">
        <CameraFeed telemetry={telemetry} camera="front" label="Front camera" />
        <CameraFeed telemetry={telemetry} camera="rear" label="Rear camera" />
      </div>
    </div>
  );
}

function pointList(path) {
  return Array.isArray(path?.points) ? path.points.filter(point => finite(point?.[0]) && finite(point?.[1])) : [];
}

function TrajectoryPlot({ telemetry, mapData, goalDraft, goalSelectionActive, onGoalDraft }) {
  const goalDragStartRef = useRef(null);
  const global = pointList(telemetry.paths?.global);
  const local = pointList(telemetry.paths?.local);
  const trace = Array.isArray(telemetry.localization?.trace) ? telemetry.localization.trace : [];
  const pose = telemetry.localization?.pose || {};
  const mapPolylines = Array.isArray(mapData.polylines) ? mapData.polylines : [];
  // HH_260810 - Normal tracking remains route-focused, but Goal Pose mode fits
  // the whole available lanelet map so an idle robot can select a remote goal.
  const goalSelectionMapPoints = goalSelectionActive
    ? mapPolylines.flatMap(line => pointList(line))
    : [];
  const maneuvers = Object.entries(telemetry.paths?.maneuvers || {})
    .map(([name, path]) => [name, pointList(path)])
    .filter(([, points]) => points.length > 0);
  const focus = [
    ...global,
    ...local,
    ...trace,
    ...maneuvers.flatMap(([, points]) => points),
    ...goalSelectionMapPoints,
  ];
  if (finite(pose.x) && finite(pose.y)) focus.push([pose.x, pose.y]);
  if (finite(goalDraft?.x) && finite(goalDraft?.y)) focus.push([goalDraft.x, goalDraft.y]);
  const seed = focus.length ? focus : [[-10, -7], [10, 7]];
  let minX = Math.min(...seed.map(point => point[0]));
  let maxX = Math.max(...seed.map(point => point[0]));
  let minY = Math.min(...seed.map(point => point[1]));
  let maxY = Math.max(...seed.map(point => point[1]));
  const spanX = Math.max(8, maxX - minX);
  const spanY = Math.max(6, maxY - minY);
  minX -= spanX * 0.12; maxX += spanX * 0.12;
  minY -= spanY * 0.12; maxY += spanY * 0.12;
  const width = 920;
  const height = 520;
  const scale = Math.min(width / (maxX - minX), height / (maxY - minY));
  const drawWidth = (maxX - minX) * scale;
  const drawHeight = (maxY - minY) * scale;
  const offsetX = (width - drawWidth) / 2;
  const offsetY = (height - drawHeight) / 2;
  const toScreen = point => [
    offsetX + (point[0] - minX) * scale,
    height - offsetY - (point[1] - minY) * scale,
  ];
  const asText = points => points.map(point => toScreen(point).map(value => value.toFixed(1)).join(',')).join(' ');
  const mapLines = mapPolylines.filter(line =>
    pointList(line).some(point => point[0] >= minX && point[0] <= maxX && point[1] >= minY && point[1] <= maxY)
  ).slice(0, 900);
  const [robotX, robotY] = finite(pose.x) && finite(pose.y) ? toScreen([pose.x, pose.y]) : [null, null];
  const yawRad = finite(pose.yaw_deg) ? pose.yaw_deg * Math.PI / 180 : 0;
  const robotHead = finite(robotX) ? [robotX + Math.cos(yawRad) * 28, robotY - Math.sin(yawRad) * 28] : [null, null];
  const goalPoint = finite(goalDraft?.x) && finite(goalDraft?.y)
    ? toScreen([goalDraft.x, goalDraft.y]) : [null, null];
  const goalYawRad = finite(goalDraft?.yaw_deg) ? goalDraft.yaw_deg * Math.PI / 180 : 0;
  const goalHead = finite(goalPoint[0])
    ? [goalPoint[0] + Math.cos(goalYawRad) * 44, goalPoint[1] - Math.sin(goalYawRad) * 44]
    : [null, null];

  const eventToMap = event => {
    const svg = event.currentTarget;
    const matrix = svg.getScreenCTM();
    if (!matrix) return null;
    const screenPoint = svg.createSVGPoint();
    screenPoint.x = event.clientX;
    screenPoint.y = event.clientY;
    const localPoint = screenPoint.matrixTransform(matrix.inverse());
    if (
      localPoint.x < offsetX || localPoint.x > offsetX + drawWidth
      || localPoint.y < offsetY || localPoint.y > offsetY + drawHeight
    ) return null;
    return [
      minX + (localPoint.x - offsetX) / scale,
      minY + (height - offsetY - localPoint.y) / scale,
    ];
  };

  const beginGoalSelection = event => {
    if (!goalSelectionActive || !onGoalDraft) return;
    const point = eventToMap(event);
    if (!point) return;
    event.preventDefault();
    event.currentTarget.setPointerCapture?.(event.pointerId);
    goalDragStartRef.current = point;
    onGoalDraft({
      x: point[0],
      y: point[1],
      yaw_deg: finite(pose.yaw_deg) ? pose.yaw_deg : 0,
    });
  };

  const updateGoalHeading = event => {
    const start = goalDragStartRef.current;
    if (!goalSelectionActive || !start || !onGoalDraft) return;
    const point = eventToMap(event);
    if (!point) return;
    event.preventDefault();
    const dx = point[0] - start[0];
    const dy = point[1] - start[1];
    const yaw = Math.hypot(dx, dy) >= 0.15
      ? Math.atan2(dy, dx) * 180 / Math.PI
      : (finite(pose.yaw_deg) ? pose.yaw_deg : 0);
    onGoalDraft({ x: start[0], y: start[1], yaw_deg: yaw });
  };

  const finishGoalSelection = event => {
    if (!goalDragStartRef.current) return;
    updateGoalHeading(event);
    goalDragStartRef.current = null;
    event.currentTarget.releasePointerCapture?.(event.pointerId);
  };

  return (
    <svg
      className={`trajectory-plot ${goalSelectionActive ? 'trajectory-plot-goal-active' : ''}`}
      viewBox={`0 0 ${width} ${height}`}
      role="img"
      aria-label="Lanelet route local path driven trajectory and manual goal selector"
      onPointerDown={beginGoalSelection}
      onPointerMove={updateGoalHeading}
      onPointerUp={finishGoalSelection}
      onPointerCancel={finishGoalSelection}
    >
      <defs>
        <pattern id="routeGrid" width="46" height="46" patternUnits="userSpaceOnUse">
          <path d="M 46 0 L 0 0 0 46" className="route-grid-line" />
        </pattern>
        <marker id="manualGoalArrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">
          <path d="M 0 0 L 10 5 L 0 10 z" className="manual-goal-arrow-head" />
        </marker>
      </defs>
      <rect width={width} height={height} fill="url(#routeGrid)" />
      {mapLines.map((line, index) => <polyline key={`${line.kind}-${index}`} points={asText(pointList(line))} className="trajectory-map-line" />)}
      {global.length > 1 && <polyline points={asText(global)} className="trajectory-global" />}
      {local.length > 1 && <polyline points={asText(local)} className="trajectory-local" />}
      {maneuvers.map(([name, points]) => <polyline key={name} points={asText(points)} className={`trajectory-maneuver trajectory-maneuver-${name}`} />)}
      {trace.length > 1 && <polyline points={asText(trace)} className="trajectory-trace" />}
      {finite(robotX) && (
        <g>
          <circle cx={robotX} cy={robotY} r="11" className="trajectory-robot" />
          <line x1={robotX} y1={robotY} x2={robotHead[0]} y2={robotHead[1]} className="trajectory-heading" />
        </g>
      )}
      {finite(goalPoint[0]) && (
        <g className="manual-goal-marker">
          <circle cx={goalPoint[0]} cy={goalPoint[1]} r="15" />
          <line x1={goalPoint[0] - 21} y1={goalPoint[1]} x2={goalPoint[0] + 21} y2={goalPoint[1]} />
          <line x1={goalPoint[0]} y1={goalPoint[1] - 21} x2={goalPoint[0]} y2={goalPoint[1] + 21} />
          <line
            x1={goalPoint[0]}
            y1={goalPoint[1]}
            x2={goalHead[0]}
            y2={goalHead[1]}
            className="manual-goal-heading"
            markerEnd="url(#manualGoalArrow)"
          />
        </g>
      )}
    </svg>
  );
}

function TrajectoryView({ telemetry, mapData }) {
  const [goalDraft, setGoalDraft] = useState(null);
  const [goalSelectionActive, setGoalSelectionActive] = useState(false);
  const [goalConfirmOpen, setGoalConfirmOpen] = useState(false);
  const [goalDispatching, setGoalDispatching] = useState(false);
  const [goalStatus, setGoalStatus] = useState({ tone: '', message: '' });
  const velocity = telemetry.motion?.velocity || {};
  const error = telemetry.motion?.tracking_error || {};
  const mission = telemetry.mission || {};
  const globalPoints = pointList(telemetry.paths?.global);
  const localPoints = pointList(telemetry.paths?.local);

  const clearGoal = () => {
    setGoalDraft(null);
    setGoalSelectionActive(false);
    setGoalConfirmOpen(false);
    setGoalStatus({ tone: '', message: '' });
  };

  const dispatchManualGoal = async () => {
    if (!goalDraft || goalDispatching) return;
    setGoalDispatching(true);
    setGoalStatus({ tone: '', message: '' });
    const query = new URLSearchParams({
      x: goalDraft.x.toFixed(6),
      y: goalDraft.y.toFixed(6),
      yaw_deg: goalDraft.yaw_deg.toFixed(3),
    });
    try {
      const response = await fetch(`/ui/manual_goal?${query.toString()}`, { method: 'POST' });
      const body = await response.json();
      if (!response.ok || !body.success) throw new Error(body.message || body.error || 'manual goal rejected');
      setGoalSelectionActive(false);
      setGoalConfirmOpen(false);
      setGoalStatus({ tone: 'ok', message: '수동 목표가 전송되어 주행을 시작했습니다.' });
    } catch (error) {
      setGoalStatus({ tone: 'error', message: error.message || '수동 목표 전송 실패' });
    } finally {
      setGoalDispatching(false);
    }
  };

  return (
    <div className="telemetry-view telemetry-trajectory-view">
      <div className="telemetry-source-row">
        <SourcePill telemetry={telemetry} source="localization.pose" label="Pose" />
        <SourcePill telemetry={telemetry} source="platform.velocity" label="Velocity" />
        <SourcePill telemetry={telemetry} source="path.global" label="Global path" staleAfter={30} />
        <SourcePill telemetry={telemetry} source="path.local" label="Local path" />
        <SourcePill telemetry={telemetry} source="planning.tracking_error" label="Tracking error" />
        <SourcePill telemetry={telemetry} source="map.markers" label="Lanelet map" staleAfter={3600} />
      </div>
      <div className="telemetry-trajectory-layout">
        <section className="telemetry-section telemetry-route-section">
          <SectionHeader title="Route and driven trace" meta={`${mapData.point_count || 0} map points`} />
          <TrajectoryPlot
            telemetry={telemetry}
            mapData={mapData}
            goalDraft={goalDraft}
            goalSelectionActive={goalSelectionActive}
            onGoalDraft={draft => {
              setGoalDraft(draft);
              setGoalConfirmOpen(false);
              setGoalStatus({ tone: '', message: '' });
            }}
          />
          <div className="telemetry-legend-row">
            <span><i className="legend-global" />Global</span>
            <span><i className="legend-local" />Local</span>
            <span><i className="legend-maneuver" />Maneuver</span>
            <span><i className="legend-trace" />Driven trace</span>
            <span><i className="legend-manual-goal" />Manual goal</span>
          </div>
        </section>
        <section className="telemetry-section telemetry-route-metrics">
          <SectionHeader title="Tracking" meta={error.active_path_source || 'no active path'} />
          <div className="telemetry-metric-grid telemetry-metric-grid-2">
            <Metric label="Speed" value={numberText(velocity.speed_kph, 2)} unit="km/h" />
            <Metric label="Yaw rate" value={numberText(velocity.yaw_rate_rps, 3)} unit="rad/s" />
            <Metric label="Lateral error" value={numberText(error.active_lateral_m, 3)} unit="m" tone={finite(error.active_lateral_m) && Math.abs(error.active_lateral_m) > 0.3 ? 'warn' : ''} />
            <Metric label="Heading error" value={numberText(error.active_heading_deg, 2)} unit="°" />
            <Metric label="Path distance" value={numberText(error.active_distance_m, 3)} unit="m" />
            <Metric label="Motion mode" value={velocity.motion_mode ?? '-'} />
          </div>
          <div className="telemetry-path-counts">
            <div><span>Global path</span><b>{globalPoints.length}</b><em>{telemetry.paths?.global?.raw_point_count || 0} raw</em></div>
            <div><span>Local path</span><b>{localPoints.length}</b><em>{telemetry.paths?.local?.raw_point_count || 0} raw</em></div>
            <div><span>Driven trace</span><b>{telemetry.localization?.trace?.length || 0}</b><em>bounded</em></div>
          </div>
          <div className="telemetry-pose-readout">
            <span>X <b>{numberText(telemetry.localization?.pose?.x, 3)} m</b></span>
            <span>Y <b>{numberText(telemetry.localization?.pose?.y, 3)} m</b></span>
            <span>Yaw <b>{numberText(telemetry.localization?.pose?.yaw_deg, 1)}°</b></span>
          </div>
          <div className="manual-goal-control">
            <div className="manual-goal-control-header">
              <strong>수동 Goal Pose</strong>
              <span>map</span>
            </div>
            <div className="manual-goal-toolbar">
              <button
                type="button"
                className={`manual-goal-tool ${goalSelectionActive ? 'active' : ''}`}
                onClick={() => {
                  setGoalSelectionActive(active => !active);
                  setGoalConfirmOpen(false);
                  setGoalStatus({ tone: '', message: '' });
                }}
                aria-pressed={goalSelectionActive}
                title="지도에서 위치를 누르고 드래그해 방향 지정"
              >
                <span aria-hidden="true">⌖</span>
                목표 선택
              </button>
              <button
                type="button"
                className="manual-goal-tool manual-goal-clear"
                onClick={clearGoal}
                disabled={!goalDraft}
                title="선택한 목표 지우기"
              >
                <span aria-hidden="true">×</span>
                지우기
              </button>
            </div>
            <div className="manual-goal-readout">
              <span>X <b>{numberText(goalDraft?.x, 3)} m</b></span>
              <span>Y <b>{numberText(goalDraft?.y, 3)} m</b></span>
              <span>Yaw <b>{numberText(goalDraft?.yaw_deg, 1)}°</b></span>
            </div>
            {goalConfirmOpen ? (
              <div className="manual-goal-confirm">
                <strong>선택한 목표로 출발하시겠습니까?</strong>
                <div>
                  <button type="button" onClick={() => setGoalConfirmOpen(false)} disabled={goalDispatching}>취소</button>
                  <button type="button" className="manual-goal-start" onClick={dispatchManualGoal} disabled={goalDispatching}>
                    {goalDispatching ? '전송 중' : '출발'}
                  </button>
                </div>
              </div>
            ) : (
              <button
                type="button"
                className="manual-goal-dispatch"
                onClick={() => setGoalConfirmOpen(true)}
                disabled={!goalDraft || !mission.ready}
                title={mission.ready ? '수동 목표 확인' : '시스템 준비 완료 후 사용할 수 있습니다'}
              >
                <span aria-hidden="true">➤</span>
                목표 확인
              </button>
            )}
            {!mission.ready && <div className="manual-goal-status warn">시스템 준비 대기</div>}
            {goalStatus.message && <div className={`manual-goal-status ${goalStatus.tone}`}>{goalStatus.message}</div>}
          </div>
        </section>
      </div>
    </div>
  );
}

const COST_LAYER_META = {
  lanelet: { label: 'Lane boundary', color: '#8d99a6' },
  lidar: { label: 'LiDAR cost', color: '#d8534f' },
  radar: { label: 'Radar cost', color: '#e2a93b' },
  inflation: { label: 'Inflation', color: '#527ca8' },
};

function localToMap(point, pose) {
  if (!finite(pose?.x) || !finite(pose?.y) || !finite(pose?.yaw_deg)) return null;
  const yaw = pose.yaw_deg * Math.PI / 180;
  return [
    pose.x + Math.cos(yaw) * point[0] - Math.sin(yaw) * point[1],
    pose.y + Math.sin(yaw) * point[0] + Math.cos(yaw) * point[1],
  ];
}

function MapPerceptionPlot({ telemetry, mapData }) {
  const pose = telemetry.localization?.pose || {};
  const obstacleCloud = pointList(telemetry.perception?.obstacle_cloud);
  const boxes = telemetry.perception?.obstacle_boxes?.polylines || [];
  const layers = telemetry.perception?.cost_layers || {};
  const layerPoints = Object.values(layers).flatMap(layer =>
    Array.isArray(layer?.occupied_samples) ? layer.occupied_samples : []
  );
  const fallbackPoint = obstacleCloud[0] || layerPoints[0] ||
    pointList(mapData.polylines?.[0] || {})[0] || [0, 0];
  const centerX = finite(pose.x) ? pose.x : fallbackPoint[0];
  const centerY = finite(pose.y) ? pose.y : fallbackPoint[1];
  const width = 920;
  const height = 520;
  const radiusX = 16;
  const radiusY = 9.2;
  const minX = centerX - radiusX;
  const maxX = centerX + radiusX;
  const minY = centerY - radiusY;
  const maxY = centerY + radiusY;
  const scale = Math.min(width / (maxX - minX), height / (maxY - minY));
  const toScreen = point => [
    (point[0] - minX) * scale,
    height - (point[1] - minY) * scale,
  ];
  const asText = points => points
    .filter(point => finite(point?.[0]) && finite(point?.[1]))
    .map(point => toScreen(point).map(value => value.toFixed(1)).join(','))
    .join(' ');
  const visible = point => point[0] >= minX && point[0] <= maxX && point[1] >= minY && point[1] <= maxY;
  const mapLines = (mapData.polylines || []).filter(line => pointList(line).some(visible)).slice(0, 900);
  const cellPath = cells => (cells || []).filter(visible).map(cell => {
    const [x, y] = toScreen(cell);
    return `M${(x - 1.5).toFixed(1)},${(y - 1.5).toFixed(1)}h3v3h-3z`;
  }).join('');
  const physicalLocal = telemetry.footprint?.physical_points || [];
  const planningLocal = telemetry.footprint?.planning_points || telemetry.footprint?.points || [];
  const physical = physicalLocal.map(point => localToMap(point, pose)).filter(Boolean);
  const planning = planningLocal.map(point => localToMap(point, pose)).filter(Boolean);
  const [robotX, robotY] = finite(pose.x) && finite(pose.y) ? toScreen([pose.x, pose.y]) : [null, null];
  const yaw = finite(pose.yaw_deg) ? pose.yaw_deg * Math.PI / 180 : 0;

  return (
    <svg className="perception-map-plot" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Lanelet cost grids obstacles and robot footprint">
      <defs>
        <pattern id="perceptionGrid" width="46" height="46" patternUnits="userSpaceOnUse">
          <path d="M 46 0 L 0 0 0 46" className="route-grid-line" />
        </pattern>
      </defs>
      <rect width={width} height={height} fill="url(#perceptionGrid)" />
      {mapLines.map((line, index) => (
        <polyline key={`${line.kind}-${index}`} points={asText(pointList(line))} className="perception-map-line" />
      ))}
      {Object.entries(COST_LAYER_META).map(([name, meta]) => {
        const path = cellPath(layers[name]?.occupied_samples);
        return path ? <path key={name} d={path} fill={meta.color} className={`cost-layer cost-layer-${name}`} /> : null;
      })}
      {obstacleCloud.filter(visible).map((point, index) => {
        const [x, y] = toScreen(point);
        return <circle key={`obstacle-${index}`} cx={x} cy={y} r="2.7" className="perception-obstacle-point" />;
      })}
      {boxes.map((line, index) => (
        <polyline key={`box-${index}`} points={asText(pointList(line))} className="perception-obstacle-box" />
      ))}
      {planning.length >= 3 && <polygon points={asText(planning)} className="perception-planning-footprint" />}
      {physical.length >= 3 && <polygon points={asText(physical)} className="perception-physical-footprint" />}
      {finite(robotX) && (
        <g>
          <circle cx={robotX} cy={robotY} r="7" className="trajectory-robot" />
          <line
            x1={robotX} y1={robotY}
            x2={robotX + Math.cos(yaw) * 25}
            y2={robotY - Math.sin(yaw) * 25}
            className="trajectory-heading"
          />
        </g>
      )}
    </svg>
  );
}

function MapPerceptionView({ telemetry, mapData }) {
  const perception = telemetry.perception || {};
  const layers = perception.cost_layers || {};
  const obstacleCloud = perception.obstacle_cloud || {};
  const boxes = perception.obstacle_boxes || {};
  return (
    <div className="telemetry-view telemetry-perception-view">
      <div className="telemetry-source-row">
        <SourcePill telemetry={telemetry} source="map.markers" label="Lanelet map" staleAfter={3600} />
        <SourcePill telemetry={telemetry} source="cost.lanelet" label="Lane cost" staleAfter={5} />
        <SourcePill telemetry={telemetry} source="cost.lidar" label="LiDAR cost" staleAfter={5} />
        <SourcePill telemetry={telemetry} source="cost.radar" label="Radar cost" staleAfter={5} />
        <SourcePill telemetry={telemetry} source="perception.obstacles" label="Obstacle cloud" />
        <SourcePill telemetry={telemetry} source="perception.boxes" label="Bounding boxes" />
      </div>
      <div className="telemetry-perception-layout">
        <section className="telemetry-section telemetry-perception-map-section">
          <SectionHeader title="Map and perception overlay" meta="32 m × 18.4 m robot-centered window" />
          <MapPerceptionPlot telemetry={telemetry} mapData={mapData} />
          <div className="telemetry-legend-row perception-legend">
            {Object.entries(COST_LAYER_META).map(([name, meta]) => (
              <span key={name}><i style={{ background: meta.color }} />{meta.label}</span>
            ))}
            <span><i className="legend-obstacle" />Obstacle</span>
            <span><i className="legend-box" />BBox</span>
          </div>
        </section>
        <section className="telemetry-section telemetry-perception-metrics">
          <SectionHeader title="Runtime layers" meta={`${mapData.point_count || 0} lanelet points`} />
          <div className="perception-layer-list">
            {Object.entries(COST_LAYER_META).map(([name, meta]) => {
              const layer = layers[name] || {};
              const source = sourceState(telemetry, `cost.${name}`, 5);
              return (
                <div key={name} className="perception-layer-row">
                  <span style={{ background: meta.color }} />
                  <strong>{meta.label}</strong>
                  <b>{source.state === 'live' ? `${layer.sample_count || 0} cells` : source.label}</b>
                  <em>{finite(layer.resolution_m) ? `${layer.resolution_m.toFixed(2)} m` : '-'}</em>
                </div>
              );
            })}
          </div>
          <div className="telemetry-metric-grid telemetry-metric-grid-2 perception-count-grid">
            <Metric label="Obstacle samples" value={obstacleCloud.sample_count ?? 0} />
            <Metric label="Obstacle source points" value={obstacleCloud.point_count ?? 0} />
            <Metric label="BBox outlines" value={boxes.outline_count ?? 0} />
            <Metric label="Map polylines" value={mapData.polylines?.length ?? 0} />
          </div>
          <div className="telemetry-subsection">
            <span>Coordinate frames</span>
            <div className="telemetry-inline-values">
              <b>Map {mapData.frame_id || '-'}</b>
              <b>Obstacle {obstacleCloud.frame_id || '-'}</b>
              <b>Robot {telemetry.footprint?.frame_id || '-'}</b>
            </div>
          </div>
        </section>
      </div>
    </div>
  );
}

function levelClass(level) {
  if (level === 2) return 'error';
  if (level === 1) return 'warn';
  return 'ok';
}

function SafetyView({ telemetry }) {
  const gate = telemetry.safety?.gate || {};
  const controllers = telemetry.safety?.controllers || {};
  const mission = telemetry.mission || {};
  const controllerOrder = ['camping_site', 'drop_zone', 'route_recovery', 'reverse_parking', 'apriltag_parking'];
  const controllerLabel = {
    camping_site: 'Camping site', drop_zone: 'Drop zone', route_recovery: 'Route recovery',
    reverse_parking: 'Reverse parking', apriltag_parking: 'AprilTag parking',
  };
  return (
    <div className="telemetry-view telemetry-safety-view">
      <div className="telemetry-source-row">
        <SourcePill telemetry={telemetry} source="control.safety_gate" label="Safety gate" />
        <SourcePill telemetry={telemetry} source="safety.radar_evidence" label="Radar evidence" />
        <SourcePill telemetry={telemetry} source="safety.obstacle_replan" label="Obstacle replan" />
        {controllerOrder.map(name => <SourcePill key={name} telemetry={telemetry} source={`controller.${name}`} label={controllerLabel[name]} />)}
      </div>
      <section className={`safety-gate-banner safety-gate-${levelClass(gate.level)}`}>
        <div>
          <span>Command safety gate</span>
          <strong>{gate.operating_state || 'WAITING FOR STATUS'}</strong>
        </div>
        <p>{gate.message || '/control/cmd_vel_safety_gate/status 대기 중'}</p>
      </section>
      <div className="telemetry-safety-layout">
        <section className="telemetry-section">
          <SectionHeader title="Motion owners" meta={mission.service_state_name || 'service state unavailable'} />
          <div className="controller-status-list">
            {controllerOrder.map(name => {
              const status = controllers[name] || {};
              return (
                <div key={name} className="controller-status-row">
                  <span className={`controller-level controller-level-${levelClass(status.level)}`} />
                  <strong>{controllerLabel[name]}</strong>
                  <b>{status.operating_state || 'NO DATA'}</b>
                  <em>{status.message || '-'}</em>
                </div>
              );
            })}
          </div>
        </section>
        <section className="telemetry-section">
          <SectionHeader title="Mission state" meta={mission.system_health || 'STARTING'} />
          <div className="telemetry-metric-grid telemetry-metric-grid-3">
            <Metric label="Mission phase" value={mission.phase || '-'} />
            <Metric label="Mission source" value={mission.source || '-'} />
            <Metric label="Service state" value={mission.service_state_name || '-'} />
            <Metric label="Ready" value={mission.ready ? 'YES' : 'NO'} tone={mission.ready ? 'ok' : 'warn'} />
            <Metric label="Engaged" value={mission.engaged ? 'YES' : 'NO'} tone={mission.engaged ? 'ok' : ''} />
            <Metric label="System" value={mission.system_health || '-'} tone={mission.system_health === 'OK' ? 'ok' : 'warn'} />
          </div>
          <div className="safety-evidence-block">
            <span>Radar obstacle evidence</span>
            <code>{telemetry.safety?.radar_evidence || 'NO DATA'}</code>
          </div>
          <div className="safety-evidence-block">
            <span>Obstacle replan</span>
            <code>{telemetry.safety?.obstacle_replan || 'NO DATA'}</code>
          </div>
          {(gate.missing_nodes?.length > 0 || gate.missing_topics?.length > 0) && (
            <div className="safety-missing-list">
              {[...(gate.missing_nodes || []), ...(gate.missing_topics || [])].map(value => <code key={value}>{value}</code>)}
            </div>
          )}
        </section>
      </div>
    </div>
  );
}

function DockingPathPlot({ telemetry }) {
  const reverse = pointList(telemetry.paths?.maneuvers?.reverse_parking);
  const tagGuided = pointList(telemetry.paths?.maneuvers?.apriltag_parking);
  const all = [...reverse, ...tagGuided];
  const width = 760;
  const height = 340;
  if (all.length < 2) {
    return (
      <div className="docking-path-empty">
        <strong>NO PARKING PATH</strong>
        <span>주차 컨트롤러 경로 토픽 대기 중</span>
      </div>
    );
  }
  const xs = all.map(point => point[0]);
  const ys = all.map(point => point[1]);
  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const spanX = Math.max(0.5, maxX - minX);
  const spanY = Math.max(0.5, maxY - minY);
  const scale = Math.min((width - 70) / spanX, (height - 70) / spanY);
  const toScreen = point => [
    35 + (point[0] - minX) * scale,
    height - 35 - (point[1] - minY) * scale,
  ];
  const pathText = points => points
    .map(point => toScreen(point).map(value => value.toFixed(1)).join(','))
    .join(' ');
  const lastPath = tagGuided.length > 0 ? tagGuided : reverse;
  const [targetX, targetY] = toScreen(lastPath[lastPath.length - 1]);
  return (
    <svg className="docking-path-plot" viewBox={`0 0 ${width} ${height}`} role="img" aria-label="Parking controller approach trajectories">
      <defs>
        <pattern id="dockingGrid" width="38" height="38" patternUnits="userSpaceOnUse">
          <path d="M 38 0 L 0 0 0 38" className="docking-grid-line" />
        </pattern>
      </defs>
      <rect width={width} height={height} fill="url(#dockingGrid)" />
      {reverse.length >= 2 && <polyline points={pathText(reverse)} className="docking-path docking-path-reverse" />}
      {tagGuided.length >= 2 && <polyline points={pathText(tagGuided)} className="docking-path docking-path-tag" />}
      <circle cx={targetX} cy={targetY} r="8" className="docking-target" />
      <line x1={targetX - 13} y1={targetY} x2={targetX + 13} y2={targetY} className="docking-target-cross" />
      <line x1={targetX} y1={targetY - 13} x2={targetX} y2={targetY + 13} className="docking-target-cross" />
    </svg>
  );
}

function DockingView({ telemetry }) {
  const docking = telemetry.docking || {};
  const tag = docking.tag || {};
  const controllers = telemetry.safety?.controllers || {};
  const reverse = controllers.reverse_parking || {};
  const april = controllers.apriltag_parking || {};
  const mission = telemetry.mission || {};
  const [pending, setPending] = useState('');
  const [commandStatus, setCommandStatus] = useState({ tone: '', message: '' });

  const postCommand = async (name, url, successMessage) => {
    setPending(name);
    setCommandStatus({ tone: '', message: '' });
    try {
      const response = await fetch(url, { method: 'POST' });
      const body = await response.json();
      if (!response.ok || !body.success) throw new Error(body.message || '명령 실패');
      setCommandStatus({ tone: 'ok', message: successMessage });
      return true;
    } catch (error) {
      setCommandStatus({ tone: 'err', message: error.message || '명령 실패' });
      return false;
    } finally {
      setPending('');
    }
  };

  return (
    <div className="telemetry-view telemetry-docking-view">
      <div className="telemetry-source-row">
        <SourcePill telemetry={telemetry} source="camera.docking" label="AprilTag debug" staleAfter={3} />
        <SourcePill telemetry={telemetry} source="docking.tag_detected" label="Tag detection" />
        <SourcePill telemetry={telemetry} source="docking.tag_pose" label="Tag pose" />
        <SourcePill telemetry={telemetry} source="platform.velocity" label="Charging CAN" />
        <SourcePill telemetry={telemetry} source="controller.apriltag_parking" label="Docking controller" />
        <SourcePill telemetry={telemetry} source="controller.reverse_parking" label="Parking controller" />
      </div>
      <div className="docking-command-bar">
        <button
          type="button"
          className="docking-return-command"
          disabled={Boolean(pending)}
          onClick={() => postCommand('return', '/ui/manual_return', '즉시 복귀 명령 전송됨')}
        >
          {pending === 'return' ? '복귀 요청 중' : '즉시 복귀'}
        </button>
        <div className={`docking-command-status ${commandStatus.tone}`}>{commandStatus.message || '명령 대기'}</div>
      </div>
      <div className="docking-layout">
        <CameraFeed telemetry={telemetry} camera="docking" label="AprilTag docking debug" />
        <section className="telemetry-section docking-status-section">
          <SectionHeader title="Docking state" meta={mission.service_state_name || 'service state unavailable'} />
          <div className="docking-state-banner">
            <div className={docking.tag_detected ? 'detected' : 'missing'}>
              <span>TAG</span><strong>{docking.tag_detected ? 'DETECTED' : 'NOT DETECTED'}</strong>
            </div>
            <div className={docking.is_charging ? 'charging' : 'idle'}>
              <span>CHARGING</span><strong>{docking.is_charging ? 'TRUE · STOP' : 'FALSE'}</strong>
            </div>
          </div>
          <div className="telemetry-metric-grid telemetry-metric-grid-3">
            <Metric label="Tag distance" value={numberText(tag.distance_m, 3)} unit="m" tone={docking.tag_detected ? 'ok' : 'warn'} />
            <Metric label="Camera X" value={numberText(tag.x_m, 3)} unit="m" />
            <Metric label="Camera Y" value={numberText(tag.y_m, 3)} unit="m" />
            <Metric label="Camera Z" value={numberText(tag.z_m, 3)} unit="m" />
            <Metric label="Tag yaw" value={numberText(tag.yaw_deg, 1)} unit="°" />
            <Metric label="Battery" value={numberText(docking.battery_percentage, 1)} unit="%" />
          </div>
          <div className="docking-controller-list">
            <div><span>Docking</span><strong>{april.operating_state || 'NO DATA'}</strong><em>{april.message || '-'}</em></div>
            <div><span>Reverse parking</span><strong>{reverse.operating_state || 'NO DATA'}</strong><em>{reverse.message || '-'}</em></div>
          </div>
        </section>
        <section className="telemetry-section docking-path-section">
          <SectionHeader title="Parking approach path" meta="controller output" />
          <DockingPathPlot telemetry={telemetry} />
          <div className="telemetry-legend-row">
            <span><i className="legend-docking-reverse" />Reverse parking</span>
            <span><i className="legend-docking-tag" />AprilTag docking</span>
            <span><i className="legend-docking-target" />Target</span>
          </div>
        </section>
      </div>
    </div>
  );
}

export default function TelemetryWorkspace({ activeTab }) {
  const [telemetry, setTelemetry] = useState(EMPTY_TELEMETRY);
  const [mapData, setMapData] = useState({ frame_id: 'map', polylines: [], point_count: 0 });
  const [connectionError, setConnectionError] = useState('');
  const [transport, setTransport] = useState('connecting');

  useEffect(() => {
    let mounted = true;
    let socket = null;
    let fallbackPollTimer = null;
    let reconnectTimer = null;
    let requestInFlight = false;
    const view = encodeURIComponent(activeTab);

    const applySnapshot = body => {
      if (!mounted) return;
      setTelemetry({ ...EMPTY_TELEMETRY, ...body });
      setConnectionError('');
    };

    const renewLease = () => fetch(
      `/api/telemetry/session?active=true&view=${view}`,
      { method: 'POST' }
    ).catch(() => {});

    const poll = async () => {
      if (requestInFlight) return;
      requestInFlight = true;
      try {
        renewLease();
        const response = await fetch('/api/telemetry', { cache: 'no-store' });
        if (!response.ok) throw new Error(`telemetry HTTP ${response.status}`);
        applySnapshot(await response.json());
      } catch (error) {
        if (mounted) setConnectionError(error.message || 'telemetry unavailable');
      } finally {
        requestInFlight = false;
      }
    };

    const stopFallback = () => {
      if (fallbackPollTimer) clearInterval(fallbackPollTimer);
      fallbackPollTimer = null;
    };

    const startFallback = () => {
      if (!mounted || fallbackPollTimer) return;
      setTransport('http-fallback');
      poll();
      fallbackPollTimer = setInterval(poll, 1000);
    };

    const connect = () => {
      if (!mounted) return;
      const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
      const host = window.location.host || 'localhost:8010';
      let currentSocket;
      try {
        currentSocket = new WebSocket(`${protocol}://${host}/ws/telemetry?view=${view}`);
        socket = currentSocket;
      } catch (error) {
        setConnectionError(error.message || 'telemetry WebSocket unavailable');
        startFallback();
        reconnectTimer = setTimeout(connect, 2000);
        return;
      }
      currentSocket.onopen = () => {
        if (!mounted) return;
        stopFallback();
        setTransport('websocket');
        setConnectionError('');
        // HH_260810 - Client-owned lease heartbeats let the backend release
        // high-bandwidth ROS subscriptions after a silent ARM kiosk/network
        // loss; successful server writes alone do not prove the UI is alive.
        currentSocket.send('lease');
        currentSocket.leaseTimer = setInterval(() => {
          if (currentSocket.readyState === WebSocket.OPEN) currentSocket.send('lease');
        }, 4000);
      };
      currentSocket.onmessage = event => {
        try {
          applySnapshot(JSON.parse(event.data));
        } catch (error) {
          if (mounted) setConnectionError(error.message || 'invalid telemetry frame');
        }
      };
      currentSocket.onerror = () => currentSocket.close();
      currentSocket.onclose = () => {
        if (currentSocket.leaseTimer) clearInterval(currentSocket.leaseTimer);
        if (socket === currentSocket) socket = null;
        if (!mounted) return;
        startFallback();
        reconnectTimer = setTimeout(connect, 2000);
      };
    };

    connect();
    return () => {
      mounted = false;
      stopFallback();
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (socket) {
        if (socket.leaseTimer) clearInterval(socket.leaseTimer);
        socket.close();
      }
      fetch(`/api/telemetry/session?active=false&view=${view}`, { method: 'POST', keepalive: true }).catch(() => {});
    };
  }, [activeTab]);

  useEffect(() => {
    if (!['trajectory', 'perception'].includes(activeTab)) return undefined;
    let mounted = true;
    let timer = null;
    const fetchMap = () => fetch('/api/telemetry/map', { cache: 'no-store' })
      .then(response => response.json())
      .then(body => {
        if (!mounted) return;
        setMapData(body);
        // HH_260810 - Acquire late transient-local map data quickly, then treat
        // the lanelet geometry as static to avoid repeated ARM64 JSON redraws.
        const retryMs = body.polylines?.length > 0 ? 15000 : 250;
        timer = setTimeout(fetchMap, retryMs);
      })
      .catch(() => {
        if (mounted) timer = setTimeout(fetchMap, 1000);
      });
    fetchMap();
    return () => { mounted = false; if (timer) clearTimeout(timer); };
  }, [activeTab]);

  const view = useMemo(() => {
    if (activeTab === 'gnss') return <GnssImuView telemetry={telemetry} />;
    if (activeTab === 'proximity') return <ProximityView telemetry={telemetry} />;
    if (activeTab === 'camera') return <CameraView telemetry={telemetry} />;
    if (activeTab === 'trajectory') return <TrajectoryView telemetry={telemetry} mapData={mapData} />;
    if (activeTab === 'perception') return <MapPerceptionView telemetry={telemetry} mapData={mapData} />;
    if (activeTab === 'safety') return <SafetyView telemetry={telemetry} />;
    if (activeTab === 'docking') return <DockingView telemetry={telemetry} />;
    return null;
  }, [activeTab, telemetry, mapData]);

  return (
    <div className="telemetry-workspace">
      <div className="telemetry-session-strip">
        <span className={`telemetry-session-light ${telemetry.session_active ? 'active' : ''}`} />
        <strong>{telemetry.session_active ? 'OPERATOR TELEMETRY ACTIVE' : 'STARTING TELEMETRY SESSION'}</strong>
        <span>View {telemetry.active_view || activeTab}</span>
        <span>{transport === 'websocket'
          ? `LIVE ${numberText(telemetry.stream_rate_hz, 1)} HZ`
          : 'HTTP FALLBACK 1 HZ'}</span>
        <span>Schema v{telemetry.schema_version || 1}</span>
        {connectionError && <em>{connectionError}</em>}
      </div>
      {view}
    </div>
  );
}

import { manualDriveGateStatus } from './manualDriveGateStatus';

// HH_261007 - Guard against an armed lease or stale ENABLED status being
// displayed as current motion admission. No fixture commands robot motion.
const manual = changes => ({ armed: true, safety_gate: {
  fresh: true, blocked: false, operating_state: 'ENABLED', level: 0,
  receipt_age_s: 0.2, source_age_s: 0.2, freshness_timeout_s: 1.5,
  reasons: [], ...changes,
} });

test('ARMED alone is unknown and a fresh gate explicitly permits only commands', () => {
  expect(manualDriveGateStatus({ armed: true }).state).toBe('unknown');
  expect(manualDriveGateStatus(manual()).label).toBe('명령 허용');
});

test('missing, old, malformed or browser-expired telemetry never appears enabled', () => {
  for (const changes of [{ fresh: false }, { receipt_age_s: null },
    { source_age_s: null }, { freshness_timeout_s: 0 }, { receipt_age_s: 2 },
    { source_age_s: 2 }, { source_age_s: -1 }, { receipt_age_s: NaN }]) {
    expect(manualDriveGateStatus(manual(changes)).state).toBe('unknown');
  }
  expect(manualDriveGateStatus(manual(), 1500).state).toBe('unknown');
});

test('boundary hold explains the map/body overlap, not a proven physical collision', () => {
  const result = manualDriveGateStatus(manual({ blocked: true,
    operating_state: 'ROUTE_SAFETY_HOLD', reasons: ['lanelet_physical_body_cost'] }));
  expect(result.label).toBe('이동 차단');
  expect(result.message).toContain('지도 차선 경계와 차체');
});

test('validated manual escape is limited, not an unconditional ENABLED claim', () => {
  const result = manualDriveGateStatus(manual({ blocked: true,
    operating_state: 'ROUTE_SAFETY_HOLD', reasons: ['lanelet_physical_body_cost'],
    recovery_reason: 'manual_projected_recovery' }));
  expect(result.state).toBe('limited');
  expect(result.message).toContain('안전한 탈출 입력만 허용');
});

test('manual escape capability never masks another active safety interlock', () => {
  const result = manualDriveGateStatus(manual({ blocked: true,
    operating_state: 'ROUTE_SAFETY_HOLD', level: 1,
    reasons: ['route_safety_hold=lanelet_physical_body_cost', 'platform_drive_enable=false'],
    recovery_reason: 'manual_projected_recovery' }));
  expect(result.state).toBe('blocked');
  expect(result.message).toContain('구동 허가가 해제');
});

test('estop and unrecognized gate states never claim permission', () => {
  const result = manualDriveGateStatus(manual({ blocked: true,
    operating_state: 'FAULT_HOLD', reasons: ['estop'] }));
  expect(result.message).toContain('비상정지');
  expect(manualDriveGateStatus(manual({ operating_state: 'UNKNOWN' })).state).toBe('blocked');
  expect(manualDriveGateStatus(manual({ level: null })).state).toBe('blocked');
});

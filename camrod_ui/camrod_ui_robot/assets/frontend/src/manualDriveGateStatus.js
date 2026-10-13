// HH_261007 - Display command admission separately from the manual-control
// lease; this helper never changes commands, speed limits or safety authority.
export function manualDriveGateStatus(manual, payloadAgeMs = 0) {
  const gate = manual?.safety_gate;
  const timeout = Number(gate?.freshness_timeout_s);
  const receiptAge = Number(gate?.receipt_age_s);
  const sourceAge = Number(gate?.source_age_s);
  const elapsed = Number(payloadAgeMs) / 1000;
  const fresh = gate?.fresh === true && gate.receipt_age_s != null
    && gate.source_age_s != null && Number.isFinite(timeout) && timeout > 0
    && Number.isFinite(receiptAge) && receiptAge >= 0
    && Number.isFinite(sourceAge) && sourceAge >= -0.1
    && Number.isFinite(elapsed) && elapsed >= 0
    && receiptAge + elapsed <= timeout && sourceAge + elapsed <= timeout;
  if (!fresh) {
    return { state: 'unknown', label: '상태 미확인',
      message: '안전 게이트 상태가 없거나 오래되었습니다. ARMED는 입력 활성화이며 이동 허용을 뜻하지 않습니다.' };
  }
  const state = String(gate.operating_state || '').toUpperCase();
  if (gate.blocked === false && gate.level === 0
      && ['ENABLED', 'DEPARTING_CHARGER'].includes(state)) {
    return { state: 'enabled', label: '명령 허용',
      message: '안전 게이트가 명령을 허용합니다. 표시 속도는 요청값이며 실제 이동 속도가 아닙니다.' };
  }
  const reasonList = Array.isArray(gate.reasons) ? gate.reasons : [];
  const reasons = reasonList.join(',');
  // HH_261007 - A projected manual escape still validates each input against
  // boundary/contact and dynamic obstacles. It is neither unrestricted motion
  // nor a claim that every input will be rejected.
  if (state === 'ROUTE_SAFETY_HOLD' && [0, 1].includes(gate.level)
      && gate.recovery_reason === 'manual_projected_recovery'
      && reasonList.length > 0 && reasonList.every(reason =>
        /^(route_safety_hold=|route_safety_retry_latched=|cost_hold=)/.test(reason)
        || reason === 'lanelet_physical_body_cost')) {
    return { state: 'limited', label: '경계 탈출 제한',
      message: '지도 차선 경계와 차체가 겹쳐 안전한 탈출 입력만 허용됩니다. 장애물·충돌 검사는 계속 적용됩니다.',
      detail: [state, reasons, gate.recovery_reason].filter(Boolean).join(' · ') };
  }
  let message = '안전 게이트가 이동 명령을 차단하고 있습니다.';
  if (/estop|emergency/i.test(reasons)) {
    message = '비상정지로 이동이 차단되었습니다. 정지 원인을 확인해주세요.';
  } else if (/platform_drive_enable=false|engage=false/i.test(reasons)) {
    message = '구동 허가가 해제되어 이동이 차단되었습니다.';
  } else if (/stale|timeout|missing/i.test(reasons)) {
    message = '제어 또는 센서 정보가 오래되었거나 없어 이동이 차단되었습니다.';
  } else if (/battery|charging|gnss|platform_error|vehicle_state|cost_latched/i.test(reasons)) {
    message = '플랫폼·센서 안전 조건으로 이동이 차단되었습니다. 아래 진단 사유를 확인해주세요.';
  } else if (/lanelet|route_safety/i.test(reasons) || state === 'ROUTE_SAFETY_HOLD') {
    message = '지도 차선 경계와 차체가 겹쳐 이동이 차단되었습니다. 안전 게이트가 검증한 탈출 방향만 허용됩니다.';
  } else if (/collision|obstacle|physical_body_cost/i.test(reasons)) {
    message = '장애물 또는 충돌 위험 판정으로 이동이 차단되었습니다.';
  } else if (state === 'FAULT_HOLD') {
    message = '제어 오류 상태로 이동이 차단되었습니다. 진단 화면에서 원인을 확인해주세요.';
  }
  return { state: 'blocked', label: '이동 차단', message,
    detail: [state, reasons].filter(Boolean).join(' · ') };
}

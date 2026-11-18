/**
 * App.js — 로봇 사이트별 토글 컨트롤 메인 컴포넌트
 *
 * 역할:
 *   1. B1~B12 사이트별 토글 버튼 12개를 그리드로 렌더링
 *   2. WebSocket으로 FastAPI 백엔드와 실시간 통신
 *   3. 버튼 클릭 시 {"site": "B1", "state": true}를 백엔드에 전송
 *   4. 백엔드에서 받은 상태로 각 버튼 UI 동기화
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import './App.css';
import TelemetryWorkspace, {
  TELEMETRY_TABS,
  DockingCommandButton,
  dockingAllowedAtServiceState,
  parkingPolicyMessage,
  postDockingRequest,
} from './TelemetryWorkspace';
import {
  ServiceEvidenceDashboard,
  ServiceEvidenceSummary,
  ServiceTripBadge,
  useServiceMetricsSummary,
} from './ServiceEvidence';
import BringupShutdownControl from './BringupShutdownControl';
import { localeOf, resetLang, T, useLang, useT } from './i18n';
import LanguageToggle from './LanguageToggle';
import SnapshotControl from './SnapshotControl';
import DrivingDisplay from './DrivingDisplay';
import { useDrivingDisplayFeed, useDrivingDisplaySnapshot } from './useDrivingDisplay';
import './DrivingIntegration.css';
// HH_261026 - Organize native diagnostics without importing simulator controls.
import AdminNavigation, { getAdminDiagnosticView } from './AdminNavigation';
import './AdminWorkspace.css';
import './AdminForestTheme.css';
// HH_261102 - Charging cargo access has its own confirmed motion flow, not a new customer login.
import ChargingAccessScreen, { chargingAccessBusy, useChargingAccess } from './ChargingAccessScreen';

// HH_260619 - Developer/test builds bypass the public operating-hours gate by default.
// Enable the kiosk time gate explicitly with REACT_APP_OPERATING_HOURS_GATE_ENABLED=true.
const parseHourEnv = (value, fallback) => {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed >= 0 && parsed <= 24 ? parsed : fallback;
};

const OPERATING_HOURS_GATE_ENABLED =
  process.env.REACT_APP_OPERATING_HOURS_GATE_ENABLED === 'true';
const OPERATING_HOURS_START = parseHourEnv(process.env.REACT_APP_OPERATING_HOURS_START, 3);
const OPERATING_HOURS_END = parseHourEnv(process.env.REACT_APP_OPERATING_HOURS_END, 23);
// HJ_261105 - How long the measured "stopped, load the robot" cue stays before ordinary standby.
export const CARGO_READY_NOTICE_MS = 3000;

// HH_261024 - Guest screens have no inactivity deadline; explicit navigation
// and completed missions return home without altering motion watchdogs.

// HH_260721 - Mirror the platform-neutral service lifecycle used by backend and control.
const SERVICE_STATE = Object.freeze({
  DROP_ZONE_WAIT: 0,
  MOVING_TO_SITE: 1,
  SITE_ARRIVED: 2,
  RETURNING_TO_DROP_ZONE: 3,
  GUEST_RECALL_SERVICE: 4,
  SITE_ENTRY: 5,
  UNLOAD_WAIT: 6,
  RECALL_TO_SITE_ROAD: 7,
  GUEST_LOADING_WAIT: 8,
  RETURN_WITH_CARGO: 9,
  DROP_ZONE_PARKING: 10,
  // HH_260721 - Mirror explicit wait, charging, and departure states from avg_msgs.
  WAITING_FOR_RETURN_REQUEST: 11,
  WAITING_FOR_CHARGING: 12,
  CHARGING: 13,
  DEPARTING_CHARGER: 14,
  DEPARTING_DROP_ZONE: 15,
  OPERATOR_STOPPED: 16,
});
const SERVICE_STATE_NAME_BY_ID = Object.freeze(
  Object.fromEntries(Object.entries(SERVICE_STATE).map(([name, id]) => [id, name]))
);
// HH_260730 - Manual RViz and regulated UI goals share one runtime vocabulary.
const MISSION_PHASE_LABELS = Object.freeze({
  INITIALIZING: '시스템 준비 중',
  READY: '주행 준비 완료',
  GOAL_RECEIVED: '목적지 접수',
  PATH_PREPARING: '경로 준비 중',
  DRIVING: '주행 중',
  SAFETY_STOP: '안전 정지',
  ARRIVED: '도착',
  STOPPED: '운행 정지',
});
const SYSTEM_HEALTH_LABELS = Object.freeze({
  STARTING: '시스템 시작 중',
  OK: '시스템 정상',
  WARNING: '시스템 경고',
  ERROR: '시스템 오류',
});
// HH_260908 - The guest-recall overlays announce the ride TO the guest.
// Any state outside the en-route phase (arrival, loading wait, return,
// standby, stop) must drop them so they never cover the mission UI.
const GUEST_RECALL_ENROUTE_STATES = new Set([
  SERVICE_STATE.MOVING_TO_SITE,
  SERVICE_STATE.GUEST_RECALL_SERVICE,
  SERVICE_STATE.RECALL_TO_SITE_ROAD,
  SERVICE_STATE.DEPARTING_CHARGER,
  SERVICE_STATE.DEPARTING_DROP_ZONE,
]);
const ARRIVAL_STATES = new Set([
  SERVICE_STATE.SITE_ARRIVED,
  SERVICE_STATE.UNLOAD_WAIT,
  SERVICE_STATE.GUEST_LOADING_WAIT,
  SERVICE_STATE.WAITING_FOR_RETURN_REQUEST,
]);
const RETURNING_STATES = new Set([
  SERVICE_STATE.RETURNING_TO_DROP_ZONE,
  SERVICE_STATE.RETURN_WITH_CARGO,
  SERVICE_STATE.DROP_ZONE_PARKING,
  SERVICE_STATE.WAITING_FOR_CHARGING,
]);
const RETURN_COMPLETION_SOURCE_STATES = new Set([
  ...RETURNING_STATES,
  SERVICE_STATE.CHARGING,
]);
const MOVING_SERVICE_STATES = new Set([
  SERVICE_STATE.MOVING_TO_SITE,
  SERVICE_STATE.SITE_ENTRY,
  SERVICE_STATE.RECALL_TO_SITE_ROAD,
  SERVICE_STATE.DEPARTING_CHARGER,
  SERVICE_STATE.DEPARTING_DROP_ZONE,
]);
const ACTIVE_MANUAL_PHASES = new Set([
  'GOAL_RECEIVED',
  'PATH_PREPARING',
  'DRIVING',
  'SAFETY_STOP',
]);
const URGENT_BATTERY_RETURN_PERCENT = 25;
const MISSION_DISPATCH_MINIMUM_PERCENT = 35;

// HH_260907 - This is presentation only: the controller owns clearance dwell,
// site re-entry and rotation. B11-B13 retain their existing opposite-side route.
function recallReturnInstructions(site, finalReady = false) {
  // HH_261102 - Explain only the current action while preserving the two separate recall confirmations.
  if (/^B(?:[1-9]|10)$/.test(String(site || ''))) {
    if (finalReady) return '짐을 모두 실었다면 복귀를 눌러 주세요.';
    return '사이트 안으로 들어가 180도 회전합니다. 주변을 비워 주세요. 정차 후 짐을 실어 주세요.';
  }
  return '짐을 싣고 주변을 비워 주세요. 반대쪽 도로로 복귀합니다.';
}

function recallCompletionLabel(site, finalReady = false) {
  return /^B(?:[1-9]|10)$/.test(String(site || ''))
    ? (finalReady ? '짐 싣기 완료 · 복귀' : '정리 완료 · 회전 시작')
    : '짐 싣기 완료 · 복귀';
}

function missionExecutionFailureMessage(reason) {
  // HH_261102 - Preserve raw failure evidence separately instead of filling the customer screen with controller fields.
  return reason ? '이동할 수 없습니다. 관리자에게 문의해 주세요.' : '';
}

function recallReturnProgress(site, description) {
  const parts = String(description || '').split(':');
  const phase = parts[0] === 'camping_site_maneuver_controller' ? parts[1] : '';
  if (/^B(?:[1-9]|10)$/.test(String(site || ''))) {
    const stages = {
      RECALL_CLEARANCE_WAIT: ['사이트 비움 안내 중', '음성 안내 후 사이트로 다시 진입합니다. 사이트와 로봇 주변을 비워주세요.'],
      ALIGN_ENTRY_YAW: ['사이트 재진입 준비', '사이트 안으로 들어가기 위해 방향을 맞추고 있습니다.'],
      CRAB_IN: ['사이트 재진입 중', '전진 복귀를 위해 사이트 안으로 이동하고 있습니다.'],
      ROTATE_180: ['사이트 안에서 180도 회전 중', '로봇이 회전하고 있습니다. 사이트와 로봇 주변에서 떨어져 주세요.'],
      RECALL_RETURN_WAIT: ['짐 싣기 완료 확인 대기', '회전을 마치고 정차했습니다. 짐 싣기 완료 버튼을 누르기 전에는 출차하지 않습니다.'],
      ALIGN_RETRACE_YAW: ['도로 출차 준비', '회전을 마치고 도로로 나갈 방향을 맞추고 있습니다.'],
      CRAB_OUT: ['사이트에서 도로로 출차 중', '도로로 나온 뒤 대기·충전 장소까지 전진으로 복귀합니다.'],
    };
    if (stages[phase]) return { label: stages[phase][0], message: stages[phase][1] };
  }
  return {
    label: '짐을 싣고 복귀 중',
    message: /^B(?:11|12|13)$/.test(String(site || ''))
      ? '사이트로 재진입하지 않고 기존 반대쪽 경로로 대기·충전 장소에 복귀 중입니다.'
      : '이용객의 짐을 싣고 대기·충전 장소로 복귀 중입니다.',
  };
}

// Presentation only: a service identity can remain active during a safety
// hold. Do not describe that retained mission as currently moving. Keep its
// site/arrival/confirmation context and all command permissions unchanged.
function serviceMotionNotice(missionPhase, systemHealth) {
  if (missionPhase === 'SAFETY_STOP') {
    return { label: '안전 정지', message: '안전 조건으로 주행이 일시 정지되었습니다. 상태를 확인해주세요.' };
  }
  if (missionPhase === 'STOPPED') {
    return { label: '운행 정지', message: '운행이 정지되었습니다.' };
  }
  if (missionPhase === 'ERROR' || systemHealth === 'ERROR') {
    // A diagnostic ERROR alone does not prove zero physical speed.
    return { label: '시스템 오류', message: '시스템 오류가 있습니다. 현재 주행 상태와 진단 내용을 확인해주세요.' };
  }
  return null;
}

// Guest call notifications are admission information, never modal authority.
// Once progress/arrival/safety has its own screen, stale notification flags
// must not cover Stop, loading confirmation, or an error explanation.
function guestAdmissionNotice({ dispatch, showRecall, noticeSite, missionPhase,
  serviceStateName, systemHealth, arrivalVisible, arrivedSite, executionError }) {
  if ((!showRecall && !noticeSite) || !dispatch?.active || dispatch.owner !== 'guest'
      || !Number.isSafeInteger(dispatch.generation) || dispatch.generation <= 0
      || !/^B(?:[1-9]|1[0-3])$/.test(String(dispatch.site || ''))
      || !['delivery', 'recall'].includes(dispatch.intent)
      || (noticeSite && noticeSite !== dispatch.site)) return '';
  if (arrivalVisible || arrivedSite || executionError || systemHealth === 'ERROR'
      || !['READY', 'GOAL_RECEIVED'].includes(missionPhase)
      || !['DROP_ZONE_WAIT', 'CHARGING', 'WAITING_FOR_CHARGING'].includes(serviceStateName)) return '';
  return `${dispatch.site} 사이트 호출을 접수했습니다. 출발을 준비합니다.`;
}

function robotCanCompleteMission(dispatch, site, serviceStateName) {
  if (!dispatch.active || !site || dispatch.site !== site
      || !Number.isSafeInteger(dispatch.generation) || dispatch.generation <= 0) return false;
  if (['operator', 'robot'].includes(dispatch.owner)) return true;
  // Physical Robot UI may complete the currently admitted Guest recall, but
  // never acquire its ownership or complete a delivery/other mission phase.
  return dispatch.owner === 'guest' && dispatch.intent === 'recall'
    && serviceStateName === 'GUEST_LOADING_WAIT';
}

// HH_260904 - Re-dock events can arrive after the HTTP command response and
// can be replayed as a websocket snapshot. Keep terminal states distinct so a
// false wait flag never implies that CAN recovered (expiry/cancel are false too).
const REDOCK_STATUS_MESSAGES = Object.freeze({
  idle: '',
  waiting_for_disconnect: '충전 접점 해제 확인 후 자동으로 재도킹합니다',
  alignment_preparing: '재도킹 정렬을 준비합니다',
  waiting_for_can: '재도킹 대기 중 · 리모컨을 CAN 모드로 전환하세요',
  can_restored: 'CAN 제어 복구됨 · 재도킹 정렬을 시작합니다',
  alignment_started: '재도킹 정렬을 시작합니다',
  expired: '재도킹 요청 시간이 만료되었습니다 · 복귀 버튼을 다시 눌러주세요',
  cancelled: '재도킹 요청이 취소되었습니다',
});

const emptyBatteryReturnState = () => ({
  pending: false,
  started: false,
  waitingForUser: false,
  urgent: false,
});

const formatMissionBlockMessage = (data) => {
  const minimum = Number.isFinite(Number(data.minimum_battery_percentage))
    ? Number(data.minimum_battery_percentage)
    : 35;
  const battery = Number(data.battery_percentage);
  if (Number.isFinite(battery) && battery >= 0) {
    return `배터리 잔량이 ${battery}%입니다. 새 사이트 이동은 ${minimum}% 이상에서만 가능합니다.`;
  }
  return `배터리 상태를 아직 받지 못했습니다. 새 사이트 이동은 ${minimum}% 이상 확인 후 가능합니다.`;
};

const formatBatteryReturnMessage = (data) => {
  const minimum = Number.isFinite(Number(data.minimum_battery_percentage))
    ? Number(data.minimum_battery_percentage)
    : 35;
  const battery = Number(data.battery_percentage);
  const prefix = Number.isFinite(battery) && battery >= 0
    ? `배터리 잔량이 ${battery}%입니다.`
    : '배터리 잔량이 낮습니다.';
  if (data.battery_return_urgent) {
    return `${prefix} 25% 미만으로 현재 작업을 중단하고 충전 도킹을 위해 즉시 복귀합니다. 로봇 주변을 비워주세요.`;
  }
  if (data.battery_return_started) {
    return `${prefix} 복귀 요청이 확인되어 충전 구역으로 이동합니다.`;
  }
  if (data.battery_return_waiting_for_user) {
    return `${prefix} 짐 처리를 마친 뒤 이용 완료 버튼을 눌러주세요. 완료 후 충전 도킹을 위해 복귀합니다.`;
  }
  return `${prefix} ${minimum}% 미만이면 새 임무를 받지 않고, 현재 임무 완료 후 충전 구역 복귀를 대기합니다.`;
};

const batteryPolicyStatus = (batteryPct, batteryReturnState) => {
  // HH_260724 - Keep the battery policy visible after the modal is dismissed.
  const battery = batteryPct == null ? NaN : Number(batteryPct);
  const batteryText = Number.isFinite(battery) && battery >= 0 ? `${battery}%` : '';
  if (batteryReturnState.urgent) {
    return { tone: 'error', label: `배터리 부족 · 즉시 복귀 ${batteryText}`.trim() };
  }
  if (batteryReturnState.started) {
    return { tone: 'warning', label: `배터리 부족 복귀 중 ${batteryText}`.trim() };
  }
  if (batteryReturnState.waitingForUser) {
    return { tone: 'warning', label: `이용 완료 후 복귀 대기 ${batteryText}`.trim() };
  }
  if (batteryReturnState.pending) {
    return { tone: 'warning', label: `임무 완료 후 복귀 ${batteryText}`.trim() };
  }
  if (!Number.isFinite(battery) || battery < 0) {
    return { tone: 'warning', label: '배터리 상태 확인 중' };
  }
  if (battery < URGENT_BATTERY_RETURN_PERCENT) {
    return { tone: 'error', label: `25% 미만 · 긴급 복귀 필요 (${battery}%)` };
  }
  if (battery < MISSION_DISPATCH_MINIMUM_PERCENT) {
    return { tone: 'warning', label: `${MISSION_DISPATCH_MINIMUM_PERCENT}% 미만 · 새 임무 대기 (${battery}%)` };
  }
  return { tone: 'ok', label: `임무 가능 · 배터리 ${battery}%` };
};

const parkingLifecycleStatus = (serviceStateName, serviceStateDescription, parkingPolicy, batteryChargeComplete = false) => {
  const state = String(serviceStateName || '').trim().toUpperCase();
  const description = String(serviceStateDescription || '').trim().toUpperCase();
  const selectedMethod = String(parkingPolicy?.parking_selected_method || '').trim().toLowerCase();

  // HH_261002 - A confirmed BMS full signal is distinct from charger contact.
  if (state === 'CHARGING') return batteryChargeComplete ? '충전 완료' : '충전 중';
  if (state === 'WAITING_FOR_CHARGING') return '충전 접점 연결 대기 중';
  if (state === 'DROP_ZONE_WAIT') return '대기·충전 장소 주차 완료';
  if (state === 'DROP_ZONE_PARKING') {
    if (
      description.includes('DROP_ZONE_MANEUVER_CONTROLLER')
      || description.includes('PARKING_APPROACH')
      || description.includes('ALIGN_FOR_PARKING')
    ) return '대기·충전 장소 주차 중';
    if (selectedMethod === 'apriltag') return '충전 도킹 중';
    if (selectedMethod === 'reverse') return '후진 주차 중';
    return '대기·충전 장소 주차 중';
  }

  if (selectedMethod === 'apriltag') return '충전 도킹 선택됨';
  if (selectedMethod === 'reverse') return '일반 후진 주차 선택됨';
  return parkingPolicy?.charging_required
    ? '충전 도킹 필요'
    : '자동 주차 · 배터리 35% 기준';
};

function WaitingRuntimeStatusPanel({
  systemHealth,
  missionPhase,
  batteryPolicy,
  parkingPolicy,
  serviceStateName,
  serviceStateDescription,
  batteryChargeComplete,
}) {
  const t = useT();
  const items = [
    {
      key: 'system',
      label: '시스템',
      value: SYSTEM_HEALTH_LABELS[systemHealth] || SYSTEM_HEALTH_LABELS.STARTING,
      tone: `health-${systemHealth.toLowerCase()}`,
    },
    {
      key: 'mission',
      label: '임무',
      value: MISSION_PHASE_LABELS[missionPhase] || MISSION_PHASE_LABELS.INITIALIZING,
      tone: 'mission',
    },
    {
      key: 'parking',
      label: '주차',
      value: parkingLifecycleStatus(serviceStateName, serviceStateDescription, parkingPolicy, batteryChargeComplete),
      tone: 'parking',
      title: parkingPolicyMessage(parkingPolicy),
    },
    {
      key: 'battery',
      label: '배터리',
      value: batteryPolicy?.label || '배터리 상태 확인 중',
      tone: `battery-${batteryPolicy?.tone || 'warning'}`,
    },
  ];

  return (
    <section className="waiting-runtime-panel" aria-label={t('로봇 운행 상태')} aria-live="polite">
      {items.map(item => (
        <div key={item.key} className={`waiting-runtime-item ${item.tone}`} title={t(item.title || item.value)}>
          <span className="waiting-runtime-dot" />
          <span className="waiting-runtime-copy">
            <small>{t(item.label)}</small>
            <strong>{t(item.value)}</strong>
          </span>
        </div>
      ))}
    </section>
  );
}

// HH_260804 - Keep one-second clock updates local so the rest of the operator
// screen does not re-render whenever only the displayed time has changed.
function LiveClock() {
  const [currentTime, setCurrentTime] = useState(new Date());
  // HJ_261118 - 날짜·시각 표기도 선택한 언어를 따른다.
  const locale = localeOf(useLang());

  // HJ_261118 - Minutes only; re-render once per minute boundary, not every second.
  useEffect(() => {
    let timer;
    const tick = () => {
      const now = new Date();
      setCurrentTime(now);
      timer = setTimeout(tick, 60000 - (now.getSeconds() * 1000 + now.getMilliseconds()) + 50);
    };
    tick();
    return () => clearTimeout(timer);
  }, []);

  const dateStr = currentTime.toLocaleDateString(locale, {
    year: 'numeric', month: locale === 'ko-KR' ? 'long' : 'short', day: 'numeric', weekday: 'short'
  });
  const timeStr = currentTime.toLocaleTimeString(locale, {
    hour: '2-digit', minute: '2-digit'
  });

  return (
    <div className="wh-right">
      <span className="wh-date">{dateStr}</span>
      <span className="wh-time">{timeStr}</span>
    </div>
  );
}

// ── 탐방로 공통 이미지 캐러셀 컴포넌트 ──────────────────────────────────────
function TrailCarousel({ title, images }) {
  const [open, setOpen] = useState(false);
  const [idx, setIdx] = useState(0);

  const prev = (e) => {
    e.stopPropagation();
    setIdx(i => (i - 1 + images.length) % images.length);
  };
  const next = (e) => {
    e.stopPropagation();
    setIdx(i => (i + 1) % images.length);
  };

  return (
    <>
      <button
        className="trail-more-btn"
        onClick={e => { e.stopPropagation(); setIdx(0); setOpen(true); }}
      >
        <T>더보기</T>
      </button>
      {open && (
        <div className="carousel-overlay" onClick={() => setOpen(false)}>
          <div className="carousel-box" onClick={e => e.stopPropagation()}>
            <button className="carousel-close-btn" onClick={() => setOpen(false)}><T>뒤로가기</T></button>
            <div className="carousel-title">{title}</div>
            <div className="carousel-img-wrap">
              <button className="carousel-arrow carousel-arrow-left" onClick={prev}>&#8249;</button>
              <img
                src={`/${encodeURIComponent(images[idx])}`}
                alt={`${title} ${idx + 1}`}
                className="carousel-img"
              />
              <button className="carousel-arrow carousel-arrow-right" onClick={next}>&#8250;</button>
            </div>
            <div className="carousel-counter">{idx + 1} / {images.length}</div>
          </div>
        </div>
      )}
    </>
  );
}

// ── 진단 모니터 컴포넌트 ──────────────────────────────────────────────────────
export function DiagnosticsMonitor({
  adminLoginId = '',
  adminLoginPw = '',
  redockStatus = null,
  parkingPolicy = {},
  serviceStateName = '',
  engageState = false,
  engageDisabled = false,
  onToggleEngage = null,
  headlightState = false,
  onToggleHeadlight = null,
}) {
  const [activeTab, setActiveTab] = useState('operation');
  const [items, setItems] = useState([]);
  const [selected, setSelected] = useState(null);
  const [expanded, setExpanded] = useState({ error: true, warn: true, ok: false });
  // HH_260907 - Return uses SOC-selected parking; the separate Dock button
  // explicitly requests charging through the same backend motion authority.
  const [motionCommandPending, setMotionCommandPending] = useState('');
  const [motionCommandStatus, setMotionCommandStatus] = useState('');
  // Display only a value acknowledged by the parameter service. Slider drafts
  // are not applied values; failed requests must not imply hardware support.
  const [steeringRate, setSteeringRate] = useState(null);
  const [steeringDraftRate, setSteeringDraftRate] = useState(null);
  const [steeringTuningAvailable, setSteeringTuningAvailable] = useState(false);
  const [steeringTuningPending, setSteeringTuningPending] = useState(false);
  const [steeringTuningStatus, setSteeringTuningStatus] = useState('설정 서비스 응답 대기');
  const steeringTuningTimerRef = useRef(null);
  const steeringTuningRef = useRef({
    active: false, available: false, pending: false, request: 0, rate: null,
  });

  const requestSteeringTuning = async (nextRate) => {
    const current = steeringTuningRef.current;
    const updating = nextRate !== undefined;
    const validRate = value => typeof value === 'number'
      && Number.isFinite(value) && value >= 0.05 && value <= 2.0;
    // Recheck at dispatch time, not only in the disabled DOM control: a queued
    // debounce or duplicate event must not POST after availability is lost.
    if (!current.active || current.pending
        || (updating && (!current.available || !validRate(nextRate)))) return false;
    const request = ++current.request;
    current.pending = true;
    setSteeringTuningPending(true);
    setSteeringTuningStatus(updating ? '적용 확인 중…' : '설정 서비스 응답 대기');
    const isCurrent = () => current.active && current.request === request;
    try {
      const response = await fetch(
        updating
          ? `/ui/platform_tuning?steering_transition_rate_radps=${encodeURIComponent(nextRate.toFixed(2))}`
          : '/ui/platform_tuning',
        updating ? { method: 'POST' } : undefined
      );
      const body = await response.json();
      if (!isCurrent()) return false;
      const rate = body.steering_transition_rate_radps;
      if (!response.ok || body.success !== true || body.available !== true || !validRate(rate)) {
        throw new Error('설정 서비스 응답을 확인할 수 없습니다');
      }
      current.rate = rate;
      current.available = true;
      setSteeringRate(rate);
      setSteeringDraftRate(rate);
      setSteeringTuningAvailable(true);
      setSteeringTuningStatus(updating ? '적용 확인됨' : '런타임 조절 가능');
      return true;
    } catch (error) {
      if (!isCurrent()) return false;
      current.available = false;
      setSteeringTuningAvailable(false);
      // Retain the last acknowledged rate internally, but hide it while the
      // service is unavailable; never promote the rejected draft to reality.
      setSteeringDraftRate(current.rate);
      setSteeringTuningStatus('설정 서비스 미가용 · 진단 화면을 다시 열어 확인하세요');
      return false;
    } finally {
      if (isCurrent()) {
        current.pending = false;
        setSteeringTuningPending(false);
      }
    }
  };

  useEffect(() => {
    if (!redockStatus?.received) return;
    setMotionCommandStatus(redockStatus.message || '');
  }, [redockStatus]);

  useEffect(() => {
    if (activeTab !== 'system') return undefined;
    const fetch_ = () =>
      fetch('/api/diagnostics')
        .then(r => r.json())
        .then(j => setItems(j.status || []))
        .catch(() => {});
    fetch_();
    // HH_260810 - System diagnostics remain a small HTTP payload, but refresh
    // promptly only while visible; sensor-heavy tabs use their dedicated WS.
    const t = setInterval(fetch_, 500);
    return () => clearInterval(t);
  }, [activeTab]);

  useEffect(() => {
    const current = steeringTuningRef.current;
    current.active = true;
    current.available = false;
    current.pending = false;
    setSteeringTuningAvailable(false);
    requestSteeringTuning();
    return () => {
      current.active = false;
      ++current.request;
      if (steeringTuningTimerRef.current) {
        clearTimeout(steeringTuningTimerRef.current);
        steeringTuningTimerRef.current = null;
      }
    };
  }, []);

  const handleSteeringRateChange = (event) => {
    const current = steeringTuningRef.current;
    if (!current.active || !current.available || current.pending) return;
    const nextRate = Number(event.target.value);
    if (!Number.isFinite(nextRate) || nextRate < 0.05 || nextRate > 2.0) return;
    setSteeringDraftRate(nextRate);
    setSteeringTuningStatus(`변경값 ${nextRate.toFixed(2)} rad/s · 적용 대기`);
    if (steeringTuningTimerRef.current) {
      clearTimeout(steeringTuningTimerRef.current);
    }
    steeringTuningTimerRef.current = setTimeout(
      () => {
        steeringTuningTimerRef.current = null;
        requestSteeringTuning(nextRate);
      },
      300
    );
  };

  const requestManualReturn = async () => {
    setMotionCommandPending('return');
    setMotionCommandStatus('');
    try {
      const response = await fetch('/ui/manual_return', { method: 'POST' });
      const body = await response.json();
      if (!response.ok || !body.success) throw new Error(body.message || '복귀 요청 실패');
      const statusByAction = {
        parking_alignment: '선택된 주차 방식으로 정렬을 시작합니다',
        parking_alignment_waiting_for_can: '재도킹 대기 중 · 리모컨을 CAN 모드로 전환하세요',
        waiting_for_disconnect: '충전 접점 해제 확인 후 자동으로 재도킹합니다',
        parking_in_progress: '주차가 이미 진행 중입니다',
        return_in_progress: '복귀가 이미 진행 중입니다',
      };
      setMotionCommandStatus(statusByAction[body.action] || '복귀 명령 전송됨');
    } catch (error) {
      setMotionCommandStatus(error.message || '복귀 요청 실패');
    } finally {
      setMotionCommandPending('');
    }
  };

  // HH_260721 - Operator diagnostics use one unambiguous three-level health scale.
  const LEVEL_STR  = { 0: 'OK', 1: 'WARN', 2: 'ERROR' };
  const DOT_COLOR  = { 0: '#5dca5d', 1: '#d4a030', 2: '#e24b4a' };
  // HH_261026 - Keep severity colors readable against the existing light surfaces.
  const HEAD_COLOR = { 0: '#287141', 1: '#91680d', 2: '#b13d36' };

  const groups = [
    { key: 'error', label: 'Error',  lvls: [2],    items: items.filter(i => i.level === 2) },
    { key: 'warn',  label: 'Warn',   lvls: [1],    items: items.filter(i => i.level === 1) },
    { key: 'ok',    label: 'OK',     lvls: [0],    items: items.filter(i => i.level === 0) },
  ].filter(g => g.items.length > 0);
  // HH_261026 - Navigation changes only the visible existing view; commands stay explicit.
  const activeView = getAdminDiagnosticView(activeTab);

  return (
    <div className="diag-monitor-wrap admin-workspace">
      <AdminNavigation
        activeTab={activeTab}
        tabs={[
          { id: 'operation', label: '운영 제어' },
          { id: 'system', label: '시스템 진단' },
          ...TELEMETRY_TABS,
          { id: 'snapshot', label: '스냅샷' },
        ]}
        renderTab={tab => (
          <button
            key={tab.id}
            type="button"
            role="tab"
            id={`operator-diagnostic-tab-${tab.id}`}
            data-ui={`operator-diagnostic-tab-${tab.id}`}
            className={`diag-tab-button ${activeTab === tab.id ? 'active' : ''}`}
            onClick={() => setActiveTab(tab.id)}
            aria-selected={activeTab === tab.id}
            aria-controls="admin-diagnostic-view"
          >
            {tab.label}
          </button>
        )}
      />
      <section className="admin-workspace-main" id="admin-diagnostic-view" role="tabpanel"
        aria-labelledby={`operator-diagnostic-tab-${activeTab}`}>
        <header className="admin-view-heading">
          <span className="admin-view-category">{activeView.category}</span>
          <h2>{activeView.title}</h2>
          <p>{activeView.help}</p>
        </header>
      {/* HJ_261118 - 운영 제어(복귀·충전, 전조등, 조향, bringup 종료)는 별도 탭, 시스템 진단은 장치 상태만. */}
      {activeTab === 'operation' ? (
        <div className="admin-system-view admin-operation-view">
          <section className="admin-system-controls" aria-labelledby="admin-system-controls-title">
            <div className="admin-section-heading">
              <h3 id="admin-system-controls-title">운영 제어</h3>
              <p>복귀·충전, 전조등과 조향 설정을 조작합니다. 장치 상태는 진단 › 시스템 진단에서 확인하세요.</p>
            </div>
            <div className="admin-system-command-grid">
      {/* ── 상단 컨트롤 바 ── */}
      <div className="diag-control-bar">
        <span className="diag-control-label"><strong>복귀·충전</strong><small>Manual Motion</small></span>
        <button
          type="button"
          className="manual-return-btn"
          onClick={requestManualReturn}
          disabled={Boolean(motionCommandPending)}
          title="현재 서비스 상태와 관계없이 drop zone 복귀를 요청"
        >
          {motionCommandPending === 'return' ? '복귀 요청 중' : '복귀 · 자동 주차'}
        </button>
        <DockingCommandButton disabled={Boolean(motionCommandPending)} serviceStateName={serviceStateName} />
        <span className="manual-motion-status">{motionCommandStatus || '명령 대기'}</span>
      </div>

      {/* ── 전조등 컨트롤 패널 ── */}
      {onToggleHeadlight && (
        <div className="diag-control-bar light-control-bar">
          <span className="diag-control-label"><strong>전조등</strong><small>Light Control</small></span>
          <button
            type="button"
            className={`manual-return-btn headlight-command-btn ${headlightState ? 'headlight-on' : ''}`}
            onClick={onToggleHeadlight}
            title="전조등을 켜거나 끕니다"
          >
            {headlightState ? 'LIGHT OFF' : 'LIGHT ON'}
          </button>
          <span className="manual-motion-status" role="status">
            {headlightState ? '전조등 켜짐' : '전조등 꺼짐'}
          </span>
        </div>
      )}
            </div>

      <div className="steering-tuning-card">
        <div className="steering-tuning-copy">
          <div className="steering-tuning-title">횡↔종 조향 전환 속도</div>
          <div className="steering-tuning-help">
            바퀴 방향이 종방향과 횡방향 사이에서 회전하는 최대 속도입니다.
            설정 서비스 연결 시 재시작 없이 조절할 수 있으며, 아래에는 확인된 적용값을 표시합니다.
          </div>
          <div className={`steering-tuning-status ${steeringTuningAvailable ? 'available' : ''}`}>
            {steeringTuningStatus}
          </div>
        </div>
        <div className="steering-tuning-control">
          <div className="steering-tuning-value">
            <strong>{steeringTuningAvailable && steeringRate !== null ? steeringRate.toFixed(2) : '—'}</strong> rad/s
            <span>{steeringTuningAvailable && steeringRate !== null ? `${Math.round(steeringRate * 100)}%` : '—'}</span>
          </div>
          <input
            className="steering-tuning-slider"
            type="range"
            min="0.05"
            max="2.0"
            step="0.05"
            value={steeringDraftRate ?? 0.05}
            disabled={!steeringTuningAvailable || steeringTuningPending}
            onChange={handleSteeringRateChange}
            aria-label="횡 종 조향 전환 속도"
          />
          <div className="steering-tuning-scale">
            <span>느림 0.05</span>
            <span>기준 1.00</span>
            <span>빠름 2.00</span>
          </div>
        </div>
      </div>
          </section>
          <BringupShutdownControl username={adminLoginId} password={adminLoginPw} />
        </div>
      ) : activeTab === 'system' ? (
        <div className="admin-system-view">

      {/* HH_261026 - Collapsing details is presentation-only and leaves refresh policy unchanged. */}
      <details className="admin-diagnostic-details" open>
        <summary>
          <strong>상태 진단 · 세부 정보</strong>
          <span className="admin-diagnostic-counts">
            <span className={groups.some(g => g.key === 'error') ? 'has-error' : ''}>오류 {items.filter(i => i.level === 2).length}</span>
            <span className={groups.some(g => g.key === 'warn') ? 'has-warn' : ''}>경고 {items.filter(i => i.level === 1).length}</span>
            <span>정상 {items.filter(i => i.level === 0).length}</span>
          </span>
        </summary>
    <div className="diag-monitor">
      {/* ── 왼쪽: 트리 패널 ── */}
      <div className="diag-tree">
        <div className="diag-tree-header">
          장치 상태
          <span className="diag-count">{items.length}개 항목</span>
        </div>
        {groups.map(g => {
          const lvl = g.lvls[0];
          const isOpen = expanded[g.key];
          return (
            <div key={g.key}>
              <button
                type="button"
                className="diag-group-header"
                style={{ color: HEAD_COLOR[lvl] }}
                onClick={() => setExpanded(e => ({ ...e, [g.key]: !e[g.key] }))}
                aria-expanded={isOpen}
              >
                <span className="diag-arrow">{isOpen ? '▼' : '▶'}</span>
                {g.label} ({g.items.length})
              </button>
              {isOpen && g.items.map((item, i) => (
                <button
                  type="button"
                  key={i}
                  className={`diag-item ${selected === item ? 'diag-item-selected' : ''}`}
                  onClick={() => setSelected(item)}
                  aria-pressed={selected === item}
                >
                  <span className="diag-dot" style={{ background: DOT_COLOR[item.level] }} />
                  {item.name}
                </button>
              ))}
            </div>
          );
        })}
        {items.length === 0 && (
          /* HH_260617 - UI diagnostics reads the namespaced system aggregator topic. */
          <div className="diag-empty">데이터 없음 — /system/diagnostics_agg 대기 중…</div>
        )}
      </div>

      {/* ── 오른쪽: 상세 패널 ── */}
      <div className="diag-detail">
        {selected ? (
          <>
            <div className="diag-detail-header">상세 정보 — {selected.name}</div>
            <div className="diag-detail-status">
              <span className="diag-dot lg" style={{ background: DOT_COLOR[selected.level] }} />
              <span style={{ color: HEAD_COLOR[selected.level], fontWeight: 600 }}>
                {LEVEL_STR[selected.level]} (level {selected.level})
              </span>
            </div>
            <div className="diag-detail-label">메시지</div>
            <div className="diag-detail-msg" style={{ borderColor: DOT_COLOR[selected.level] }}>
              {selected.message || '—'}
            </div>
            {selected.values && selected.values.length > 0 && (
              <>
                <div className="diag-detail-label" style={{ marginTop: '0.8rem' }}>세부 값</div>
                <table className="diag-kv-table">
                  <tbody>
                    {selected.values.map((kv, i) => (
                      <tr key={i}>
                        <td className="diag-kv-key">{kv.key}</td>
                        <td className="diag-kv-val">{kv.value}</td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </>
            )}
          </>
        ) : (
          <div className="diag-empty">← 항목을 선택하세요</div>
        )}
      </div>
    </div>
      </details>
        </div>
      ) : activeTab === 'snapshot' ? (
        <div className="snapshot-tab-view">
          <SnapshotControl />
        </div>
      ) : (
        <TelemetryWorkspace
          activeTab={activeTab}
          adminLoginId={adminLoginId}
          adminLoginPw={adminLoginPw}
          redockStatus={redockStatus}
          parkingPolicy={parkingPolicy}
          serviceStateName={serviceStateName}
          engageState={engageState}
          engageDisabled={engageDisabled}
          onToggleEngage={onToggleEngage}
        />
      )}
      </section>
    </div>
  );
}

// ── 시설 안내 콘텐츠 컴포넌트 (영지 구성 위치 안내 포함) ───────────────────
function FacilityContent() {
  const [zoneMap, setZoneMap] = useState(null); // 'A'~'F' or null

  const zones = [
    { zone: 'A', count: '17동', note: '무공해 영지(A1~A6) 포함' },
    { zone: 'B', count: '13동', note: '' },
    { zone: 'C', count: '32동', note: '' },
    { zone: 'D', count: '25동', note: '' },
    { zone: 'E', count: '18동', note: '' },
    { zone: 'F', count: '8동', note: '' },
  ];

  return (
    <div className="facility-wrap">

      {/* ── 헤더 배너 ── */}
      <div className="park-banner">
        <div className="park-banner-content">
          <div className="park-banner-logo">
            <img src="/월악산_국립공원_로고.jpg" alt="국립공원관리공단 로고" />
          </div>
          <div className="park-banner-text">
            <div className="park-banner-label">Korea National Park Service · Woraksan National Park</div>
            <div className="park-banner-breadcrumb">
              <span><T>월악산 국립공원</T></span>
              <span className="park-banner-sep">|</span>
              <span><T>닷돈재 야영장</T></span>
              <span className="park-banner-sep">|</span>
              <span className="park-banner-current"><T>시설 안내</T></span>
            </div>
          </div>
        </div>
      </div>

      {/* ── 기본 정보 카드 ── */}
      <div className="facility-section">
        <div className="facility-section-title"><T>📋 기본 정보</T></div>
        <div className="facility-info-grid">
          <div className="facility-info-item">
            <span className="fi-icon">📍</span>
            <div>
              <div className="fi-label"><T>주소</T></div>
              <div className="fi-value"><T>충북 제천시 한수면 송계리 70-2</T></div>
            </div>
          </div>
          <div className="facility-info-item">
            <span className="fi-icon">📞</span>
            <div>
              <div className="fi-label"><T>문의처</T></div>
              <div className="fi-value">043-653-3252</div>
            </div>
          </div>
          <div className="facility-info-item">
            <span className="fi-icon">🕐</span>
            <div>
              <div className="fi-label"><T>입 / 퇴실 시간</T></div>
              <div className="fi-value"><T>오후 2시 입실 · 익일 낮 12시(정오) 퇴실</T></div>
            </div>
          </div>
        </div>
      </div>

      {/* ── 이용 금액 ── */}
      <div className="facility-section">
        <div className="facility-section-title"><T>💰 이용 금액</T></div>
        <div className="facility-fee-table">
          <div className="fee-row fee-header">
            <span><T>구분</T></span>
            <span><T>주말·성수기</T></span>
            <span><T>주중</T></span>
          </div>
          <div className="fee-row">
            <span><T>🚗 자동차 야영지</T></span>
            <span className="fee-peak"><T>30,000원</T></span>
            <span className="fee-normal"><T>20,000원</T></span>
          </div>
        </div>
        <div className="facility-notice-small">
          <p><T>• 주중: 법정공휴일 전일 제외, 매주 일~목요일</T></p>
          <p><T>• 주말: 매주 금·토요일, 법정공휴일 전일</T></p>
          <p><T>• 성수기: 매해 7.1~8.31, 10.1~11.15</T></p>
        </div>
      </div>

      {/* ── 편의시설 ── */}
      <div className="facility-section">
        <div className="facility-section-title"><T>🏕️ 편의시설</T></div>
        <div className="facility-amenity-grid">
          {[
            { icon: '🚿', label: '샤워장', desc: '1동 · 09~13시 / 15~18시\n6분 1,000원, 추가 3분 500원' },
            { icon: '🚰', label: '개수대', desc: '4동' },
            { icon: '🚻', label: '화장실', desc: '2동' },
            { icon: '⚡', label: '전기', desc: '최대 1,300W' },
            { icon: '📶', label: 'Wi-Fi', desc: '미제공', unavailable: true },
            { icon: '🪑', label: '테이블', desc: '제공' },
            { icon: '❄️', label: '공용 냉장고', desc: '미제공', unavailable: true },
            { icon: '🔌', label: '전기차 충전소', desc: '이용 가능' },
          ].map((item, i) => (
            <div key={i} className={`amenity-chip ${item.unavailable ? 'unavailable' : ''}`}>
              <span className="amenity-icon">{item.icon}</span>
              <span className="amenity-label"><T>{item.label}</T></span>
              <span className="amenity-desc"><T>{item.desc}</T></span>
            </div>
          ))}
        </div>
      </div>

      {/* ── 영지 정보 ── */}
      <div className="facility-section">
        <div className="facility-section-title"><T>🗺️ 영지 구성</T></div>
        <div className="facility-zone-grid">
          {zones.map((z, i) => (
            <div key={i} className={`zone-chip ${z.note ? 'zone-special' : ''}`}>
              <div className="zone-chip-info">
                <span className="zone-name"><T>{`${z.zone}구역`}</T></span>
                <span className="zone-count"><T>{z.count}</T></span>
                {z.note && <span className="zone-note">🌿 <T>{z.note}</T></span>}
              </div>
              <button
                className="zone-map-btn"
                onClick={e => { e.stopPropagation(); setZoneMap(z.zone); }}
              >
                <T>위치 안내</T>
              </button>
            </div>
          ))}
        </div>
        <div className="facility-notice-small" style={{ marginTop: '0.7rem' }}>
          <p><T>• 영지 규격: 가로 5m × 세로 7m (마사토)</T></p>
          <p><T>• 총 113동 운영 (자동차 야영지)</T></p>
        </div>
      </div>

      {/* ── 운영 정책 안내 ── */}
      <div className="facility-policy">
        <div className="facility-section-title" style={{ marginBottom: '0.6rem' }}><T>⚠️ 운영 정책</T></div>
        <p><T>• 일부 </T><strong><T>무공해 영지</T></strong><T>를 지정 운영합니다. 전기·수소차, 대중교통·자전거·도보 이용객만 예약 가능합니다.</T></p>
        <p><T>• 운영정책 위반에 따른 책임은 예약자에게 있으니 예약 시 세부 사항을 꼼꼼히 확인해주세요.</T></p>
      </div>

      {/* ── 구역 배치도 오버레이 ── */}
      {zoneMap && (
        <div className="zone-map-overlay">
          <div className="zone-map-box">
            <button className="zone-map-back-btn" onClick={() => setZoneMap(null)}><T>뒤로가기</T></button>
            <img
              src={`/월악산_배치도_${zoneMap}.png`}
              alt={`${zoneMap}구역 배치도`}
              className="zone-map-img"
            />
          </div>
        </div>
      )}

    </div>
  );
}

// ── 대기 화면 우측 버튼 및 모달 콘텐츠 정의 ───────────────────────────────
// HJ_261118 - Arrival screen copy. The action each button triggers is unchanged:
// delivery → usage complete/return; recall B1–B10 first stage → turnaround;
// recall final stage (or B11–B13) → loading complete/return.
function arrivalScreenCopy(site, recall, finalReady) {
  const lidStep = ['lid', '출발 전에 덮개를 꼭 닫아 주세요', '덮개가 닫혀 있는지 한 번 더 확인해 주세요'];
  const returnStep = ['return', '로봇이 드랍존으로 복귀합니다', '로봇이 출발하면 길을 비켜 주세요'];
  if (!recall) {
    return {
      title: `${site} 사이트에 도착했어요!`,
      subtitle: '배달 서비스 · 아래 순서대로 진행해 주세요',
      steps: [['unload', '물건을 내려주세요', '적재함 안의 짐을 모두 꺼내 주세요'], lidStep, returnStep],
      question: '사용을 종료하시겠습니까?',
      hint: '짐을 모두 꺼냈다면 눌러 주세요. 로봇이 드랍존으로 돌아가요.',
      action: '사용 종료',
      confirmTitle: '사용을 종료할까요?',
      confirmBody: '확인을 누르면 로봇이 드랍존으로 출발해요. 덮개를 닫고 주변을 비워 주세요.',
    };
  }
  const turnaroundStage = /^B(?:[1-9]|10)$/.test(String(site || '')) && !finalReady;
  if (turnaroundStage) {
    return {
      title: `${site} 사이트 앞에 도착했어요!`,
      subtitle: '호출 서비스 · 로봇이 사이트 안에서 방향을 돌려요',
      steps: [
        ['clear', '주변을 비워 주세요', '로봇이 사이트 안으로 들어가 180도 돌아요'],
        ['turn', '회전이 끝날 때까지 기다려 주세요', '로봇이 멈추면 짐을 실을 수 있어요'],
        ['load', '회전 후 짐을 실어 주세요', '다음 화면에서 다시 안내해 드려요'],
      ],
      question: '짐 정리를 마치셨나요?',
      hint: '누르면 로봇이 사이트 안으로 들어가 회전을 시작해요.',
      action: recallCompletionLabel(site, finalReady),
      confirmTitle: '회전을 시작할까요?',
      confirmBody: '확인을 누르면 로봇이 사이트 안으로 들어가 180도 돌아요. 주변을 비워 주세요.',
    };
  }
  return {
    title: finalReady ? '회전을 마쳤어요! 짐을 실어 주세요' : `${site} 사이트 앞에 도착했어요!`,
    subtitle: '호출 서비스 · 아래 순서대로 진행해 주세요',
    steps: [['load', '짐을 실어 주세요', '적재함에 짐을 모두 실어 주세요'], lidStep, returnStep],
    question: '사용을 종료하시겠습니까?',
    hint: '짐을 모두 실었다면 눌러 주세요. 로봇이 드랍존으로 돌아가요.',
    action: recallCompletionLabel(site, finalReady),
    confirmTitle: '사용을 종료할까요?',
    confirmBody: '확인을 누르면 로봇이 드랍존으로 출발해요. 덮개를 닫고 주변을 비워 주세요.',
  };
}

// HJ_261118 - Simple line illustrations for the arrival steps.
function ArrivalArt({ kind }) {
  const robot = (x, y, w = 104, h = 62) => (
    <>
      <rect x={x} y={y} width={w} height={h} rx="8" fill="#f4f1ea" stroke="#bfb19b" strokeWidth="3" />
      <rect x={x + 10} y={y + h} width="18" height="16" rx="4" fill="#3b3733" />
      <rect x={x + w - 28} y={y + h} width="18" height="16" rx="4" fill="#3b3733" />
    </>
  );
  const arrow = 'none';
  const ink = { fill: arrow, stroke: '#1f3a2e', strokeWidth: 5, strokeLinecap: 'round', strokeLinejoin: 'round' };
  const box = (x, y) => (
    <><rect x={x} y={y} width="40" height="34" rx="4" fill="#d8b98a" stroke="#b38e5b" strokeWidth="3" />
      <path d={`M${x} ${y + 12}H${x + 40}`} stroke="#b38e5b" strokeWidth="3" /></>
  );
  const scenes = {
    unload: <>{robot(30, 58)}{box(140, 96)}<path d="M160 40 V80 M150 70 L160 82 L170 70" {...ink} /></>,
    load: <>{robot(30, 58)}{box(140, 96)}<path d="M160 86 V44 M150 54 L160 42 L170 54" {...ink} /></>,
    lid: <>{robot(40, 62, 120, 58)}<rect x="34" y="50" width="132" height="16" rx="6" fill="#6b4a2f" />
      <circle cx="152" cy="40" r="22" fill="#1f3a2e" /><path d="M142 40 l7 7 l13 -14" fill="none" stroke="#fff" strokeWidth="5" strokeLinecap="round" strokeLinejoin="round" /></>,
    return: <><path d="M20 130 Q100 110 180 130" fill="none" stroke="#d7cdb8" strokeWidth="16" strokeLinecap="round" />
      {robot(96, 62, 76, 46)}<path d="M84 50 H40 M52 38 L38 50 L52 62" {...ink} />
      <rect x="16" y="72" width="48" height="30" rx="6" fill="#6b4a2f" />
      <text x="40" y="92" textAnchor="middle" fontSize="13" fontWeight="800" fill="#fff"><T>드랍존</T></text></>,
    clear: <>{robot(48, 58)}<circle cx="100" cy="88" r="70" fill="none" stroke="#6b4a2f" strokeWidth="4" strokeDasharray="10 9" /></>,
    turn: <>{robot(48, 58)}<path d="M150 40 A56 56 0 1 0 160 96" {...ink} /><path d="M146 92 L162 98 L166 82" {...ink} /></>,
  };
  return (
    <svg viewBox="0 0 200 150" aria-hidden="true">
      <ellipse cx="100" cy="140" rx="80" ry="7" fill="#000" opacity=".08" />
      {scenes[kind] || scenes.unload}
    </svg>
  );
}

// HJ_261118 - Line icons for the forest-theme service chooser (white on a bark tile).
const SERVICE_ICONS = {
  delivery: (
    <svg viewBox="0 0 24 24"><path d="M21 8 12 3 3 8v8l9 5 9-5z" /><path d="m3 8 9 5 9-5M12 13v8" /></svg>
  ),
  recall: (
    <svg viewBox="0 0 24 24"><path d="M12 21s-6.5-5.6-6.5-11a6.5 6.5 0 0 1 13 0c0 5.4-6.5 11-6.5 11z" /><circle cx="12" cy="10" r="2.4" /></svg>
  ),
  docking: (
    <svg viewBox="0 0 24 24"><rect x="2.5" y="7" width="17" height="10" rx="2" /><path d="M22 11v2M11.5 9l-2 3h3l-2 3" /></svg>
  ),
};

// HJ_261118 - Home-screen card art and one-line descriptions (forest theme).
const HOME_CARD_ART = {
  usage: { image: 'robot_gemini_nobg.png', description: '짐 싣기부터 받기까지 3단계' },
  facility: { image: 'information_nobg.png', description: '화장실 · 개수대 · 매점 · 관리사무소' },
  routes: { image: 'hiking_trail_nobg.png', description: '코스별 거리와 난이도' },
};

const SIDE_BUTTONS = [
  {
    id: 'usage',
    label: '로봇 이용 방법',
    icon: (
      <img src="/robot_gemini_nobg.png" alt="로봇" style={{width:'100%',height:'100%',objectFit:'contain'}}/>
    ),
    title: '로봇 이용 방법',
    content: (
      <div>
        <div className="park-banner" style={{ marginBottom: '1.2rem' }}>
          <div className="park-banner-content">
            <div className="park-banner-logo">
              <img src="/월악산_국립공원_로고.jpg" alt="국립공원관리공단 로고" />
            </div>
            <div className="park-banner-text">
              <div className="park-banner-label">Korea National Park Service · Woraksan National Park</div>
              <div className="park-banner-breadcrumb">
                <span><T>월악산 국립공원</T></span>
                <span className="park-banner-sep">|</span>
                <span><T>닷돈재 야영장</T></span>
                <span className="park-banner-sep">|</span>
                <span className="park-banner-current"><T>로봇 이용 방법</T></span>
              </div>
            </div>
          </div>
        </div>
        <div className="guide-grid">

        {/* ── 1. 서비스 선택 방법 ── */}
        <div className="guide-card">
          <div className="guide-card-header guide-card-header-service">
            <svg viewBox="0 0 32 32" fill="none">
              <circle cx="16" cy="14" r="7" stroke="#fff" strokeWidth="2.2"/>
              <circle cx="16" cy="14" r="2.5" fill="#fff"/>
              <path d="M16 21 C10 26 6 29 6 29 L16 27 L26 29 C26 29 22 26 16 21Z" fill="#fff" opacity="0.8"/>
            </svg>
            <span><T>서비스 선택 방법</T></span>
          </div>
          <div className="guide-card-body">
            <figure className="guide-card-visual">
              <img
                src={`${process.env.PUBLIC_URL}/guide/service-selection.png`}
                alt="배달 서비스와 호출 서비스를 선택하는 화면"
              />
              <figcaption><T>서비스 선택 화면</T></figcaption>
            </figure>
            <div className="guide-card-copy">
              <div className="guide-service-definition guide-service-delivery">
                <div>
                  <strong><T>배달 서비스</T></strong>
                  <span><T>캠핑 시작</T></span>
                </div>
                <p><T>캠핑을 시작할 때 사용합니다. 드랍존의 캠핑 장비를 선택한 캠핑 사이트 안으로 운반합니다.</T></p>
              </div>
              <div className="guide-service-definition guide-service-recall">
                <div>
                  <strong><T>호출 서비스</T></strong>
                  <span><T>캠핑 종료</T></span>
                </div>
                <p><T>캠핑을 마칠 때 사용합니다. 로봇을 사이트 도로 측 대기점으로 호출해 장비를 싣고 드랍존으로 복귀합니다.</T></p>
              </div>
              <div className="guide-service-admin-notice" role="note">
                <span aria-hidden="true">🔒</span>
                <p><strong><T>[충전] 버튼은 관리자 전용 기능입니다.</T></strong><T> 이용객은 누르거나 조작하지 마세요.</T></p>
              </div>
              {[
                '대기 화면에서 [서비스 선택]을 누른 뒤 이용 목적에 맞는 서비스를 고릅니다.',
                '사이트(B1~B13)와 안내를 확인하고 [예]를 누른 뒤 사이트명을 입력합니다.',
              ].map((text, i) => (
                <div key={i} className="guide-step">
                  <span className="guide-step-num guide-step-num-service">{i + 1}</span>
                  <span><T>{text}</T></span>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* ── 2. 로봇 정지 방법 ── */}
        <div className="guide-card">
          <div className="guide-card-header guide-card-header-stop">
            <svg viewBox="0 0 32 32" fill="none">
              <circle cx="16" cy="16" r="11" stroke="#fff" strokeWidth="2.2"/>
              <rect x="11" y="11" width="10" height="10" rx="2" fill="#fff"/>
            </svg>
            <span><T>로봇 정지 방법</T></span>
          </div>
          <div className="guide-card-body">
            <figure className="guide-card-visual">
              <img
                src={`${process.env.PUBLIC_URL}/guide/robot-stop.png`}
                alt="운행 정지 안내와 예 버튼이 표시된 주행 화면"
              />
              <figcaption><T>주행 화면의 정지 버튼</T></figcaption>
            </figure>
            <div className="guide-card-copy">
              {[
                '로봇 이동 중 왼쪽 화면에서 “운행을 정지하시겠습니까?”를 확인합니다.',
                '문구 아래의 빨간색 [예] 버튼을 누릅니다.',
                '화면에 “운행이 정지되었습니다”가 표시되는지 확인합니다.',
              ].map((text, i) => (
                <div key={i} className="guide-step">
                  <span className="guide-step-num guide-step-num-stop">{i + 1}</span>
                  <span><T>{text}</T></span>
                </div>
              ))}
            </div>
          </div>
        </div>

        </div>
      </div>
    ),
  },
  {
    id: 'facility',
    label: '시설 안내',
    icon: (
      <img
        src={`${process.env.PUBLIC_URL}/information_nobg.png`}
        alt="시설 안내"
        className="waiting-grid-btn-icon-img"
      />
    ),
    title: '시설 안내',
    content: <FacilityContent />,
  },
  {
    id: 'routes',
    label: '탐방로',
    icon: (
      <img
        src={`${process.env.PUBLIC_URL}/hiking_trail_nobg.png`}
        alt="탐방로"
        className="waiting-grid-btn-icon-img"
      />
    ),
    title: '탐방로 안내',
    content: (
      <div className="trail-wrap">

        {/* ── 헤더 배너 ── */}
        <div className="park-banner">
          <div className="park-banner-content">
            <div className="park-banner-logo">
              <img src="/월악산_국립공원_로고.jpg" alt="국립공원관리공단 로고" />
            </div>
            <div className="park-banner-text">
              <div className="park-banner-label">Korea National Park Service · Woraksan National Park</div>
              <div className="park-banner-breadcrumb">
                <span><T>월악산 국립공원</T></span>
                <span className="park-banner-sep">|</span>
                <span><T>닷돈재 야영장</T></span>
                <span className="park-banner-sep">|</span>
                <span className="park-banner-current"><T>탐방로 안내</T></span>
              </div>
            </div>
          </div>
        </div>

        {/* ── 통제 상황 경고 배너 ── */}
        <div className="trail-warning">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="#e65100">
            <path d="M12 2L1 21h22L12 2zm0 3.5L20.5 20h-17L12 5.5zM11 10v4h2v-4h-2zm0 6v2h2v-2h-2z"/>
          </svg>
          <span><T>입산 이전 통제 상황을 반드시 확인해주세요</T></span>
        </div>

        {/* ── 탐방로 카드 그리드 ── */}
        <div className="trail-grid">
          {[
            { name: '만수계곡 자연관찰로',     start: '닷돈재',      time: '1~1.5시간',       dist: '왕복 2km',      color: '#66bb6a', images: ['월악산_만수계곡1.JPG','월악산_만수계곡2.jpg','월악산_만수계곡3.jpeg','월악산_만수계곡4.JPG','월악산_만수계곡5.JPG'] },
            { name: '하늘재 역사관찰로',        start: '닷돈재·송계', time: '약 2시간',         dist: null,            color: '#66bb6a', images: ['월악산_하늘재_역사1.JPG','월악산_하늘재_역사2.jpg','월악산_하늘재_역사3.JPG'] },
            { name: '송계계곡 산책 (송계팔경)', start: '덕주·송계',   time: '약 30분',          dist: null,            color: '#66bb6a', images: ['월악산_송계계곡 산책1.JPG','월악산_송계계곡 산책2.JPG'] },
            { name: '만수봉 계곡코스',          start: '닷돈재',      time: '약 2.5시간',       dist: '왕복 4.4km',    color: '#2d6e40' },
            { name: '북바위산 코스',            start: '송계',        time: '2시간대',          dist: null,            color: '#2d6e40', images: ['월악산_북바위산1.jpg','월악산_북바위산2.jpg','월악산_북바위산3.jpg','월악산_북바위산4.jpg'] },
            { name: '덕주산성 탐방로',          start: '덕주',        time: '1~2시간',          dist: null,            color: '#2d6e40', images: ['월악산_덕주산성_1.JPG'] },
            { name: '영봉코스 (동창교 출발)',   start: '송계',        time: '약 3시간',         dist: '편도 약 4.3km', color: '#1b5e20', images: ['월악산_영봉코스_동창교_출발1.jpg','월악산_영봉코스_동창교_출발2.JPG'] },
            { name: '영봉코스 (덕주사 출발)',   start: '덕주',        time: '3시간 40분 이상',  dist: '편도 약 6km',   color: '#1b5e20', images: ['월악산_영봉코스_덕주사_출발1.jpg','월악산_영봉코스_덕주사_출발2.jpg'] },
          ].map((trail, i) => (
            <div key={i} className="trail-card" style={{ borderLeftColor: trail.color }}>
              <div className="trail-card-name"><T>{trail.name}</T></div>
              <div className="trail-card-meta">
                <span className="trail-meta-item">
                  <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"/></svg>
                  <T>{trail.start}</T>
                </span>
                <span className="trail-meta-item">
                  <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8zm.5-13H11v6l5.25 3.15.75-1.23-4.5-2.67V7z"/></svg>
                  <T>{trail.time}</T>
                </span>
                {trail.dist && (
                  <span className="trail-meta-item">
                    <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><path d="M1.5 4v5h5L12 3l5.5 6h5V4H1.5zM12 19l-5.5-6H1.5v5h21v-5h-5L12 19z"/></svg>
                    <T>{trail.dist}</T>
                  </span>
                )}
              </div>
              {trail.images && <TrailCarousel title={trail.name} images={trail.images} />}
            </div>
          ))}
        </div>

        {/* ── 하단 안내 ── */}
        <p className="trail-footer-note"><T>※ 기상 상황 및 계절에 따라 입산이 통제될 수 있습니다. 안전한 탐방을 위해 공단 안내에 따라주세요.</T></p>
      </div>
    ),
  },
  {
    id: 'settings',
    label: '진단',
    icon: (
      <svg viewBox="0 0 100 100">
        <defs><clipPath id="sg-diag"><circle cx="50" cy="50" r="36"/></clipPath></defs>
        {/* 배경 원 */}
        <circle cx="50" cy="50" r="36" fill="#EAF3DE"/>
        {/* 배경 하늘 */}
        <rect x="14" y="14" width="72" height="36" fill="#C9E8F5" opacity="0.4" clipPath="url(#sg-diag)"/>
        {/* 클립보드 몸체 */}
        <rect x="30" y="32" width="40" height="48" rx="4" fill="#4A7C3F"/>
        <rect x="30" y="32" width="40" height="48" rx="4" fill="none" stroke="#2d5a24" strokeWidth="1.2"/>
        {/* 클립보드 상단 헤더 */}
        <rect x="30" y="32" width="40" height="10" rx="4" fill="#2d5a24"/>
        {/* 화면 영역 */}
        <rect x="34" y="46" width="32" height="22" rx="3" fill="#1b3d14"/>
        {/* 심전도 파형 */}
        <polyline
          points="36,57 40,57 43,50 46,64 49,57 52,57 54,53 57,57 64,57"
          fill="none" stroke="#A8E063" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
        {/* 하단 텍스트 라인들 */}
        <rect x="34" y="71" width="20" height="2" rx="1" fill="#A5C880" opacity="0.6"/>
        <rect x="34" y="75" width="14" height="2" rx="1" fill="#A5C880" opacity="0.35"/>
        {/* 체크 상태 뱃지 */}
        <circle cx="72" cy="34" r="10" fill="#BA7517"/>
        <circle cx="72" cy="34" r="7.5" fill="#EF9F27"/>
        <polyline points="68,34 71,37 76,30" fill="none" stroke="#fff" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
      </svg>
    ),
    title: '진단',
    content: <DiagnosticsMonitor />,
  },
];

// ── 관리할 사이트 목록 (toggle_publisher.py의 SITE_NAMES와 동일) ──────────
const SITE_NAMES = Array.from({ length: 13 }, (_, i) => `B${i + 1}`);
// HJ_261118 - A site's zone is the leading letters of its id (B13 → B), so adding
// A1…/C1… sites to SITE_NAMES creates the A/C zone tabs without UI changes.
const siteZone = site => (String(site).match(/^[A-Za-z]+/) || [''])[0].toUpperCase();
// HJ_261118 - TEMPORARY placeholder zones shown on the site board only. They are not in
// SITE_NAMES (the backend rejects unknown sites), render as "준비 중" and cannot be
// selected. Move a site into SITE_NAMES once map/planning support it; delete when done.
const PLANNED_SITES = [
  ...Array.from({ length: 22 }, (_, i) => `A${i + 1}`),
  ...Array.from({ length: 8 }, (_, i) => `C${i + 1}`),
].filter(site => !SITE_NAMES.includes(site));
const BOARD_SITES = [...SITE_NAMES, ...PLANNED_SITES];
const isPlannedSite = site => PLANNED_SITES.includes(site);
const SITE_ZONES = Array.from(new Set(BOARD_SITES.map(siteZone))).sort();
const DEFAULT_SITE_ZONE = SITE_NAMES.length ? siteZone(SITE_NAMES[0]) : (SITE_ZONES[0] || '');
const sitesInZone = zone => BOARD_SITES.filter(site => siteZone(site) === zone);
// Up to 16 sites (4 × 4) fit one board page; larger zones page within the zone.
const ZONE_PAGE_SIZE = 16;
const sitePageInZone = site => Math.max(0, Math.floor(sitesInZone(siteZone(site)).indexOf(site) / ZONE_PAGE_SIZE));

// ── 사이트별 이미지 파일 매핑 ─────────────────────────────────────────────
const SITE_IMAGES = {
  B1: 'B1_site.jpg', B2: 'B2_site.jpg', B3: 'B3_site.jpg',
  B4: 'B4_site.jpg', B5: 'B5_site.jpg', B6: 'B6_site.jpg',
  B7: 'B7_site.jpg', B8: 'B8_site.jpg', B9: 'B9_ste.jpg',
  B10: 'B10_site.jpg', B11: 'B11_site.jpg', B12: 'B12_site.jpg',
  B13: 'B13_site.jpg',
};

// level: 0=오프라인, 1=약함, 2=보통, 3=강함
function WifiIcon({ level }) {
  const arcColor = (minLevel) => level >= minLevel ? '#2d6e40' : '#d0d8d0';
  return (
    <svg width="52" height="52" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
      <path d="M1.5 8.5C5.1 4.9 10.3 3 12 3s6.9 1.9 10.5 5.5" stroke={arcColor(3)} strokeWidth="2" strokeLinecap="round"/>
      <path d="M4.5 11.5C6.9 9.1 9.7 8 12 8s5.1 1.1 7.5 3.5" stroke={arcColor(2)} strokeWidth="2" strokeLinecap="round"/>
      <path d="M7.5 14.5C8.9 13.1 10.4 12.5 12 12.5s3.1.6 4.5 2" stroke={arcColor(1)} strokeWidth="2" strokeLinecap="round"/>
      <circle cx="12" cy="18" r="1.5" fill={level > 0 ? '#2d6e40' : '#e53935'}/>
      {level === 0 && (
        <line x1="4" y1="4" x2="20" y2="20" stroke="#e53935" strokeWidth="2" strokeLinecap="round"/>
      )}
    </svg>
  );
}

// pct: 0~100 (null이면 수신 전 — "–%" 표시)
function BatteryIcon({ pct }) {
  const fillRatio = pct === null ? 0 : Math.max(0, Math.min(100, pct)) / 100;
  const fillColor =
    pct === null ? '#d0d0d0'
    : pct < 25  ? '#e53935'
    : pct < 35  ? '#f57c00'
    : '#2d6e40';

  // 내부 채움 가용 너비: x=3.5 ~ x=38.5 → 35px
  const fillW = Math.round(fillRatio * 35);

  return (
    <svg width="64" height="37" viewBox="0 0 48 28" fill="none" xmlns="http://www.w3.org/2000/svg">
      {/* 배터리 몸체 */}
      <rect x="1" y="4" width="40" height="20" rx="4" ry="4"
        stroke="rgba(0,0,0,0.3)" strokeWidth="2" fill="rgba(0,0,0,0.04)"/>
      {/* 배터리 +극 돌기 */}
      <rect x="42" y="10" width="5" height="8" rx="2" ry="2"
        fill="rgba(0,0,0,0.2)"/>
      {/* 배터리 내부 채움 */}
      {fillW > 0 && (
        <rect x="3.5" y="6.5" width={fillW} height="15" rx="2" ry="2"
          fill={fillColor}/>
      )}
    </svg>
  );
}

// 261111 - Only this view subscribes to the 10 Hz driving samples. App owns the
// polling lifetime (useDrivingDisplayFeed) but no longer re-renders per sample.
function LiveDrivingDisplay({ feed, store, ...props }) {
  const snapshot = useDrivingDisplaySnapshot(feed, store);
  return <DrivingDisplay snapshot={snapshot} {...props} />;
}

function App({ drivingPreviewSnapshot = null, drivingPreviewTheme = 'light' } = {}) {
  // HH_261001 - Local preview renders this SAME application. Only its data source changes;
  // all robot-network authority is explicitly disabled in preview mode.
  const previewMode = drivingPreviewSnapshot !== null;
  // HJ_261118 - 손님 화면 언어(한·영). 관리자 화면은 번역하지 않는다.
  const lang = useLang();
  const t = useT();
  // ── React 상태(State) 선언 ──────────────────────────────────────────────
  const [states, setStates] = useState(() => {
    const init = {};
    SITE_NAMES.forEach(site => { init[site] = false; });
    return init;
  });
  const [occupiedSites, setOccupiedSites] = useState([]);

  const [connected, setConnected] = useState(false);
  const [drivingTheme, setDrivingTheme] = useState('light');
  const [batteryPct, setBatteryPct] = useState(null); // null = 아직 수신 전
  // HH_261002 - Present charging completion only after the backend confirms
  // BMS full status or 100% SOC during an active charging session.
  const [batteryChargeComplete, setBatteryChargeComplete] = useState(false);
  const [togglePage, setTogglePage] = useState(0);   // HJ_261118 - page within the active zone
  const [siteZoneTab, setSiteZoneTab] = useState(DEFAULT_SITE_ZONE);
  const [engageState, setEngageState] = useState(false);
  // HH_260721 - Display operational progress independently from diagnostic health.
  const [serviceStateName, setServiceStateName] = useState('PREPARING');
  const [serviceStateDescription, setServiceStateDescription] = useState('');
  const [missionExecutionError, setMissionExecutionError] = useState('');
  const missionExecutionErrorRef = useRef('');
  const [recallFinalReturnReady, setRecallFinalReturnReady] = useState(false);
  const recallFinalReturnReadyRef = useRef(false);
  const [returnRequestPending, setReturnRequestPending] = useState(false);
  const returnRequestPendingRef = useRef(false);
  const serviceStateIdRef = useRef(null);
  const [systemHealth, setSystemHealth] = useState('STARTING');
  const [missionPhase, setMissionPhase] = useState('INITIALIZING');
  const [missionSource, setMissionSource] = useState('none');
  const [headlightState, setHeadlightState] = useState(false); // HH_260708 - Track the headlight toggle.
  const [signalLevel, setSignalLevel] = useState(() => {
    if (!navigator.onLine) return 0;
    const conn = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
    if (!conn) return 3;
    const type = conn.effectiveType;
    if (type === '4g') return 3;
    if (type === '3g') return 2;
    return 1;
  });
  const [showWaiting, setShowWaiting] = useState(true); // 대기 화면 표시 여부
  const [showServiceSelection, setShowServiceSelection] = useState(false);
  // 서비스 선택 전 안전 안내를 보여주는 중인지 여부
  const wsRef = useRef(null);
  // Treat every reconnect as a new authority channel. Callbacks from a closed
  // socket must never roll the current mission back to an older snapshot.
  const wsMountedRef = useRef(false);
  const wsGenerationRef = useRef(0);
  const wsReconnectTimerRef = useRef(null);

  const [returnCompletionPending, setReturnCompletionPending] = useState(false);

  const [outsideHoursMsg, setOutsideHoursMsg] = useState(false); // 운영시간 외 안내 메시지
  const [selectedSite, setSelectedSite] = useState(null);         // 이미지 프리뷰 대상 사이트
  // HH_260904 - Delivery enters the campsite; recall stops at the authored
  // roadside wait pose. Keep the intent explicit through both confirmations.
  const [destinationIntent, setDestinationIntent] = useState('delivery');
  const destinationIntentRef = useRef('delivery');
  // The service menu owns the delivery/recall role. Pin it so an idle
  // "no mission" snapshot cannot silently flip the role back to delivery.
  const intentPinnedRef = useRef(false);
  // The backend owns mission admission. Keep its exact identity so OFF and
  // Return commands cannot affect a mission accepted after this UI snapshot.
  const missionDispatchActiveRef = useRef(false);
  const missionDispatchGenerationRef = useRef(0);
  const missionDispatchSiteRef = useRef('');
  const missionDispatchOwnerRef = useRef('');
  const missionAuthorityRevisionRef = useRef(0);
  const [missionDispatch, setMissionDispatch] = useState({
    active: false,
    generation: 0,
    site: '',
    owner: '',
    intent: '',
  });
  const recallRequestEpochRef = useRef(0);
  const [recallRequestPending, setRecallRequestPending] = useState(false);
  const [activeRecallSite, setActiveRecallSite] = useState(null);
  const [arrivedSite, setArrivedSite] = useState(null);           // 도착 완료 사이트
  const [showArrivalComplete, setShowArrivalComplete] = useState(false); // 이용 완료 팝업
  // HJ_261118 - Second confirmation before an arrival action makes the robot move.
  const [arrivalConfirmOpen, setArrivalConfirmOpen] = useState(false);
  const [isReturning, setIsReturning] = useState(false);                 // Drop Zone 복귀 중
  const [activeModal, setActiveModal] = useState(null);           // 현재 열린 모달 ID
  const [showLoginModal, setShowLoginModal] = useState(false);   // 설정 로그인 모달
  const [loginId, setLoginId] = useState('');
  const [loginPw, setLoginPw] = useState('');
  const [loginError, setLoginError] = useState('');
  const [activeField, setActiveField] = useState('id'); // 가상 키보드 포커스 필드
  const [kbCaps, setKbCaps] = useState(false);          // 가상 키보드 대소문자
  const [diagPressProgress, setDiagPressProgress] = useState(0); // 진단 장누르기 진행도 (0~100)
  const [showGuestRecall, setShowGuestRecall] = useState(false); // HJ_260601: 게스트 호출 알림 팝업
  const [guestNavigateSite, setGuestNavigateSite] = useState(null); // 게스트 사이트 이동 알림
  const diagPressAnimRef = useRef(null);
  const [showMoveConfirm, setShowMoveConfirm] = useState(false); // 출발 최종 확인 팝업
  const [stopConfirmation, setStopConfirmation] = useState(null); // HH_261001 - Second confirmation for the stop button.
  const [showDockingConfirm, setShowDockingConfirm] = useState(false);
  const [serviceDockingPending, setServiceDockingPending] = useState(false);
  const [serviceDockingStatus, setServiceDockingStatus] = useState('');
  const serviceDockingPendingRef = useRef(false);
  const [showMoveVerify, setShowMoveVerify] = useState(false);  // 사이트명 입력 확인 팝업
  const [moveVerifyInput, setMoveVerifyInput] = useState('');
  const [moveVerifyError, setMoveVerifyError] = useState(false);
  const [missionBlockMessage, setMissionBlockMessage] = useState('');
  const [batteryReturnMessage, setBatteryReturnMessage] = useState('');
  const [batteryReturnState, setBatteryReturnState] = useState(emptyBatteryReturnState);
  const batteryReturnStateRef = useRef(emptyBatteryReturnState());
  const [parkingPolicy, setParkingPolicy] = useState({
    charging_required: false,
    parking_policy_mode: 'auto',
    parking_selected_method: '',
  });
  const [redockStatus, setRedockStatus] = useState({
    pending: false,
    waitingForCan: false,
    status: 'idle',
    message: '',
    received: false,
  });
  // HH_260819 - Public field-operation evidence stays lightweight and separate
  // from the administrator-only, high-bandwidth telemetry session.
  const serviceMetrics = useServiceMetricsSummary(3000, !previewMode);
  const chargingAccess = useChargingAccess(connected && !previewMode);
  const chargingAccessLocked = chargingAccess.pending || chargingAccessBusy(chargingAccess.snapshot?.phase);
  // HJ_261105 - After the staging move, the backend-measured stop is shown briefly as a loading cue,
  // then ordinary standby takes over for destination selection. The timer only changes the screen;
  // it never authorizes motion, and it restarts whenever the measured stop is lost and regained.
  const cargoAccessReady = chargingAccess.snapshot?.phase === 'ready'
    && chargingAccess.snapshot?.cargo_access_ready === true;
  const [cargoReadyNoticeExpired, setCargoReadyNoticeExpired] = useState(false);
  useEffect(() => {
    setCargoReadyNoticeExpired(false);
    if (!cargoAccessReady) return undefined;
    const timer = setTimeout(() => setCargoReadyNoticeExpired(true), CARGO_READY_NOTICE_MS);
    return () => clearTimeout(timer);
  }, [cargoAccessReady]);

  const previewMission = drivingPreviewSnapshot?.mission;
  const previewConnected = drivingPreviewSnapshot?.connected;
  const previewBattery = drivingPreviewSnapshot?.battery?.percentage;
  const previewActive = previewMission?.active;
  const previewGeneration = previewMission?.generation;
  const previewSite = previewMission?.site;
  const previewIntent = previewMission?.intent;
  const previewServiceState = previewMission?.service_state_name;
  const previewPhase = previewMission?.phase;
  const previewDescription = previewMission?.description;
  useEffect(() => {
    if (!previewMode) return;
    setConnected(previewConnected === true);
    setBatteryPct(previewBattery ?? null);
    setBatteryChargeComplete(false);
    setServiceStateName(previewServiceState || 'DROP_ZONE_WAIT');
    setServiceStateDescription(previewDescription || '');
    setMissionPhase(previewPhase || 'READY');
    setSystemHealth('OK');
    setMissionSource(previewActive ? 'ui' : 'none');
    setMissionDispatch({ active: Boolean(previewActive), generation: previewGeneration || 0,
      site: previewSite || '', intent: previewIntent || 'delivery', owner: 'robot' });
    setStates(Object.fromEntries(SITE_NAMES.map(site => [site, Boolean(previewActive && site === previewSite)])));
    setDestinationIntent(previewIntent === 'recall' ? 'recall' : 'delivery');
    setActiveRecallSite(previewActive && previewIntent === 'recall' ? previewSite : null);
    setIsReturning(['RETURNING_TO_DROP_ZONE', 'RETURN_WITH_CARGO', 'DROP_ZONE_PARKING'].includes(previewServiceState));
    setShowWaiting(!previewActive);
    setShowServiceSelection(false);
    setShowArrivalComplete(false);
    setSelectedSite(null);
    setArrivedSite(null);
    setSiteZoneTab(previewSite ? siteZone(previewSite) : DEFAULT_SITE_ZONE);
    setTogglePage(previewSite ? sitePageInZone(previewSite) : 0);
  }, [previewMode, previewConnected, previewBattery, previewActive, previewGeneration,
    previewSite, previewIntent, previewServiceState, previewPhase, previewDescription]);

  const openSettingsLoginModal = () => {
    if (previewMode) return;
    setActiveModal(null);
    setLoginId('');
    setLoginPw('');
    setLoginError('');
    setActiveField('id');
    setKbCaps(false);
    setShowLoginModal(true);
  };

  const handleLogin = () => {
    if (previewMode) return;
    if (loginId === 'admin' && loginPw === '1234') {
      setShowLoginModal(false);
      setActiveModal('settings');
    } else {
      setLoginError('아이디 또는 비밀번호가 올바르지 않습니다.');
    }
  };

  // ── 가상 키보드 입력 처리 ───────────────────────────────────────────────────
  const handleVirtualKey = (key) => {
    const setter = activeField === 'id' ? setLoginId
      : activeField === 'pw' ? setLoginPw
      : setMoveVerifyInput;
    if (key === '⇧') {
      setKbCaps(prev => !prev);
    } else {
      // HJ_260804 - Site verification codes are case-insensitive at the API
      // boundary. Functional updates also retain every queued touchscreen key.
      const forceUpper = activeField === 'moveVerify' || kbCaps;
      setter(current => {
        if (key === '⌫') return current.slice(0, -1);
        if (key === 'SPACE') return current + ' ';
        return current + (forceUpper ? key.toUpperCase() : key);
      });
    }
    if (activeField === 'moveVerify') {
      setMoveVerifyError(false);
    }
  };

  const KB_ROWS = [
    ['1','2','3','4','5','6','7','8','9','0','⌫'],
    ['q','w','e','r','t','y','u','i','o','p'],
    ['a','s','d','f','g','h','j','k','l'],
    ['⇧','z','x','c','v','b','n','m','⇧'],
  ];

  // ── 진단 페이지 비밀 진입 (우측 끝 5초 장누르기) ───────────────────────────
  const handleDiagPressStart = (e) => {
    if (previewMode) return;
    e.stopPropagation();
    const startTime = Date.now();
    const duration = 1500;

    const animate = () => {
      const elapsed = Date.now() - startTime;
      const progress = Math.min((elapsed / duration) * 100, 100);
      setDiagPressProgress(progress);

      if (progress < 100) {
        diagPressAnimRef.current = requestAnimationFrame(animate);
      } else {
        diagPressAnimRef.current = null;
        setDiagPressProgress(0);
        openSettingsLoginModal();
      }
    };

    diagPressAnimRef.current = requestAnimationFrame(animate);
  };

  const handleDiagPressEnd = (e) => {
    e.stopPropagation();
    if (diagPressAnimRef.current) {
      cancelAnimationFrame(diagPressAnimRef.current);
      diagPressAnimRef.current = null;
    }
    setDiagPressProgress(0);
  };

  // 하나라도 ON인지 확인
  const anyOn = Object.values(states).some(v => v);

  // 현재 ON인 사이트 (이동 중인 사이트)
  const activeStateSite = SITE_NAMES.find(s => states[s]) || null;
  // The backend mirrors both delivery and recall in the shared site-state
  // buttons. Keep a typed recall out of the delivery-only presentation path.
  const activeSite = activeRecallSite ? null : activeStateSite;
  // HH_260730 - A manual mission exists only after a source-aware RViz goal,
  // not merely because the operator has armed engage.
  const manualDriveActive =
    missionSource === 'manual'
    && !activeSite
    && !selectedSite
    && !arrivedSite
    && !isReturning
    && ACTIVE_MANUAL_PHASES.has(missionPhase);
  const displayedReturning =
    isReturning && missionPhase !== 'INITIALIZING';
  // RETURN_WITH_CARGO is shared by Delivery and Recall. The accepted mission
  // identity, rather than a generic service-state number, selects UI wording
  // and determines which client is allowed to send Return.
  const hasExplicitRecallIntent = Boolean(activeRecallSite);
  const recallArrivalPresentation =
    hasExplicitRecallIntent
    && (
      serviceStateName === 'GUEST_LOADING_WAIT'
      || activeRecallSite === arrivedSite
    );
  const recallReturnPresentation =
    hasExplicitRecallIntent
    && serviceStateName === 'RETURN_WITH_CARGO';
  // HH_261002 - The first Recall completion starts a controller-owned site
  // re-entry/turnaround while the arrival site stays latched for the second
  // confirmation. Suppress only its stale arrival presentation; retain the
  // mission identity and Return authority until RECALL_RETURN_WAIT is ready.
  const recallTurnaroundInProgress = Boolean(
    missionDispatch.active && missionDispatch.intent === 'recall'
    && activeRecallSite && activeRecallSite === arrivedSite
    && serviceStateName === 'RETURN_WITH_CARGO'
    && !recallFinalReturnReady
  );
  const robotOwnsReturn = !missionExecutionError && robotCanCompleteMission(
    missionDispatch, arrivedSite, serviceStateName
  );
  const recallProgress = recallReturnProgress(activeRecallSite, serviceStateDescription);
  const motionNotice = serviceMotionNotice(missionPhase, systemHealth);
  const guestAdmissionMessage = guestAdmissionNotice({
    dispatch: missionDispatch, showRecall: showGuestRecall, noticeSite: guestNavigateSite,
    missionPhase, serviceStateName, systemHealth,
    arrivalVisible: showArrivalComplete, arrivedSite, executionError: missionExecutionError,
  });
  const guestAdmissionStatus = guestAdmissionMessage ? (
    <span data-ui="guest-admission-status" role="status" aria-live="polite"
      style={{ display: 'block', fontSize: '0.85rem', position: 'static', pointerEvents: 'none' }}>
      {t(guestAdmissionMessage)}
    </span>
  ) : null;
  const guestOwnsReturn =
    missionDispatch.active
    && missionDispatch.site === arrivedSite
    && missionDispatch.generation > 0
    && missionDispatch.owner === 'guest';

  // HH_261001 - Presentation only. Dismissing this layer must not call Stop,
  // Return, engage, site-toggle or any other mission-authority handler.
  const drivingDisplay = useDrivingDisplayFeed({
    missionDispatch, serviceStateName, missionPhase, connected,
    injectedSnapshot: drivingPreviewSnapshot,
    // HH_261002 - A diagnostic ERROR during an accepted safety hold is a
    // warning to show in the driving view, not a reason to abandon that view.
    // Mission errors and operator modals still own their existing screens.
    blocked: showLoginModal || Boolean(activeModal)
      || showMoveConfirm
      || showMoveVerify || showDockingConfirm || Boolean(missionExecutionError)
      || (showArrivalComplete && arrivedSite && robotOwnsReturn
        && !recallTurnaroundInProgress),
  });
  // ── 운영시간 게이트 확인 ───────────────────────────────────────────────
  const isWithinOperatingHours = () => {
    if (!OPERATING_HOURS_GATE_ENABLED) {
      return true;
    }

    const hour = new Date().getHours();
    if (OPERATING_HOURS_START <= OPERATING_HOURS_END) {
      return hour >= OPERATING_HOURS_START && hour < OPERATING_HOURS_END;
    }

    return hour >= OPERATING_HOURS_START || hour < OPERATING_HOURS_END;
  };
  // HJ_261118 - Drives the home hero's "ready / outside hours" status line.
  const operatingNow = isWithinOperatingHours();

  // ── 대기 화면 터치 핸들러 (운영시간 체크) ──────────────────────────────
  const handleWaitingClick = () => {
    if (chargingAccessLocked) return;
    if (!isWithinOperatingHours()) {
      setOutsideHoursMsg(true);
      setTimeout(() => setOutsideHoursMsg(false), 3000);
      return;
    }
    // HJ_261118 - Go straight to the service chooser; the safety poster gate
    // before it was removed at the operator's request.
    // Pin the public chooser immediately, not only after role selection:
    // idle station heartbeats are status, not navigation commands.
    intentPinnedRef.current = true;
    setShowServiceSelection(true);
    setShowWaiting(false);
  };



  useEffect(() => {
    if (!returnCompletionPending || missionDispatch.active) return;
    // HH_260929 - A completed return resets the touchscreen to the normal
    // standby selection state only after mission dispatch is no longer active.
    setReturnCompletionPending(false);
    intentPinnedRef.current = false;
    destinationIntentRef.current = 'delivery';
    setDestinationIntent('delivery');
    setActiveRecallSite(null);
    setSelectedSite(null);
    setArrivedSite(null);
    setShowArrivalComplete(false);
    setShowServiceSelection(false);
    setShowWaiting(true);
  }, [returnCompletionPending, missionDispatch.active]);

  // HH_261002 - An admitted mission can outlive its site toggle during a
  // recovery. Keep its driving presentation above the public standby screen.
  useEffect(() => {
    if ((isReturning || missionDispatch.active) && showWaiting) {
      setShowWaiting(false);
    }
  }, [isReturning, missionDispatch.active, showWaiting]);

  // Reopening standby ends the visit, so the next visitor starts from the
  // service menu instead of inheriting the previous role.
  useEffect(() => {
    if (showWaiting) intentPinnedRef.current = false;
  }, [showWaiting]);

  // An accepted or already-running mission must always show the live control
  // surface instead of the public service chooser.
  useEffect(() => {
    if (anyOn || manualDriveActive || isReturning || missionDispatch.active) {
      setShowServiceSelection(false);
    }
  }, [anyOn, manualDriveActive, isReturning, missionDispatch.active]);

  useEffect(() => {
    if (!showServiceSelection) {
      setShowDockingConfirm(false);
    }
  }, [showServiceSelection]);

  // ── 인터넷 신호 강도 감지 (navigator.connection + online/offline) ─────
  useEffect(() => {
    const getLevel = () => {
      if (!navigator.onLine) return 0;
      const conn = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
      if (!conn) return 3;
      const type = conn.effectiveType;
      if (type === '4g') return 3;
      if (type === '3g') return 2;
      return 1;
    };
    const update = () => setSignalLevel(getLevel());
    window.addEventListener('online',  update);
    window.addEventListener('offline', update);
    const conn = navigator.connection || navigator.mozConnection || navigator.webkitConnection;
    if (conn) conn.addEventListener('change', update);
    return () => {
      window.removeEventListener('online',  update);
      window.removeEventListener('offline', update);
      if (conn) conn.removeEventListener('change', update);
    };
  }, []);

  // ── WebSocket 연결 함수 ─────────────────────────────────────────────────
  const connect = useCallback(() => {
    if (!wsMountedRef.current) return;
    if (wsReconnectTimerRef.current !== null) {
      clearTimeout(wsReconnectTimerRef.current);
      wsReconnectTimerRef.current = null;
    }
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    const host = window.location.host || 'localhost:8010';
    const connectionGeneration = wsGenerationRef.current + 1;
    wsGenerationRef.current = connectionGeneration;

    // FastAPI 백엔드의 WebSocket 엔드포인트에 연결 (same host:port as HTTP)
    const ws = new WebSocket(`${protocol}://${host}/ws`);
    wsRef.current = ws;

    // 연결 성공 시 → 연결 상태 녹색 표시
    ws.onopen = () => {
      if (
        !wsMountedRef.current
        || wsGenerationRef.current !== connectionGeneration
        || wsRef.current !== ws
      ) {
        ws.close();
        return;
      }
      setConnected(true);
    };

    // 서버에서 메시지 수신 시 → 상태 업데이트
    ws.onmessage = (event) => {
      if (
        !wsMountedRef.current
        || wsGenerationRef.current !== connectionGeneration
        || wsRef.current !== ws
      ) return;
      const data = JSON.parse(event.data);

      // Persist failure snapshots across reloads and minimal phase frames.
      // Only an explicit backend clear may remove the operator-recovery warning.
      if ('mission_execution_error' in data || data.error === 'campsite_maneuver_failed') {
        const reason = 'mission_execution_error' in data
          ? String(data.mission_execution_error || '')
          : String(data.message || '사이트 제어 오류');
        missionExecutionErrorRef.current = reason;
        setMissionExecutionError(reason);
      }
      if ('recall_final_return_ready' in data) {
        const ready = data.recall_final_return_ready === true;
        if (ready !== recallFinalReturnReadyRef.current) {
          returnRequestPendingRef.current = false;
          setReturnRequestPending(false);
        }
        recallFinalReturnReadyRef.current = ready;
        setRecallFinalReturnReady(ready);
      }
      if (data.error) {
        returnRequestPendingRef.current = false;
        setReturnRequestPending(false);
        if (['recall_completion_stage_mismatch', 'recall_final_confirmation_required', 'recall_final_return_not_ready'].includes(data.error)) {
          setMissionBlockMessage('완료 단계가 변경되었습니다. 현재 화면의 안내를 확인한 후 다시 눌러주세요.');
        }
      }

      if (data.error === 'battery_below_mission_minimum') {
        const blockedBattery = Number(data.battery_percentage);
        if (Number.isFinite(blockedBattery) && blockedBattery >= 0) {
          setBatteryPct(blockedBattery);
        }
        const cleared = {};
        SITE_NAMES.forEach(site => { cleared[site] = false; });
        setStates(cleared);
        setSelectedSite(null);
        setShowMoveConfirm(false);
        setShowMoveVerify(false);
        setMoveVerifyInput('');
        setMoveVerifyError(false);
        setEngageState(false);
        setMissionBlockMessage(formatMissionBlockMessage(data));
        return;
      }
      if (data.error === 'mission_already_active') {
        setSelectedSite(null);
        setShowMoveConfirm(false);
        setShowMoveVerify(false);
        setMoveVerifyInput('');
        setMoveVerifyError(false);
        setMissionBlockMessage(
          data.message || '다른 목적지 운행이 이미 진행 중입니다.'
        );
      }
      if (
        data.error === 'stale_or_unowned_return'
        || data.error === 'stale_or_unowned_destination_stop'
        || data.error === 'return_not_at_service_wait'
      ) {
        // Restore the authoritative wait state after rejecting an optimistic
        // command from an older browser snapshot.
        setIsReturning(false);
        setMissionBlockMessage(
          data.message || '현재 운행과 일치하지 않는 이전 요청이 취소되었습니다.'
        );
        const waitSite = String(data.return_wait_site || '').trim();
        const waitOwner = String(data.return_wait_owner || '').trim();
        if (
          data.return_wait_active === true
          && SITE_NAMES.includes(waitSite)
          && ['operator', 'robot'].includes(waitOwner)
        ) {
          setArrivedSite(waitSite);
          setShowArrivalComplete(true);
        } else {
          setArrivedSite(null);
          setShowArrivalComplete(false);
        }
      }

      // HH_261002 - Preserve shared departure retry feedback without issuing
      // movement; an operator may select the failed destination again.
      if (data.departure_failed && data.mission_retryable) {
        const retrySite = String(data.mission_retry_site || '선택 사이트');
        const retryOwner = String(data.mission_retry_owner || '');
        setMissionBlockMessage(
          (!data.message || data.message === 'Drop-zone exit failed; select the destination again to retry')
            ? `${retrySite} 출차에 실패했습니다. 같은 사이트를 다시 선택해 주세요.`
            : data.message
        );
        if (retryOwner !== 'guest') {
          const cleared = {};
          SITE_NAMES.forEach(site => { cleared[site] = false; });
          setStates(cleared);
          setSelectedSite(null);
          setActiveRecallSite(null);
          setShowMoveConfirm(false);
          setShowMoveVerify(false);
          setMoveVerifyInput('');
          setMoveVerifyError(false);
          missionDispatchActiveRef.current = false;
        }
      }

      if ('charging_required' in data || 'parking_policy_mode' in data || 'parking_selected_method' in data) {
        setParkingPolicy(previous => ({
          charging_required: 'charging_required' in data ? Boolean(data.charging_required) : previous.charging_required,
          parking_policy_mode: data.parking_policy_mode ?? previous.parking_policy_mode,
          parking_selected_method: data.parking_selected_method ?? previous.parking_selected_method,
        }));
      }
      if ('battery_return_pending' in data || 'battery_return_urgent' in data
          || 'battery_return_started' in data || 'battery_return_waiting_for_user' in data) {
        const next = { ...batteryReturnStateRef.current };
        if ('battery_return_pending' in data) next.pending = Boolean(data.battery_return_pending);
        if ('battery_return_started' in data) next.started = Boolean(data.battery_return_started);
        if ('battery_return_waiting_for_user' in data) next.waitingForUser = Boolean(data.battery_return_waiting_for_user);
        if ('battery_return_urgent' in data) next.urgent = Boolean(data.battery_return_urgent);
        batteryReturnStateRef.current = next;
        setBatteryReturnState(next);
        setBatteryReturnMessage(next.pending || next.urgent ? formatBatteryReturnMessage({
          ...data,
          battery_return_started: next.started,
          battery_return_waiting_for_user: next.waitingForUser,
          battery_return_urgent: next.urgent,
        }) : '');
      }

      // HH_260904 - Preserve asynchronous re-dock progress after the Return
      // HTTP response. Charger release and CAN handoff can happen later.
      if (
        'redock_pending' in data
        || 'redock_waiting_for_can' in data
        || 'redock_status' in data
        || 'redock_message' in data
      ) {
        setRedockStatus(previous => {
          const pending = 'redock_pending' in data
            ? Boolean(data.redock_pending)
            : previous.pending;
          const waitingForCan = 'redock_waiting_for_can' in data
            ? Boolean(data.redock_waiting_for_can)
            : previous.waitingForCan;
          const hasExplicitStatus = 'redock_status' in data;
          const backendStatus = hasExplicitStatus
            ? String(data.redock_status || '').trim().toLowerCase()
            : '';
          const backendMessage = String(
            data.redock_message ?? data.message ?? ''
          ).trim();

          let status = previous.status || 'idle';
          if (waitingForCan) {
            status = 'waiting_for_can';
          } else if (pending) {
            status = 'waiting_for_disconnect';
          } else if (hasExplicitStatus) {
            status = backendStatus || 'idle';
          } else if (
            'redock_pending' in data || 'redock_waiting_for_can' in data
          ) {
            status = 'idle';
          }

          let message = '';
          if (waitingForCan || pending || hasExplicitStatus) {
            message = Object.prototype.hasOwnProperty.call(
              REDOCK_STATUS_MESSAGES, status
            ) ? REDOCK_STATUS_MESSAGES[status] : backendMessage;
          } else if (backendMessage) {
            // Compatibility for a backend that sends a terminal message but
            // no explicit status. Preserve it instead of guessing CAN recovery.
            message = backendMessage;
          }
          return { pending, waitingForCan, status, message, received: true };
        });
      }

      // The backend snapshot is the sole mission-admission authority shared
      // with Guest UI. Apply it before presentation-only site state.
      if ('mission_dispatch_active' in data) {
        const dispatchActive = Boolean(data.mission_dispatch_active);
        const dispatchIntent = String(data.mission_dispatch_intent || '');
        const dispatchSite = String(data.mission_dispatch_site || '');
        const dispatchGeneration = dispatchActive
          ? Number(data.mission_dispatch_generation || 0)
          : 0;
        const dispatchOwner = dispatchActive
          ? String(data.mission_dispatch_owner || '')
          : '';
        if (
          missionDispatchActiveRef.current !== dispatchActive
          || missionDispatchGenerationRef.current !== dispatchGeneration
          || missionDispatchSiteRef.current !== (dispatchActive ? dispatchSite : '')
          || missionDispatchOwnerRef.current !== dispatchOwner
        ) {
          missionAuthorityRevisionRef.current += 1;
        }
        missionDispatchActiveRef.current = dispatchActive;
        missionDispatchGenerationRef.current = dispatchGeneration;
        missionDispatchSiteRef.current = dispatchActive ? dispatchSite : '';
        missionDispatchOwnerRef.current = dispatchOwner;
        setMissionDispatch({
          active: dispatchActive,
          generation: dispatchGeneration,
          site: dispatchActive ? dispatchSite : '',
          owner: dispatchOwner,
          intent: dispatchActive ? dispatchIntent : '',
        });
        if (!dispatchActive || (dispatchOwner === 'guest' && dispatchIntent !== 'recall')) {
          setShowArrivalComplete(false);
        }
        // The guest notifications describe an admitted mission. Once the
        // backend reports no active identity they are stale regardless of
        // which service state (if any) accompanied the clear.
        if (!dispatchActive) {
          setShowGuestRecall(false);
          setGuestNavigateSite(null);
        }
        if (dispatchActive) {
          intentPinnedRef.current = false;
          setShowWaiting(false);
          if (dispatchIntent === 'recall' && SITE_NAMES.includes(dispatchSite)) {
            destinationIntentRef.current = 'recall';
            setDestinationIntent('recall');
            setActiveRecallSite(dispatchSite);
          } else if (dispatchIntent === 'delivery') {
            destinationIntentRef.current = 'delivery';
            setDestinationIntent('delivery');
            setActiveRecallSite(null);
          }
        } else if (!dispatchIntent && !intentPinnedRef.current) {
          destinationIntentRef.current = 'delivery';
          setDestinationIntent('delivery');
          setActiveRecallSite(null);
        }
      }

      // 초기 연결 시: {"states": {"B1": false, ...}} 전체 상태 수신
      if ('states' in data) {
        setStates(prev => ({ ...prev, ...data.states }));
      }
      // 개별 토글 변경 시: {"site": "B1", "state": true} 수신
      if ('site' in data && 'state' in data) {
        setStates(prev => ({ ...prev, [data.site]: data.state }));
      }
      if ('robot_recall_site' in data) {
        const replayedRecallSite = String(data.robot_recall_site || '');
        if (SITE_NAMES.includes(replayedRecallSite)) {
          destinationIntentRef.current = 'recall';
          setDestinationIntent('recall');
          setActiveRecallSite(replayedRecallSite);
          setSelectedSite(null);
          setShowWaiting(false);
        } else if (!intentPinnedRef.current) {
          destinationIntentRef.current = 'delivery';
          setDestinationIntent('delivery');
          setActiveRecallSite(null);
        }
      }
      // HH_260723 - Apply perception occupancy before allowing a campsite selection.
      if ('occupied_sites' in data && Array.isArray(data.occupied_sites)) {
        setOccupiedSites(prev => (
          prev.length === data.occupied_sites.length
          && prev.every((site, index) => site === data.occupied_sites[index])
        ) ? prev : data.occupied_sites);
        // A tent blocks delivery into a campsite, but is the expected target
        // of a roadside recall. Do not clear a recall selection on occupancy.
        setSelectedSite(prev => (
          destinationIntentRef.current === 'delivery'
          && data.occupied_sites.includes(prev)
        ) ? null : prev);
      }
      // HH_260721 - Handle arrival and lifecycle updates through the shared service contract.
      if ('arrived' in data) {
        setArrivedSite(data.arrived);
        setShowArrivalComplete(
          missionDispatchOwnerRef.current !== 'guest'
          || destinationIntentRef.current === 'recall'
        );
      }
      // Service lifecycle: 0=drop-zone wait, 6=site unload wait, 9/10=returning.
      if ('service_state' in data) {
        const serviceState = Number(data.service_state);
        const nextStateName = data.service_state_name || SERVICE_STATE_NAME_BY_ID[serviceState];
        if (nextStateName) setServiceStateName(nextStateName);
        const previousServiceState = serviceStateIdRef.current;
        serviceStateIdRef.current = serviceState;
        // Backend also sends minimal {service_state, returning} frames. Those
        // acknowledge the same state and must not erase its clearance phase.
        if ('service_state_description' in data) {
          setServiceStateDescription(String(data.service_state_description || ''));
        } else if (previousServiceState !== serviceState) {
          setServiceStateDescription('');
        }
        if (serviceState === SERVICE_STATE.DROP_ZONE_WAIT) {
          if (RETURN_COMPLETION_SOURCE_STATES.has(previousServiceState)) {
            setReturnCompletionPending(true);
          }
        } else if (!RETURN_COMPLETION_SOURCE_STATES.has(serviceState)) {
          setReturnCompletionPending(false);
        }
        // HH_260908 - Arrival ends the "moving to the guest" announcement:
        // the loading-wait / return / standby screens own the display now.
        if (!GUEST_RECALL_ENROUTE_STATES.has(serviceState)) {
          setShowGuestRecall(false);
          setGuestNavigateSite(null);
        }
        if (
          serviceState === SERVICE_STATE.DROP_ZONE_WAIT
          || serviceState === SERVICE_STATE.WAITING_FOR_CHARGING
          || serviceState === SERVICE_STATE.CHARGING
        ) {
          // HH_260721 - Charging remains a stopped standby state with destination selection enabled.
          setArrivedSite(null);
          setShowArrivalComplete(false);
          setIsReturning(false);
          // These stationary states are also emitted while an accepted Recall
          // waits for departure. Do not reopen standby until the backend has
          // explicitly cleared the mission identity.
          if (
            destinationIntentRef.current !== 'recall'
            && !missionDispatchActiveRef.current
            && !intentPinnedRef.current
          ) {
            setShowWaiting(true);
          }
          setShowGuestRecall(false);
          setGuestNavigateSite(null);
        } else if (serviceState === SERVICE_STATE.OPERATOR_STOPPED) {
          // HH_260724 - Operator stop/cancel clears mission selection but remains visible as service state.
          const cleared = {};
          SITE_NAMES.forEach(site => { cleared[site] = false; });
          setStates(cleared);
          setSelectedSite(null);
          setShowMoveConfirm(false);
          setShowMoveVerify(false);
          setMoveVerifyInput('');
          setMoveVerifyError(false);
          setArrivedSite(null);
          setShowArrivalComplete(false);
          setIsReturning(false);
          setShowGuestRecall(false);
          setGuestNavigateSite(null);
          setActiveRecallSite(null);
          if (
            missionDispatchActiveRef.current
            || missionDispatchGenerationRef.current !== 0
            || missionDispatchSiteRef.current
            || missionDispatchOwnerRef.current
          ) {
            missionAuthorityRevisionRef.current += 1;
          }
          missionDispatchActiveRef.current = false;
          missionDispatchGenerationRef.current = 0;
          missionDispatchSiteRef.current = '';
          missionDispatchOwnerRef.current = '';
          setMissionDispatch({
            active: false,
            generation: 0,
            site: '',
            owner: '',
            intent: '',
          });
        } else if (MOVING_SERVICE_STATES.has(serviceState)) {
          // HH_260724 - Once backend has accepted motion, do not leave the confirmation preview open.
          setSelectedSite(null);
          setShowMoveConfirm(false);
          setShowMoveVerify(false);
          setMoveVerifyInput('');
          setMoveVerifyError(false);
        } else if (ARRIVAL_STATES.has(serviceState) && data.site) {
          setArrivedSite(data.site);
          setShowArrivalComplete(
            missionDispatchOwnerRef.current !== 'guest'
            || (destinationIntentRef.current === 'recall'
              && serviceState === SERVICE_STATE.GUEST_LOADING_WAIT)
          );
          setIsReturning(false);
        } else if (RETURNING_STATES.has(serviceState) || data.returning) {
          // HH_261002 - Recall uses RETURN_WITH_CARGO during the turnaround
          // and RECALL_RETURN_WAIT. Retain the arrival identity for the second
          // confirmation; hide its modal only while final-ready is false.
          const recallCompletionStage = (
            serviceState === SERVICE_STATE.RETURN_WITH_CARGO
            && destinationIntentRef.current === 'recall'
          );
          if (!recallCompletionStage) {
            setShowArrivalComplete(false);
            setArrivedSite(null);
          }
          setIsReturning(true);
        }
      }
      if ('system_health' in data) {
        setSystemHealth(String(data.system_health || 'STARTING').toUpperCase());
      }
      if ('mission_phase' in data) {
        setMissionPhase(
          String(data.mission_phase || 'INITIALIZING').toUpperCase()
        );
      }
      if ('mission_source' in data) {
        setMissionSource(String(data.mission_source || 'none').toLowerCase());
      }
      // HJ_260601: 게스트 호출 알림: {"guest_recall": true} 수신
      if (data.guest_recall) {
        setShowGuestRecall(true);
      }
      // HH_260708 - Receive guest-site navigation notifications from the backend.
      if (data.guest_navigate) {
        setGuestNavigateSite(data.guest_navigate);
      }
      // HH_260708 - Mirror backend battery percentage for the operator header.
      if ('battery' in data) {
        setBatteryPct(data.battery);
      } else if ('battery_percentage' in data) {
        const snapshotBattery = Number(data.battery_percentage);
        setBatteryPct(Number.isFinite(snapshotBattery) ? snapshotBattery : null);
      }
      // HH_261002 - Keep the full-charge notice tied to its explicit backend
      // signal; a percent reading or CHARGING state alone cannot prove completion.
      if ('battery_charge_complete' in data) {
        setBatteryChargeComplete(Boolean(data.battery_charge_complete));
      }
      // HH_260708 - Mirror planning engage state broadcast by the backend.
      if ('engage' in data) {
        setEngageState(data.engage);
      } else if ('engaged' in data) {
        setEngageState(data.engaged);
      }
      // HH_260708 - Mirror exterior headlight state from the platform light bridge.
      if ('headlight' in data) {
        setHeadlightState(data.headlight);
      }
      if (missionExecutionErrorRef.current) setShowArrivalComplete(false);
      if ('service_state' in data && !ARRIVAL_STATES.has(Number(data.service_state))) {
        returnRequestPendingRef.current = false;
        setReturnRequestPending(false);
      }
    };

    // HH_260708 - Reconnect the operator WebSocket after transient disconnects.
    ws.onclose = () => {
      if (
        wsGenerationRef.current !== connectionGeneration
        || wsRef.current !== ws
      ) return;
      wsRef.current = null;
      setConnected(false);
      setBatteryChargeComplete(false);
      returnRequestPendingRef.current = false;
      setReturnRequestPending(false);
      if (wsMountedRef.current) {
        wsReconnectTimerRef.current = setTimeout(() => {
          wsReconnectTimerRef.current = null;
          connect();
        }, 2000);
      }
    };

    ws.onerror = () => ws.close();
  }, []);

  // ── 컴포넌트 마운트/언마운트 시 WebSocket 관리 ──────────────────────────
  useEffect(() => {
    if (previewMode) return undefined;
    wsMountedRef.current = true;
    connect();
    return () => {
      wsMountedRef.current = false;
      wsGenerationRef.current += 1;
      if (wsReconnectTimerRef.current !== null) {
        clearTimeout(wsReconnectTimerRef.current);
        wsReconnectTimerRef.current = null;
      }
      const socket = wsRef.current;
      wsRef.current = null;
      if (socket) socket.close();
    };
  }, [connect, previewMode]);

  useEffect(() => {
    if (showWaiting) {
      // HH_260810 - Keep an authenticated diagnostic session mounted while the
      // service UI moves between waiting, driving, arrival, and return screens.
      // Informational overlays still close when the public waiting screen wins.
      setActiveModal(current => current === 'settings' ? current : null);
      setLoginError('');
    }
  }, [showWaiting]);

  // ── 개별 사이트 토글 핸들러 ─────────────────────────────────────────────
  const handleEngage = () => {
    if (previewMode) return;
    // HH_261102 - A stale service control must not open a competing drive owner during cargo access.
    if (chargingAccessLocked && !engageState) return;
    if (engageState) {
      // HH_260724 - ENGAGE OFF is an operator stop, so route it through the full backend stop path.
      fetch('/ui/stop', { method: 'POST' })
        .then((res) => { if (res.ok) setEngageState(false); })
        .catch(() => {});
      return;
    }
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ engage: true }));
    }
  };

  const handleManualStop = () => {
    if (previewMode) return;
    // HH_260724 - Manual driving stop must cancel Nav2 and publish OPERATOR_STOPPED.
    fetch('/ui/stop', { method: 'POST' }).catch(() => {});
    setEngageState(false);
  };

  // HH_260708 - Send the headlight toggle through the UI command endpoint.
  const handleHeadlight = () => {
    if (previewMode) return;
    const next = !headlightState;
    fetch(`/ui/headlight?value=${next}`, { method: 'POST' })
      .then((res) => { if (res.ok) setHeadlightState(next); })
      .catch(() => {});
  };

  const selectDestinationIntent = (intent) => {
    if (intent !== 'delivery' && intent !== 'recall') return;
    destinationIntentRef.current = intent;
    setDestinationIntent(intent);
    setSelectedSite(null);
    setShowMoveConfirm(false);
    setShowMoveVerify(false);
    setMoveVerifyInput('');
    setMoveVerifyError(false);
    setMissionBlockMessage('');
  };

  const activateDestinationService = (intent) => {
    if (chargingAccessLocked) return;
    selectDestinationIntent(intent);
    intentPinnedRef.current = true;
    setTogglePage(0);
    setShowServiceSelection(false);
  };

  const handleServiceDocking = async () => {
    if (previewMode) return;
    if (
      chargingAccessLocked
      || serviceDockingPendingRef.current
      || !dockingAllowedAtServiceState(serviceStateName)
    ) {
      return;
    }
    setShowDockingConfirm(false);
    serviceDockingPendingRef.current = true;
    setServiceDockingPending(true);
    setServiceDockingStatus('');
    try {
      const body = await postDockingRequest();
      setServiceDockingStatus(body.message || '충전 요청이 접수되었습니다.');
    } catch (error) {
      setServiceDockingStatus(error.message || '충전 요청 실패');
    } finally {
      serviceDockingPendingRef.current = false;
      setServiceDockingPending(false);
    }
  };

  const requestCampingSiteRecall = async (site) => {
    if (chargingAccessLocked) return;
    if (previewMode) return;
    const requestEpoch = recallRequestEpochRef.current + 1;
    recallRequestEpochRef.current = requestEpoch;
    const authorityRevisionAtRequest = missionAuthorityRevisionRef.current;
    setRecallRequestPending(true);
    setMissionBlockMessage('');
    try {
      // This dedicated endpoint is intentionally not replaced with
      // /ui/destination on failure: that would enter the campsite as delivery.
      const response = await fetch(
        `/ui/camping_site_recall?site=${encodeURIComponent(site)}&intent=recall`,
        { method: 'POST' }
      );
      const body = await response.json().catch(() => ({}));
      if (!response.ok || !body.success) {
        const unavailable = response.status === 404
          ? '이용객 호출 기능이 아직 백엔드에 연결되지 않았습니다.'
          : '이용객 호출 요청이 거부되었습니다.';
        throw new Error(body.message || unavailable);
      }
      // Require an intent echo so a miswired ordinary destination endpoint
      // cannot be presented to the operator as a safe roadside recall.
      if (body.intent !== 'recall') {
        throw new Error('백엔드가 호출 전용 응답을 확인하지 않았습니다.');
      }
      const admittedGeneration = Number(
        body.mission_dispatch_generation || 0
      );
      const admittedSite = String(
        body.mission_dispatch_site || ''
      ).trim();
      const admittedOwner = String(
        body.mission_dispatch_owner || ''
      ).trim();
      if (
        body.mission_dispatch_active !== true
        || admittedSite !== site
        || admittedOwner !== 'robot'
        || !Number.isSafeInteger(admittedGeneration)
        || admittedGeneration <= 0
      ) {
        throw new Error('백엔드가 호출 운행 세대를 확정하지 않았습니다.');
      }
      const currentAuthorityMatches = Boolean(
        missionDispatchActiveRef.current
        && missionDispatchSiteRef.current === admittedSite
        && missionDispatchOwnerRef.current === admittedOwner
        && missionDispatchGenerationRef.current === admittedGeneration
      );
      if (
        recallRequestEpochRef.current !== requestEpoch
        || (
          missionAuthorityRevisionRef.current !== authorityRevisionAtRequest
          && !currentAuthorityMatches
        )
      ) {
        throw new Error(
          '요청 처리 중 운행 권한이 변경되어 이전 호출 응답을 무시했습니다.'
        );
      }
      setActiveRecallSite(site);
      missionDispatchActiveRef.current = true;
      missionDispatchSiteRef.current = admittedSite;
      missionDispatchOwnerRef.current = admittedOwner;
      missionDispatchGenerationRef.current = admittedGeneration;
      setMissionDispatch({
        active: true,
        generation: admittedGeneration,
        site: admittedSite,
        owner: admittedOwner,
        intent: 'recall',
      });
      setArrivedSite(null);
      setShowArrivalComplete(false);
      setIsReturning(false);
      setSelectedSite(null);
      return true;
    } catch (error) {
      if (recallRequestEpochRef.current === requestEpoch) {
        setMissionBlockMessage(
          error.message || '이용객 호출 요청을 전송하지 못했습니다.'
        );
      }
      return false;
    } finally {
      if (recallRequestEpochRef.current === requestEpoch) {
        setRecallRequestPending(false);
      }
    }
  };

  const handleToggle = (site) => {
    if (chargingAccessLocked) return;
    // HJ_261118 - Placeholder (planned) sites are display-only.
    if (isPlannedSite(site)) return;
    // HH_260723 - Occupancy blocks physical campsite entry, not the typed
    // roadside recall used when a tent is already installed at the site.
    if (
      destinationIntent === 'delivery'
      && occupiedSites.includes(site)
    ) {
      return;
    }
    if (activeRecallSite || recallRequestPending) return;
    if (states[site]) {
      // 이미 ON → OFF 불가, 아무것도 하지 않음
      return;
    }
    if (anyOn) {
      // 다른 사이트가 ON 중 → 선택 불가
      return;
    }
    // OFF → 이미지 프리뷰 표시 (아직 publish 안 함)
    setSelectedSite(site);
    setArrivedSite(null);
  };

  // ── 프리뷰에서 "Yes" 클릭 → 실제 ON publish ─────────────────────────────
  const handleConfirmMove = async () => {
    if (previewMode) return;
    if (chargingAccessLocked) return;
    if (!selectedSite) return;
    if (destinationIntent === 'recall') {
      await requestCampingSiteRecall(selectedSite);
      return;
    }
    if (!occupiedSites.includes(selectedSite)) {
      applyToggle(selectedSite, true);
      setSelectedSite(null);
    }
  };

  // ── 이동중 "Yes" 클릭 → 운행 정지 (OFF) ─────────────────────────────────
  const handleStopMove = () => {
    if (previewMode) return;
    // Return, docking, and parking can continue after the active-site toggle
    // has already cleared. Use the authoritative full stop for every visible
    // service-motion stop button so all motion owners are cancelled together.
    fetch('/ui/stop', { method: 'POST' }).catch(() => {});
    setEngageState(false);
  };

  const requestStopConfirmation = (kind) => {
    if (previewMode) return;
    setStopConfirmation({ kind, revision: missionAuthorityRevisionRef.current });
  };

  const confirmOperatorStop = () => {
    const pending = stopConfirmation;
    setStopConfirmation(null);
    // HH_261001 - A delayed tap must not stop a different or ended mission.
    if (!pending || pending.revision !== missionAuthorityRevisionRef.current) return;
    if (pending.kind === 'manual') {
      if (manualDriveActive) handleManualStop();
    } else if (activeStateSite || activeRecallSite || displayedReturning) {
      handleStopMove();
    }
  };

  // ── 이용 완료 버튼 클릭 → state=3(RETURNING) publish 요청 ──────────────
  const requestArrivalComplete = () => {
    if (previewMode || returnRequestPending) return;
    setArrivalConfirmOpen(true);
  };
  const confirmArrivalComplete = () => {
    setArrivalConfirmOpen(false);
    handleArrivalComplete();
  };
  const handleArrivalComplete = () => {
    if (previewMode) return;
    if (missionExecutionErrorRef.current) {
      setMissionBlockMessage(missionExecutionFailureMessage(missionExecutionErrorRef.current));
      return;
    }
    if (returnRequestPendingRef.current) return;
    if (!robotOwnsReturn) {
      setMissionBlockMessage('현재 운행의 복귀 권한이 이 화면에 없습니다.');
      return;
    }
    if (!wsRef.current || wsRef.current.readyState !== WebSocket.OPEN) {
      setMissionBlockMessage('로봇 연결을 확인하고 다시 눌러주세요.');
      return;
    }
    // Keep site+generation binding for both locally and Guest-admitted recall.
    // Progress changes only when the controller's service state is received.
    returnRequestPendingRef.current = true;
    setReturnRequestPending(true);
    wsRef.current.send(JSON.stringify({
      usage_complete: true,
      site: missionDispatchSiteRef.current,
      mission_generation: missionDispatchGenerationRef.current,
      recall_final_return: recallFinalReturnReady,
    }));
    // Keep the completion window open until the backend confirms the next
    // service phase. For a call mission this covers campsite re-entry and the
    // turnaround that precedes the final loading-complete confirmation.
  };

  // ── 실제 토글 적용 & WebSocket 전송 ─────────────────────────────────────
  const applyToggle = (site, newState) => {
    if (chargingAccessLocked) return;
    if (previewMode) return;
    setMissionBlockMessage('');
    const updated = {};
    SITE_NAMES.forEach(s => { updated[s] = false; });
    if (newState) updated[site] = true;

    setStates(updated);

    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      SITE_NAMES.forEach(s => {
        const st = updated[s];
        if (st !== states[s]) {
          wsRef.current.send(JSON.stringify({
            site: s,
            state: st,
            mission_generation: st ? 0 : missionDispatchGenerationRef.current,
          }));
        }
      });
    }
  };

  // ── JSX 렌더링 ─────────────────────────────────────────────────────────
  const currentBatteryPolicy = batteryPolicyStatus(batteryPct, batteryReturnState);
  // HH_261002 - The idle home map is opened only by a visible user action.
  // It reuses the read-only driving view without starting or altering a mission.
  const idleMapPanel = !missionDispatch.active && drivingDisplay.visible && (
    <div className="idle-map-panel" data-ui="idle-navigation-map">
      <LiveDrivingDisplay feed={drivingDisplay.feed} store={drivingDisplay.store} onDismiss={drivingDisplay.dismiss}
        onToggleTheme={previewMode ? undefined : () => setDrivingTheme(current => current === 'light' ? 'dark' : 'light')}
        systemHealth={systemHealth} batteryPolicy={currentBatteryPolicy}
        embedded={false} demo={previewMode} theme={previewMode ? drivingPreviewTheme : drivingTheme} />
    </div>
  );
  const missionExecutionWarning = missionExecutionError && (
    <div role="alert" className="mission-block-msg" style={{ color: '#b42318', padding: '1rem', background: '#fff0ee' }}>
      <p>{t(missionExecutionFailureMessage(missionExecutionError))}</p>
      <details data-ui="operator-mission-error-details">
        <summary>{t('상세 원인')}</summary>
        <p style={{ overflowWrap: 'anywhere' }}>{missionExecutionError}</p>
      </details>
    </div>
  );
  const missionSelectionLocked = Boolean(
    anyOn || isReturning || activeRecallSite || recallRequestPending || chargingAccessLocked
  );
  // HJ_261118 - 키오스크는 여러 손님이 차례로 쓰므로 영어 선택을 다음 손님에게 남기지 않는다:
  // 서비스(미션)가 끝나면 한국어로 되돌린다. 대기 화면 무입력으로는 되돌리지 않는다(운영 요청).
  const langMissionActiveRef = useRef(missionDispatch.active);
  useEffect(() => {
    if (langMissionActiveRef.current && !missionDispatch.active) resetLang();
    langMissionActiveRef.current = missionDispatch.active;
  }, [missionDispatch.active]);

  // HJ_261118 - Never leave the arrival confirmation open for a stale arrival.
  useEffect(() => {
    setArrivalConfirmOpen(false);
  }, [arrivedSite, recallFinalReturnReady, recallTurnaroundInProgress, robotOwnsReturn]);
  // HJ_261118 - Keep the zone tab/page on the site the visitor or mission is about.
  const focusedSite = selectedSite || activeSite || activeRecallSite || arrivedSite;
  useEffect(() => {
    if (!focusedSite || !SITE_NAMES.includes(focusedSite)) return;
    setSiteZoneTab(siteZone(focusedSite));
    setTogglePage(sitePageInZone(focusedSite));
  }, [focusedSite]);
  const modalData = SIDE_BUTTONS.find(b => b.id === activeModal);
  const serviceEvidenceModalOpen = activeModal === 'service-evidence';
  const serviceEvidenceModal = serviceEvidenceModalOpen ? (
    <div className="modal-overlay">
      <div
        className="modal-box service-evidence-modal"
        onClick={e => e.stopPropagation()}
      >
        <div className="modal-header">
          <span className="modal-title">실증 운행 현황</span>
          <button className="modal-back-btn" onClick={() => setActiveModal(null)}>뒤로가기</button>
        </div>
        <div className="modal-body">
          <ServiceEvidenceDashboard
            summaryData={serviceMetrics.data}
            summaryLoading={serviceMetrics.loading}
            summaryError={serviceMetrics.error}
          />
        </div>
      </div>
    </div>
  ) : null;
  const adminEntryZone = (
    <div
      data-ui="operator-admin-entry"
      className="diag-secret-zone diag-secret-zone-global"
      onMouseDown={handleDiagPressStart}
      onMouseUp={handleDiagPressEnd}
      onMouseLeave={handleDiagPressEnd}
      onTouchStart={handleDiagPressStart}
      onTouchEnd={handleDiagPressEnd}
      onTouchCancel={handleDiagPressEnd}
    >
      {diagPressProgress > 0 && (
        <div className="diag-press-progress-wrap">
          <div
            className="diag-press-bar"
            style={{ transform: `scaleY(${diagPressProgress / 100})` }}
          />
        </div>
      )}
    </div>
  );

  // HH_260810 - Admin diagnostics are a top-level operator surface. Keeping
  // this branch independent of showWaiting prevents arrival/service updates
  // from tearing down telemetry subscriptions while an operator is observing.
  if (showLoginModal || activeModal === 'settings') {
    return (
      <div className="admin-runtime-shell waiting-theme-forest admin-theme-forest">
        {showLoginModal ? (
          <div className="modal-overlay">
            <div className="modal-box login-modal-box" data-ui="operator-admin-login-modal" onClick={e => e.stopPropagation()}>
              <div className="modal-header">
                <span className="modal-title">진단 관리자 인증</span>
                <button className="modal-back-btn" data-ui="operator-admin-login-cancel" onClick={() => setShowLoginModal(false)}>뒤로가기</button>
              </div>
              <div className="modal-body login-modal-body">
                <div className="login-fields-row">
                  <div className="login-field">
                    <label>아이디</label>
                    <input
                      type="text"
                      data-ui="operator-admin-id"
                      value={loginId}
                      onChange={e => setLoginId(e.target.value)}
                      onFocus={() => setActiveField('id')}
                      onKeyDown={e => { if (e.key === 'Enter') handleLogin(); }}
                      placeholder="아이디를 입력하세요"
                    />
                  </div>
                  <div className="login-field">
                    <label>비밀번호</label>
                    <input
                      id="login-pw-input"
                      type="password"
                      data-ui="operator-admin-password"
                      value={loginPw}
                      onChange={e => setLoginPw(e.target.value)}
                      onFocus={() => setActiveField('pw')}
                      onKeyDown={e => { if (e.key === 'Enter') handleLogin(); }}
                      placeholder="비밀번호를 입력하세요"
                    />
                  </div>
                </div>
                <button className="login-submit-btn" data-ui="operator-admin-login" onClick={handleLogin}>확인</button>
                {loginError && <p className="login-error">{loginError}</p>}
                <div className="vkb-wrap">
                  <div className="vkb-field-tabs">
                    <button className={`vkb-tab${activeField === 'id' ? ' active' : ''}`} onClick={() => setActiveField('id')}>아이디</button>
                    <button className={`vkb-tab${activeField === 'pw' ? ' active' : ''}`} onClick={() => setActiveField('pw')}>비밀번호</button>
                  </div>
                  {KB_ROWS.map((row, ri) => (
                    <div key={ri} className="vkb-row">
                      {/* HH_261024 - Both Shift keys need distinct React identities. */}
                      {row.map((key, ki) => (
                        <button
                          key={`${ri}-${ki}`}
                          className={`vkb-key${key === '⌫' ? ' vkb-wide' : ''}${key === '⇧' ? ' vkb-wide' + (kbCaps ? ' vkb-caps-on' : '') : ''}`}
                          onClick={() => handleVirtualKey(key)}
                        >
                          {key === '⇧' ? (kbCaps ? '⇧ ON' : '⇧') : key}
                        </button>
                      ))}
                    </div>
                  ))}
                  <div className="vkb-row">
                    <button className="vkb-key vkb-space" onClick={() => handleVirtualKey('SPACE')}>SPACE</button>
                  </div>
                </div>
              </div>
            </div>
          </div>
        ) : (
          <div className="modal-overlay">
            <div className="modal-box" onClick={e => e.stopPropagation()}>
              <div className="modal-header">
                <span className="modal-title">{modalData?.title || '진단'}</span>
                <button className="modal-back-btn" data-ui="operator-admin-exit" onClick={() => setActiveModal(null)}>뒤로가기</button>
              </div>
              <div className="modal-body">
                <DiagnosticsMonitor
                  adminLoginId={loginId}
                  adminLoginPw={loginPw}
                  redockStatus={redockStatus}
                  parkingPolicy={parkingPolicy}
                  serviceStateName={serviceStateName}
                  engageState={engageState}
                  engageDisabled={isReturning}
                  onToggleEngage={handleEngage}
                  headlightState={headlightState}
                  onToggleHeadlight={handleHeadlight}
                />
              </div>
            </div>
          </div>
        )}
      </div>
    );
  }

  // HH_261102 - Keep the stopped drop_zone status visible, but let ready use the existing service chooser.
  const chargingPhase = chargingAccess.snapshot?.phase;
  // HJ_261105 - Operator stop/backend restart publishes OPERATOR_STOPPED while the robot stays on the
  // charger; the backend charging contact (same fact that gates can_leave) keeps cargo access reachable.
  // After the staging move, ready stays on the charging screen until the measured stop has been shown
  // for CARGO_READY_NOTICE_MS; from then on the robot is in ordinary drop_zone standby.
  const chargingContact = chargingAccess.snapshot?.platform_is_charging === true;
  const chargingStandby = chargingContact
    || (chargingPhase === 'ready' && !(cargoAccessReady && cargoReadyNoticeExpired))
    || ['CHARGING', 'WAITING_FOR_CHARGING'].includes(serviceStateName);
  if (!previewMode && (chargingAccessLocked || chargingPhase === 'error'
    || (showWaiting && !activeModal && !missionDispatch.active
      && chargingStandby))) {
    return <ChargingAccessScreen access={chargingAccess} batteryChargeComplete={batteryChargeComplete}
      onServices={handleWaitingClick} onStop={handleStopMove} adminEntry={adminEntryZone} />;
  }

  // HJ_261118 - Shared forest-theme header for the home and service-selection
  // screens: logo/title, battery pill and clock (WiFi and the idle map entry
  // were removed at the operator's request).
  const renderForestHeader = ({ admissionStatus = null } = {}) => (
    <>
      <div className="waiting-header">
        <div className="wh-content">
          <div className="wh-left">
            <div className="wh-logo">
              <img src="/월악산_국립공원_로고.jpg" alt="월악산 국립공원 로고" />
            </div>
            <div className="wh-title-block">
              <span className="wh-subtitle">{t('국립공원공단 · Woraksan National Park')}</span>
              <span className="wh-main-title">
                {lang === 'en' ? t('월악산 국립공원 배송 로봇') : <>월악산 <em>국립공원</em> 배송 로봇</>}
              </span>
              {admissionStatus}
            </div>
          </div>
          <div className="wh-right-group">
            <LanguageToggle />
            <div className="wh-battery wh-battery-pill">
              <BatteryIcon pct={batteryPct} />
              <span className="wh-battery-label">
                {batteryPct === null ? '–%' : `${batteryPct}%`}
              </span>
            </div>
            <LiveClock />
          </div>
        </div>
      </div>
      <div className="waiting-header-accent" aria-hidden="true" />
    </>
  );

  // 대기 화면: 모든 토글 OFF 상태일 때 표시, 클릭/터치 시 토글 화면으로 전환
  if (showWaiting) {
    return (
      <div className="waiting-screen waiting-theme-forest" data-ui="operator-waiting-screen">
        {/* ── 상단 헤더 배너 ── */}
        {renderForestHeader({ admissionStatus: guestAdmissionStatus })}

        <WaitingRuntimeStatusPanel
          systemHealth={systemHealth}
          missionPhase={missionPhase}
          batteryPolicy={currentBatteryPolicy}
          parkingPolicy={parkingPolicy}
          serviceStateName={serviceStateName}
          serviceStateDescription={serviceStateDescription}
          batteryChargeComplete={batteryChargeComplete}
        />

        {/* ── 하단 콘텐츠 영역: 실증 요약 + 기존 4개 버튼 2×2 ── */}
        <div className="waiting-body">
          {missionExecutionWarning}

          {/* ── 공개 실증 운행 요약: 기존 2×2 서비스 버튼은 그대로 유지 ── */}
          <ServiceEvidenceSummary
            data={serviceMetrics.data}
            loading={serviceMetrics.loading}
            error={serviceMetrics.error}
            onOpen={() => setActiveModal('service-evidence')}
          />

          {/* ── HJ_261118 - 서비스 영역: 왼쪽 서비스 선택(대표 카드) + 오른쪽 안내 3종 ── */}
          <div className="waiting-services">
            <button className="waiting-hero" data-ui="operator-open-destination" onClick={handleWaitingClick}>
              <span className={`waiting-hero-ready${operatingNow ? '' : ' closed'}`}>
                <i aria-hidden="true" />
                {t(operatingNow ? '지금 바로 출발할 수 있어요' : '현재는 운영시간이 아닙니다')}
              </span>
              <span className="waiting-hero-title">{t('서비스 선택')}</span>
              {outsideHoursMsg
                ? <span className="waiting-hero-desc outside-hours">{t('현재는 운영시간이 아닙니다.')}</span>
                : <span className="waiting-hero-desc">{t('캠핑 사이트까지 무거운 짐을')}<br />{t('로봇이 대신 옮겨드려요.')}</span>
              }
              <span className="waiting-hero-go">
                {t('시작하기')}
                <svg viewBox="0 0 24 24" aria-hidden="true"><path d="M5 12h14M13 6l6 6-6 6" /></svg>
              </span>
            </button>

            <div className="waiting-side">
              {SIDE_BUTTONS.filter(btn => HOME_CARD_ART[btn.id]).map(btn => (
                <button
                  key={btn.id}
                  className="waiting-side-card"
                  data-ui={`operator-open-${btn.id}`}
                  onClick={e => {
                    e.stopPropagation();
                    setActiveModal(btn.id);
                  }}
                >
                  <span className="waiting-side-art">
                    <img src={`${process.env.PUBLIC_URL}/${HOME_CARD_ART[btn.id].image}`} alt="" />
                  </span>
                  <span className="waiting-side-text">
                    <strong>{t(btn.label)}</strong>
                    <small>{t(HOME_CARD_ART[btn.id].description)}</small>
                  </span>
                  <svg className="waiting-side-chevron" viewBox="0 0 24 24" aria-hidden="true"><path d="m9 6 6 6-6 6" /></svg>
                </button>
              ))}
            </div>
          </div>

          {/* ── 숨겨진 진단 진입 영역 (모든 서비스 화면 공통) ── */}
          {adminEntryZone}

        </div>

        {/* ── 모달 오버레이 ── */}
        {idleMapPanel}
        {serviceEvidenceModal}

        {activeModal && activeModal !== 'settings' && modalData && (
          <div className="modal-overlay">
            <div className="modal-box" onClick={e => e.stopPropagation()}>
              <div className="modal-header">
                <span className="modal-title">{t(modalData.title)}</span>
                <button className="modal-back-btn" onClick={() => setActiveModal(null)}>{t('뒤로가기')}</button>
              </div>
              <div className="modal-body">{modalData.content}</div>
            </div>
          </div>
        )}

        {batteryReturnMessage && (
          <div className="move-confirm-overlay mission-block-overlay" onClick={() => setBatteryReturnMessage('')}>
            <div className="move-confirm-box mission-block-box" onClick={e => e.stopPropagation()}>
              <p className="move-confirm-msg mission-block-msg">{t(batteryReturnMessage)}</p>
              <div className="move-confirm-btns">
                <button
                  className="move-confirm-no mission-block-close"
                  onClick={() => setBatteryReturnMessage('')}
                >
                  {t('확인')}
                </button>
              </div>
            </div>
          </div>
        )}
      </div>
    );
  }

  if (showServiceSelection) {
    const dockingAvailable = dockingAllowedAtServiceState(serviceStateName);
    return (
      <div className="main-layout waiting-theme-forest" data-ui="operator-service-selection-screen">
        {adminEntryZone}

        {renderForestHeader()}

        {/* 대기 화면과 같은 운행 상태 배너를 서비스 선택 화면에서도 유지한다. */}
        <WaitingRuntimeStatusPanel
          systemHealth={systemHealth}
          missionPhase={missionPhase}
          batteryPolicy={currentBatteryPolicy}
          parkingPolicy={parkingPolicy}
          serviceStateName={serviceStateName}
          serviceStateDescription={serviceStateDescription}
          batteryChargeComplete={batteryChargeComplete}
        />

        <main className="service-selection-body">
          <div className="service-selection-heading">
            {/* HH_261102 - Keep familiar service names and a single short purpose per card. */}
            <h1>{t('어떤 서비스를 이용하시겠어요?')}</h1>
          </div>

          <div className="service-selection-grid">
            <button
              type="button"
              className="service-choice-card service-choice-delivery"
              data-ui="operator-intent-delivery"
              // HJ_261118 - Delivery goes straight to site selection (confirm dialog removed).
              onClick={() => activateDestinationService('delivery')}
            >
              <span className="service-choice-icon" aria-hidden="true">{SERVICE_ICONS.delivery}</span>
              <strong>{t('배달 서비스')}</strong>
              <span>{t('캠핑 사이트 안으로 짐을 배달해요.')}</span>
            </button>

            <button
              type="button"
              className="service-choice-card service-choice-recall"
              data-ui="operator-intent-recall"
              // HJ_261118 - Recall goes straight to site selection (confirm dialog removed).
              onClick={() => activateDestinationService('recall')}
            >
              <span className="service-choice-icon" aria-hidden="true">{SERVICE_ICONS.recall}</span>
              <strong>{t('호출 서비스')}</strong>
              <span>{t('로봇을 사이트 앞 도로로 불러요.')}</span>
            </button>

            <div className="service-selection-dock-wrap">
              <button
                type="button"
                className="service-choice-card service-choice-docking"
                data-ui="operator-service-docking"
                disabled={!dockingAvailable || serviceDockingPending}
                onClick={() => {
                  setServiceDockingStatus('');
                  setShowDockingConfirm(true);
                }}
              >
                {/* HH_260909 - Operators read "도킹" as jargon. The card, its
                    confirm dialog and its status line now all say 충전; the
                    data-ui hook and CSS class keep the docking identifier. */}
                <span className="service-choice-icon" aria-hidden="true">{SERVICE_ICONS.docking}</span>
                <strong>{t(serviceDockingPending ? '충전 요청 중' : '충전')}</strong>
                <span>
                  {dockingAvailable
                    ? t('관리자용 · 충전 시작')
                    : t('관리자용 · 주차 위치에서 이용')}
                </span>
              </button>
              {serviceDockingStatus && (
                <span className="manual-motion-status" role="status">
                  {serviceDockingStatus}
                </span>
              )}
            </div>
          </div>

          <button
            type="button"
            className="service-selection-back"
            onClick={() => {
              setShowDockingConfirm(false);
              setShowServiceSelection(false);
              setShowWaiting(true);
            }}
          >
            {/* HJ_261105 - Name the screen this button actually returns to. */}
            {t(chargingStandby ? '← 충전 화면으로' : '← 대기 화면으로')}
          </button>
        </main>

        {idleMapPanel}
        {showDockingConfirm && (
          <div className="move-confirm-overlay" onClick={() => setShowDockingConfirm(false)}>
            <div className="move-confirm-box" onClick={e => e.stopPropagation()}>
              <p className="move-confirm-msg">
                {t('충전을 시작할까요?')}<br />
                {t('배터리 잔량과 관계없이 충전을 요청합니다.')}
              </p>
              <div className="move-confirm-btns">
                <button className="move-confirm-yes" data-ui="operator-service-docking-confirm" onClick={handleServiceDocking}>
                  {t('충전 요청')}
                </button>
                <button className="move-confirm-no" onClick={() => setShowDockingConfirm(false)}>
                  {t('취소')}
                </button>
              </div>
            </div>
          </div>
        )}
      </div>
    );
  }

  // HJ_261118 - Site board paging: 16 sites per page within the active zone.
  const zoneSites = sitesInZone(siteZoneTab);
  const zonePageCount = Math.max(1, Math.ceil(zoneSites.length / ZONE_PAGE_SIZE));
  const zonePageSites = zoneSites.slice(togglePage * ZONE_PAGE_SIZE, (togglePage + 1) * ZONE_PAGE_SIZE);

  return (
    <div className={`main-layout waiting-theme-forest${previewMode ? ' driving-preview-app' : ''}${drivingDisplay.visible ? ' main-layout-driving' : ''}`}
      data-ui="operator-control-screen">
      {adminEntryZone}

      {/* ── 상단 헤더 배너 (HJ_261118 - 대기 화면과 같은 숲 테마 헤더) ── */}
      {renderForestHeader({ admissionStatus: guestAdmissionStatus })}

      {/* 대기·서비스 선택 화면과 같은 운행 상태 배너를 목적지 화면에서도 유지한다. */}
      <WaitingRuntimeStatusPanel
        systemHealth={systemHealth}
        missionPhase={missionPhase}
        batteryPolicy={currentBatteryPolicy}
        parkingPolicy={parkingPolicy}
        serviceStateName={serviceStateName}
        serviceStateDescription={serviceStateDescription}
        batteryChargeComplete={batteryChargeComplete}
      />

      {/* ── 역할 배너: 서비스 메뉴에서 확정된 배달/리콜을 바디 전체 폭에 표시 ── */}
      <div className={`mission-role-banner role-${destinationIntent}`}>
        <strong>{t(destinationIntent === 'recall' ? '호출 서비스' : '배달 서비스')}</strong>
        {previewMode && <span className="driving-integrated-demo-label">{t('화면 시연 · 실제 주행 아님')}</span>}
        {drivingDisplay.canOpen && !drivingDisplay.visible && (
          <button type="button" className="driving-integrated-open" data-ui="open-driving-display"
            onClick={drivingDisplay.open}>{t('주행 화면 보기')}</button>
        )}
      </div>

      {/* ── 바디: 프리뷰 + 컨트롤 패널 ── */}
      <div className={`control-body${drivingDisplay.visible ? ' control-body-driving' : ''}`}
        data-preview={previewMode && !drivingDisplay.visible ? 'returned' : undefined}>
        {drivingDisplay.visible && (
          <div className="driving-integrated-panel" data-ui="driving-display-overlay">
            <LiveDrivingDisplay feed={drivingDisplay.feed} store={drivingDisplay.store} onDismiss={drivingDisplay.dismiss}
              onSafetyStop={previewMode ? undefined : handleStopMove}
              onToggleTheme={previewMode ? undefined : () => setDrivingTheme(current => current === 'light' ? 'dark' : 'light')}
              systemHealth={systemHealth} batteryPolicy={currentBatteryPolicy}
              embedded demo={previewMode} theme={previewMode ? drivingPreviewTheme : drivingTheme} />
          </div>
        )}

        {/* ── 왼쪽: 사이트 이미지 프리뷰 패널 ── */}
        <div className="preview-panel">
          <div className="preview-panel-main">
          {missionExecutionError ? missionExecutionWarning : missionPhase === 'INITIALIZING' ? (
            <>
              <span className="preview-placeholder-title">{t('시스템 준비 중')}</span>
              <span className="preview-placeholder">
                {t('잠시만 기다려 주세요.')}
              </span>
            </>
          ) : selectedSite ? (
            // HJ_261118 - Selected-site card: photo, zone badge, role, one CTA.
            <div className="site-pick">
              {SITE_IMAGES[selectedSite] ? (
                <img
                  src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[selectedSite]}`}
                  alt={`${selectedSite} site`}
                  className="preview-image site-pick-photo"
                />
              ) : (
                <div className="site-pick-photo site-pick-photo-empty">{t('사이트 사진 준비 중')}</div>
              )}
              <div className="site-pick-name">
                <span className="site-pick-zone">{siteZone(selectedSite)}</span>
                <p className="preview-site-name">{selectedSite}</p>
                <span className="site-pick-role">{t(destinationIntent === 'recall' ? '호출' : '배달')}</span>
              </div>
              <p className="site-pick-info">
                {destinationIntent === 'recall'
                  ? <>{t('사이트 앞 도로로 로봇이 와요.')}<br />{t('사이트 안으로는 들어가지 않아요.')}</>
                  : <>{t('사이트 안까지 로봇이 짐을 옮겨드려요.')}<br />{t('텐트와 장비가 길을 막지 않게 해주세요.')}</>}
              </p>
              <button
                className="preview-yes-btn site-pick-go"
                data-ui="operator-site-preview-confirm"
                onClick={() => setShowMoveConfirm(true)}
                disabled={recallRequestPending}
              >
                {recallRequestPending
                  ? t('요청 중…')
                  : t(destinationIntent === 'recall' ? '여기로 로봇 부르기' : '여기로 짐 보내기')}
              </button>
            </div>
          ) : arrivedSite && !recallTurnaroundInProgress ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[arrivedSite]}`}
                alt={`${arrivedSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{arrivedSite}</p>
              <p className="preview-arrived">
                {recallArrivalPresentation
                  ? t(recallFinalReturnReady ? '사이트 안에서 회전 완료 · 정차 중' : '도로 측 대기점에 도착했습니다.')
                  : t('배송 로봇이 사이트 내부 목적지에 도착했습니다.')}
              </p>
              {recallArrivalPresentation && (
                <p className="preview-question">{recallReturnInstructions(arrivedSite, recallFinalReturnReady)}</p>
              )}
              {recallFinalReturnReady && batteryReturnState.urgent && <p role="alert" className="preview-question">{t('배터리가 25% 미만입니다. 안전을 위해 짐 싣기 완료 확인 전에는 자동 출차하지 않습니다.')}</p>}
              {robotOwnsReturn ? (
                <button className="preview-return-btn" data-ui="operator-arrival-return"
                  onClick={requestArrivalComplete} disabled={returnRequestPending}>
                  {returnRequestPending ? t('완료 요청 중…') : recallArrivalPresentation
                    ? t(recallCompletionLabel(arrivedSite, recallFinalReturnReady))
                    /* HH_260909 - Delivery arrival is announced as arrival, not
                       as receipt of goods. The guest screen uses the same word. */
                    : t('도착 완료 · 복귀')}
                </button>
              ) : (
                <p className="preview-question">
                  {guestOwnsReturn
                    ? t('이 임무의 완료·복귀는 이용객 화면에서 진행합니다.')
                    : t('현재 임무의 복귀 권한을 확인하고 있습니다.')}
                </p>
              )}
            </>
          ) : ['CHARGING', 'WAITING_FOR_CHARGING', 'DROP_ZONE_PARKING'].includes(serviceStateName) ? (
            <>
              <span className="preview-placeholder-title">
                {t(motionNotice?.label || parkingLifecycleStatus(serviceStateName, serviceStateDescription, parkingPolicy, batteryChargeComplete))}
              </span>
              <p className="preview-returning" aria-live="polite">
                {motionNotice?.message ? t(motionNotice.message) : (serviceStateName === 'CHARGING'
                  ? (batteryChargeComplete
                    ? t('배터리가 100%로 충전되었습니다.')
                    : Number.isFinite(Number(batteryPct)) && batteryPct !== null && Number(batteryPct) >= 0
                      ? t(`현재 배터리 ${batteryPct}% · 배터리를 충전하고 있습니다.`)
                      : t('플랫폼에서 충전 상태가 확인되었습니다.'))
                  : serviceStateName === 'WAITING_FOR_CHARGING'
                    ? t('충전 도킹 정렬이 완료되어 충전 접점 연결을 기다리고 있습니다.')
                    : t('로봇이 대기·충전 장소에서 마지막 주차 동작을 진행하고 있습니다.'))}
              </p>
            </>
          ) : displayedReturning ? (
            <>
              <span className="preview-placeholder-title">{t('대기·충전 장소')}</span>
              {recallReturnPresentation && <p className="preview-site-name">{t(motionNotice?.label || recallProgress.label)}</p>}
              <p className="preview-returning" aria-live="polite">
                {motionNotice?.message ? t(motionNotice.message) : (recallReturnPresentation
                  ? t(recallProgress.message)
                  : serviceStateName === 'WAITING_FOR_CHARGING'
                  ? t('주차를 마치고 충전기 연결을 기다리고 있습니다.')
                  : serviceStateName === 'DROP_ZONE_PARKING'
                    ? t('대기·충전 장소에서 주차 중입니다.')
                    : t('배송을 마치고 대기·충전 장소로 복귀 중입니다.'))}
              </p>
              <p className="preview-question">{t('운행을 정지하시겠습니까?')}</p>
              <div className="preview-yn-btns">
                <button type="button" className="preview-stop-btn" onClick={() => requestStopConfirmation('service')}>{lang === 'en' ? 'Stop' : '운행 정지'}</button>
              </div>
            </>
          ) : activeSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[activeSite]}`}
                alt={`${activeSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{activeSite}</p>
              <p className="preview-moving">{t(motionNotice?.message || '배송 로봇이 이동중 입니다.')}</p>
              <p className="preview-question">{t('운행을 정지하시겠습니까?')}</p>
              <div className="preview-yn-btns">
                <button type="button" className="preview-stop-btn" onClick={() => requestStopConfirmation('service')}>{lang === 'en' ? 'Stop' : '운행 정지'}</button>
              </div>
            </>
          ) : activeRecallSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[activeRecallSite]}`}
                alt={`${activeRecallSite} 호출 사이트`}
                className="preview-image"
              />
              <p className="preview-site-name">{t(`${activeRecallSite} 호출`)}</p>
              <p className="preview-moving">{t(motionNotice?.message || '도로 측 대기 지점으로 이동 중입니다.')}</p>
              <p className="preview-question">{t('운행을 정지하시겠습니까?')}</p>
              <div className="preview-yn-btns">
                <button type="button" className="preview-stop-btn" onClick={() => requestStopConfirmation('service')}>{lang === 'en' ? 'Stop' : '운행 정지'}</button>
              </div>
            </>
          ) : manualDriveActive ? (
            <>
              <span className="preview-placeholder-title">{t('지정 위치로 이동')}</span>
              <p className="preview-moving">
                {t(motionNotice?.message || MISSION_PHASE_LABELS[missionPhase] || missionPhase)}
              </p>
              <p className="preview-question">{t('운행을 정지하시겠습니까?')}</p>
              <div className="preview-yn-btns">
                <button type="button" className="preview-stop-btn" onClick={() => requestStopConfirmation('manual')}>{lang === 'en' ? 'Stop' : '운행 정지'}</button>
              </div>
            </>
          ) : serviceStateName === 'OPERATOR_STOPPED' ? (
            <>
              <span className="preview-placeholder-title">{t('운행 정지')}</span>
              <p className="preview-returning">{t('운행이 정지되었습니다.')}</p>
            </>
          ) : (
            <>
              <span className="preview-placeholder-title">{t('사이트를 골라주세요')}</span>
              <span className="preview-placeholder">
                {t(destinationIntent === 'recall' ? '오른쪽에서 로봇을 부를 사이트를 누르면' : '오른쪽에서 짐을 받을 사이트를 누르면')}<br />
                {t('사진과 함께 여기에 보여드려요.')}
              </span>
            </>
          )}
          <ServiceTripBadge
            serviceActive={Boolean(
              activeSite || activeRecallSite || arrivedSite || displayedReturning
            )}
            currentService={serviceMetrics.data?.current_service}
            loading={serviceMetrics.loading}
            error={serviceMetrics.error}
            onOpen={() => setActiveModal('service-evidence')}
          />
          </div>
          {/* HJ_261118 - Back to the service chooser lives at the bottom of the left card. */}
          <button
            className="control-back-btn"
            data-ui="operator-back-to-services"
            onClick={() => setShowServiceSelection(true)}
            disabled={Boolean(activeSite || activeRecallSite || isReturning)}
          >
            {t('← 서비스 선택')}
          </button>
        </div>

        {/* ── 오른쪽: 구역 탭 + 사이트 판 (HJ_261118) ── */}
        <div className="app site-board">
          <div className="site-zone-tabs" role="tablist">
            {SITE_ZONES.map(zone => {
              const zoneSites = sitesInZone(zone).filter(site => !isPlannedSite(site));
              const count = destinationIntent === 'recall'
                ? zoneSites.filter(site => occupiedSites.includes(site)).length
                : zoneSites.filter(site => !occupiedSites.includes(site)).length;
              return (
                <button
                  key={zone}
                  type="button"
                  role="tab"
                  aria-selected={zone === siteZoneTab}
                  data-ui={`operator-site-zone-${zone}`}
                  className={`site-zone-tab${zone === siteZoneTab ? ' active' : ''}`}
                  onClick={() => { setSiteZoneTab(zone); setTogglePage(0); }}
                >
                  <span className="site-zone-badge">{zone}</span>
                  <span className="site-zone-copy">
                    <strong>{t(`${zone}구역`)}</strong>
                    <small>
                      {!zoneSites.length
                        ? t('운영 준비 중')
                        : t(destinationIntent === 'recall' ? `텐트 ${count}곳` : `빈 자리 ${count}곳`)}
                    </small>
                  </span>
                </button>
              );
            })}
          </div>

          <div className="site-board-head">
            <h1>
              {t(`${siteZoneTab}구역에서 ${destinationIntent === 'recall'
                ? '로봇을 부를 사이트를 골라주세요'
                : '짐을 받을 사이트를 골라주세요'}`)}
            </h1>
            <div className="site-board-legend" aria-hidden="true">
              {destinationIntent === 'recall' ? (
                <><span><i className="legend-tent" />{t('텐트 있음')}</span><span><i className="legend-free" />{t('비어 있음')}</span></>
              ) : (
                <><span><i className="legend-free" />{t('비어 있음')}</span><span><i className="legend-busy" />{t('사용 중')}</span></>
              )}
            </div>
          </div>

          <div className="toggle-grid">
            {zonePageSites.map(site => {
                const occupied = occupiedSites.includes(site);
                const planned = isPlannedSite(site);
                const status = planned
                  ? t('준비 중')
                  : states[site]
                  ? t('ON · 진행 중')
                  : selectedSite === site
                    ? t('선택됨')
                    : occupied
                      ? t(destinationIntent === 'recall' ? '텐트 있음' : '사용 중')
                      : t('비어 있음');
                return (
                  <button
                    key={site}
                    data-ui={`operator-site-${site}`}
                    className={`toggle-card ${states[site] ? 'on' : ''} ${selectedSite === site ? 'selected' : ''} ${missionSelectionLocked && !states[site] ? 'locked' : ''} ${destinationIntent === 'delivery' && occupied ? 'occupied' : ''} ${destinationIntent === 'recall' && occupied ? 'tent' : ''} ${planned ? 'planned' : ''}`}
                    onClick={() => handleToggle(site)}
                    disabled={
                      planned
                      || missionSelectionLocked
                      || (
                        destinationIntent === 'delivery'
                        && occupied
                      )
                    }
                  >
                    <span className="site-label">{site}</span>
                    <span className="site-status">{status}</span>
                  </button>
                );
              })}
          </div>

          {/* A zone with more than 16 sites pages within that zone (4 × 4 per page). */}
          {zonePageCount > 1 && (
            <div className="site-board-pager">
              <button
                className="page-arrow left"
                aria-label={t('이전 쪽')}
                onClick={() => setTogglePage(p => Math.max(0, p - 1))}
                disabled={togglePage === 0}
              >
                ◀
              </button>
              <div className="page-dots">
                {Array.from({ length: zonePageCount }).map((_, i) => (
                  <span key={i} data-ui={`operator-site-page-${i}`} className={`page-dot ${togglePage === i ? 'active' : ''}`} onClick={() => setTogglePage(i)} />
                ))}
              </div>
              <span className="site-board-page-label">
                {zonePageSites[0]}~{zonePageSites[zonePageSites.length - 1]} · {togglePage + 1}/{zonePageCount}{lang === 'en' ? '' : '쪽'}
              </span>
              <button
                className="page-arrow right"
                aria-label={t('다음 쪽')}
                onClick={() => setTogglePage(p => Math.min(zonePageCount - 1, p + 1))}
                disabled={togglePage === zonePageCount - 1}
              >
                ▶
              </button>
            </div>
          )}
        </div>

      </div>

      {/* 운행 중에도 현재 미션을 중단하지 않고 누적 운행 기록을 조회한다. */}
      {serviceEvidenceModal}

      {/* ── HJ_261118 - 도착 화면 (기존 출발 최종 확인 팝업을 전체 화면으로 대체) ── */}
          {showArrivalComplete && arrivedSite && robotOwnsReturn
            && !recallTurnaroundInProgress && (() => {
        const copy = arrivalScreenCopy(arrivedSite, recallArrivalPresentation, recallFinalReturnReady);
        return (
          <div className="arrival-screen waiting-theme-forest" data-ui="operator-arrival-screen">
            {renderForestHeader()}
            <section className="arrival-trip">
              <span className="arrival-face" aria-hidden="true">
                <svg viewBox="0 0 20 12"><path d="M2 10 Q10 0 18 10" /></svg>
                <svg viewBox="0 0 20 12"><path d="M2 10 Q10 0 18 10" /></svg>
              </span>
              <div className="arrival-trip-copy">
                <h1>{t(copy.title)}</h1>
                <p>{t(copy.subtitle)}</p>
              </div>
              <span className="arrival-status"><i />{t('도착 · 정차 중')}</span>
            </section>
            {recallFinalReturnReady && batteryReturnState.urgent && (
              <p role="alert" className="arrival-alert">{t('배터리가 25% 미만이어도 짐 싣기 완료 확인 전에는 자동 출차하지 않습니다.')}</p>
            )}
            <section className="arrival-steps">
              {copy.steps.map(([kind, title, detail], index) => (
                <div className="arrival-step" key={kind}>
                  <span className="arrival-step-num">{index + 1}</span>
                  <div className="arrival-step-art"><ArrivalArt kind={kind} /></div>
                  <strong>{t(title)}</strong>
                  <small>{t(detail)}</small>
                </div>
              ))}
            </section>
            <section className="arrival-end">
              <div className="arrival-end-copy">
                <strong>{t(copy.question)}</strong>
                <small>{t(copy.hint)}</small>
              </div>
              <button type="button" className="arrival-end-btn" data-ui="operator-arrival-finish"
                onClick={requestArrivalComplete} disabled={returnRequestPending}>
                {t(returnRequestPending ? '완료 요청 중…' : copy.action)}
              </button>
            </section>
          </div>
        );
      })()}

      {/* HJ_261118 - 도착 후 출발 전 한 번 더 확인 (도착 화면·왼쪽 복귀 버튼 공통) */}
      {arrivalConfirmOpen && arrivedSite && robotOwnsReturn && !recallTurnaroundInProgress && (() => {
        const copy = arrivalScreenCopy(arrivedSite, recallArrivalPresentation, recallFinalReturnReady);
        return (
          <div className="move-confirm-overlay arrival-confirm-overlay" onClick={() => setArrivalConfirmOpen(false)}>
            <div className="move-confirm-box" role="alertdialog" aria-modal="true" onClick={e => e.stopPropagation()}>
              <p className="move-confirm-msg">
                {t(copy.confirmTitle)}<br />
                <span className="arrival-confirm-body">{t(copy.confirmBody)}</span>
              </p>
              <div className="move-confirm-btns">
                <button className="move-confirm-no" data-ui="operator-arrival-return-cancel"
                  onClick={() => setArrivalConfirmOpen(false)}>
                  {t('취소')}
                </button>
                <button className="move-confirm-yes" data-ui="operator-arrival-return-confirm"
                  onClick={confirmArrivalComplete} disabled={returnRequestPending}>
                  {t(copy.action)}
                </button>
              </div>
            </div>
          </div>
        );
      })()}

      {showMoveConfirm && (
        <div className="move-confirm-overlay" onClick={() => setShowMoveConfirm(false)}>
          <div className="move-confirm-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg">
              {destinationIntent === 'recall' ? (
                <>
                  {t(`${selectedSite} 앞 도로로 부를까요?`)}<br />
                  {t('사이트 안으로는 들어가지 않아요.')}
                </>
              ) : (
                <>
                  {/* HH_261102 - Never claim stopping is impossible when the existing Stop control remains available. */}
                  {t(`${selectedSite}로 배달할까요?`)}<br />
                  {t('로봇 주변을 비워 주세요.')}
                </>
              )}
            </p>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-yes"
                data-ui="operator-move-confirm-yes"
                onClick={() => {
                  setShowMoveConfirm(false);
                  setMoveVerifyInput('');
                  setMoveVerifyError(false);
                  setActiveField('moveVerify');
                  setShowMoveVerify(true);
                }}
              >
                {t('다음')}
              </button>
              <button
                className="move-confirm-no"
                onClick={() => setShowMoveConfirm(false)}
              >
                {t('취소')}
              </button>
            </div>
          </div>
        </div>
      )}

      {stopConfirmation && (
        <div className="move-confirm-overlay" data-ui="operator-stop-confirm-dialog"
          onClick={() => setStopConfirmation(null)}>
          <div className="move-confirm-box operator-stop-confirm-box" role="alertdialog" aria-modal="true"
            aria-labelledby="operator-stop-confirm-title" onClick={e => e.stopPropagation()}>
            <p id="operator-stop-confirm-title" className="move-confirm-msg">{t('운행을 정지하시겠습니까?')}</p>
            <div className="move-confirm-btns">
              <button type="button" className="move-confirm-yes" data-ui="operator-stop-confirm-yes"
                onClick={confirmOperatorStop}>{t('예')}</button>
              <button type="button" className="move-confirm-no" data-ui="operator-stop-confirm-no"
                onClick={() => setStopConfirmation(null)}>{t('아니요')}</button>
            </div>
          </div>
        </div>
      )}

      {/* 사이트명 입력 확인 팝업 */}
      {showMoveVerify && selectedSite && (
        <div className="move-verify-overlay" onClick={() => { setShowMoveVerify(false); setMoveVerifyError(false); }}>
          <div className="move-verify-box" onClick={e => e.stopPropagation()}>
            <p className="move-verify-title">
              {destinationIntent === 'recall'
                ? t('호출할 사이트')
                : t('배달할 사이트')}<br />
              <span className="move-verify-site">{selectedSite}</span>
            </p>
            <p className="move-verify-sub">{t('아래에 사이트명을 입력해주세요')}</p>
            {moveVerifyError && (
              <div className="move-verify-error-box">
                <p className="move-verify-retry">{t('사이트명이 맞지 않아요. 다시 입력해 주세요.')}</p>
              </div>
            )}
            <input
              data-ui="operator-site-code-input"
              className={`move-verify-input${moveVerifyError ? ' error' : ''}`}
              value={moveVerifyInput}
              onChange={e => { setMoveVerifyInput(e.target.value.toUpperCase()); setMoveVerifyError(false); }}
              onFocus={() => setActiveField('moveVerify')}
              placeholder={t(`예: ${selectedSite}`)}
              autoFocus
            />

            {/* ── 가상 키보드 (사이트명 확인) ── */}
            <div className="vkb-wrap move-verify-keyboard">
              {KB_ROWS.map((row, ri) => (
                <div key={ri} className="vkb-row">
                  {row.map(key => (
                    <button
                      type="button"
                      key={key + ri}
                      className={`vkb-key${key === '⌫' ? ' vkb-wide' : ''}${key === '⇧' ? ' vkb-wide' + (kbCaps ? ' vkb-caps-on' : '') : ''}`}
                      onClick={() => handleVirtualKey(key)}
                    >
                      {key === '⇧' ? (kbCaps ? '⇧ ON' : '⇧') : key === '⌫' ? key : key.toUpperCase()}
                    </button>
                  ))}
                </div>
              ))}
              <div className="vkb-row">
                <button type="button" className="vkb-key vkb-space" onClick={() => handleVirtualKey('SPACE')}>SPACE</button>
              </div>
            </div>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-no"
                onClick={() => { setShowMoveVerify(false); setMoveVerifyError(false); }}
              >
                {t('취소')}
              </button>
              <button
                className="move-confirm-yes"
                data-ui="operator-site-code-confirm"
                onClick={() => {
                  if (moveVerifyInput.trim().toUpperCase() === selectedSite.toUpperCase()) {
                    setShowMoveVerify(false);
                    handleConfirmMove();
                  } else {
                    setMoveVerifyError(true);
                  }
                }}
              >
                {t('확인')}
              </button>
            </div>
          </div>
        </div>
      )}

      {missionBlockMessage && (
        <div className="move-confirm-overlay mission-block-overlay" onClick={() => setMissionBlockMessage('')}>
          <div className="move-confirm-box mission-block-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg mission-block-msg">{t(missionBlockMessage)}</p>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-no mission-block-close"
                onClick={() => setMissionBlockMessage('')}
              >
                {t('확인')}
              </button>
            </div>
          </div>
        </div>
      )}

      {batteryReturnMessage && (
        <div className="move-confirm-overlay mission-block-overlay" onClick={() => setBatteryReturnMessage('')}>
          <div className="move-confirm-box mission-block-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg mission-block-msg">{t(batteryReturnMessage)}</p>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-no mission-block-close"
                onClick={() => setBatteryReturnMessage('')}
              >
                {t('확인')}
              </button>
            </div>
          </div>
        </div>
      )}


    </div>
  );
}

export default App;

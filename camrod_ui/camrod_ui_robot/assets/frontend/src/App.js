/**
 * App.js — 로봇 사이트별 토글 컨트롤 메인 컴포넌트
 *
 * 역할:
 *   1. B1~B13 사이트별 목적지 버튼을 페이지 단위로 렌더링
 *   2. WebSocket으로 FastAPI 백엔드와 실시간 통신
 *   3. 버튼 클릭 시 {"site": "B1", "state": true}를 백엔드에 전송
 *   4. 백엔드에서 받은 상태로 각 버튼 UI 동기화
 */

import React, { useState, useEffect, useRef, useCallback } from 'react';
import './App.css';
import RobotAnimation from './RobotAnimation';
import TelemetryWorkspace, { TELEMETRY_TABS } from './TelemetryWorkspace';
import {
  ServiceEvidenceDashboard,
  ServiceEvidenceSummary,
  ServiceTripBadge,
  useServiceMetricsSummary,
} from './ServiceEvidence';

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
  INITIALIZING: '초기화 중',
  READY: '운행 준비 완료',
  GOAL_RECEIVED: '목표 수신',
  PATH_PREPARING: '경로 준비 중',
  DRIVING: '주행 중',
  SAFETY_STOP: '안전 정지',
  ARRIVED: '도착',
  STOPPED: '운행 정지',
});
const SYSTEM_HEALTH_LABELS = Object.freeze({
  // HH_260906 - Health remains independent from service progress, while the
  // primary operator status is presented consistently in Korean.
  STARTING: '시스템 시작 중',
  OK: '시스템 정상',
  WARNING: '시스템 경고',
  ERROR: '시스템 오류',
});
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
const CRITICAL_BATTERY_STOP_PERCENT = 20;
const MISSION_DISPATCH_MINIMUM_PERCENT = 35;

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
  if (data.battery_return_started) {
    return `${prefix} 복귀 요청이 확인되어 대기·충전 장소로 이동합니다.`;
  }
  if (data.battery_return_waiting_for_user) {
    return `${prefix} 짐 처리를 마친 뒤 완료·복귀 버튼을 눌러주세요. 확인 전에는 이동하지 않습니다.`;
  }
  return `${prefix} ${minimum}% 미만이면 새 임무를 받지 않고, 현재 임무 완료 후 대기·충전 장소 복귀를 기다립니다.`;
};

const batteryPolicyStatus = (batteryPct, batteryReturnState) => {
  // HH_260724 - Keep the battery policy visible after the modal is dismissed.
  const battery = Number(batteryPct);
  const batteryText = Number.isFinite(battery) && battery >= 0 ? `${battery}%` : '';
  if (batteryReturnState.started) {
    return { tone: 'warning', label: `저전력 자동 복귀 중 ${batteryText}`.trim() };
  }
  if (batteryReturnState.waitingForUser) {
    return { tone: 'warning', label: `이용 완료·복귀 요청 대기 ${batteryText}`.trim() };
  }
  if (batteryReturnState.pending) {
    return { tone: 'warning', label: `현재 임무 완료 후 복귀 ${batteryText}`.trim() };
  }
  if (!Number.isFinite(battery) || battery < 0) {
    return { tone: 'warning', label: '배터리 상태 확인 중' };
  }
  if (battery <= CRITICAL_BATTERY_STOP_PERCENT) {
    return { tone: 'error', label: `배터리 위험 수준 · 운행 정지 ${battery}%` };
  }
  if (battery < MISSION_DISPATCH_MINIMUM_PERCENT) {
    return { tone: 'warning', label: `임무 보류 · ${MISSION_DISPATCH_MINIMUM_PERCENT}% 미만 (${battery}%)` };
  }
  return { tone: 'ok', label: `임무 배터리 준비 완료 ${battery}%` };
};

// HH_260721 - Reuse one health/service presentation on waiting and destination screens.
function RuntimeStatus({ systemHealth, missionPhase, batteryPolicy }) {
  return (
    <div className="ch-runtime-status" aria-live="polite">
      <span className={`ch-runtime-line health-${systemHealth.toLowerCase()}`}>
        <span className="ch-runtime-dot" />
        {SYSTEM_HEALTH_LABELS[systemHealth] || SYSTEM_HEALTH_LABELS.STARTING}
      </span>
      <span className="ch-runtime-line service-state">
        <span className="ch-runtime-dot" />
        {MISSION_PHASE_LABELS[missionPhase] || MISSION_PHASE_LABELS.INITIALIZING}
      </span>
      {batteryPolicy && (
        <span className={`ch-runtime-line battery-policy policy-${batteryPolicy.tone}`}>
          <span className="ch-runtime-dot" />
          {batteryPolicy.label}
        </span>
      )}
    </div>
  );
}

// HH_260804 - Keep one-second clock updates local so the rest of the operator
// screen does not re-render whenever only the displayed time has changed.
function LiveClock() {
  const [currentTime, setCurrentTime] = useState(new Date());

  useEffect(() => {
    const timer = setInterval(() => setCurrentTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  const dateStr = currentTime.toLocaleDateString('ko-KR', {
    year: 'numeric', month: 'long', day: 'numeric', weekday: 'short'
  });
  const timeStr = currentTime.toLocaleTimeString('ko-KR', {
    hour: '2-digit', minute: '2-digit', second: '2-digit'
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
        더보기
      </button>
      {open && (
        <div className="carousel-overlay" onClick={() => setOpen(false)}>
          <div className="carousel-box" onClick={e => e.stopPropagation()}>
            <button className="carousel-close-btn" onClick={() => setOpen(false)}>뒤로가기</button>
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
function DiagnosticsMonitor({ redockStatus = null }) {
  const [activeTab, setActiveTab] = useState('system');
  const [items, setItems] = useState([]);
  const [selected, setSelected] = useState(null);
  const [expanded, setExpanded] = useState({ error: true, warn: true, ok: false });
  // HH_260819 - Return is the sole operator motion command. The backend selects
  // campsite exit, drop-zone routing, or final parking from authoritative state.
  const [motionCommandPending, setMotionCommandPending] = useState('');
  const [motionCommandStatus, setMotionCommandStatus] = useState('');
  const [steeringRate, setSteeringRate] = useState(0.5);
  const [steeringTuningAvailable, setSteeringTuningAvailable] = useState(false);
  const [steeringTuningStatus, setSteeringTuningStatus] = useState('드라이버 연결 확인 중');
  const steeringTuningTimerRef = useRef(null);

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
    fetch('/ui/platform_tuning')
      .then(async response => {
        const body = await response.json();
        if (!response.ok || !body.success) throw new Error(body.message || '설정 조회 실패');
        setSteeringRate(Number(body.steering_transition_rate_radps));
        setSteeringTuningAvailable(true);
        setSteeringTuningStatus('런타임 조절 가능');
      })
      .catch(error => {
        setSteeringTuningAvailable(false);
        setSteeringTuningStatus(error.message || 'Ranger 드라이버 연결 안 됨');
      });
    return () => {
      if (steeringTuningTimerRef.current) {
        clearTimeout(steeringTuningTimerRef.current);
      }
    };
  }, []);

  const applySteeringRate = (nextRate) => {
    const clamped = Math.max(0.05, Math.min(2.0, Number(nextRate)));
    setSteeringTuningStatus('적용 중…');
    fetch(
      `/ui/platform_tuning?steering_transition_rate_radps=${encodeURIComponent(clamped.toFixed(2))}`,
      { method: 'POST' }
    )
      .then(async response => {
        const body = await response.json();
        if (!response.ok || !body.success) throw new Error(body.message || '적용 실패');
        setSteeringTuningAvailable(true);
        setSteeringRate(Number(body.steering_transition_rate_radps));
        setSteeringTuningStatus('즉시 적용됨');
      })
      .catch(error => {
        setSteeringTuningStatus(error.message || '적용 실패');
      });
  };

  const handleSteeringRateChange = (event) => {
    const nextRate = Number(event.target.value);
    setSteeringRate(nextRate);
    if (steeringTuningTimerRef.current) {
      clearTimeout(steeringTuningTimerRef.current);
    }
    steeringTuningTimerRef.current = setTimeout(
      () => applySteeringRate(nextRate),
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
        parking_alignment: '재도킹 정렬을 시작합니다',
        parking_alignment_waiting_for_can: '재도킹 대기 중 · 리모컨을 CAN 모드로 전환하세요',
        waiting_for_disconnect: '충전 접점 해제 확인 후 자동으로 재도킹합니다',
        parking_in_progress: '도킹이 이미 진행 중입니다',
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
  const HEAD_COLOR = { 0: '#6ec86e', 1: '#e0b06c', 2: '#e06c6c' };

  const groups = [
    { key: 'error', label: 'Error',  lvls: [2],    items: items.filter(i => i.level === 2) },
    { key: 'warn',  label: 'Warn',   lvls: [1],    items: items.filter(i => i.level === 1) },
    { key: 'ok',    label: 'OK',     lvls: [0],    items: items.filter(i => i.level === 0) },
  ].filter(g => g.items.length > 0);

  return (
    <div className="diag-monitor-wrap">
      <nav className="diag-tab-bar" aria-label="관리자 진단 화면">
        {[{ id: 'system', label: '시스템' }, ...TELEMETRY_TABS].map(tab => (
          <button
            key={tab.id}
            type="button"
            data-ui={`operator-diagnostic-tab-${tab.id}`}
            className={`diag-tab-button ${activeTab === tab.id ? 'active' : ''}`}
            onClick={() => setActiveTab(tab.id)}
            aria-selected={activeTab === tab.id}
          >
            {tab.label}
          </button>
        ))}
      </nav>
      {activeTab === 'system' ? (
        <>
      {/* ── 상단 컨트롤 바 ── */}
      <div className="diag-control-bar">
        <span className="diag-control-label">수동 운행</span>
        <button
          type="button"
          className="manual-return-btn"
          onClick={requestManualReturn}
          disabled={Boolean(motionCommandPending)}
          title="현재 서비스 상태와 관계없이 drop zone 복귀를 요청"
        >
          {motionCommandPending === 'return' ? '복귀 요청 중' : '복귀'}
        </button>
        <span className="manual-motion-status">{motionCommandStatus || '명령 대기'}</span>
      </div>

      <div className="steering-tuning-card">
        <div className="steering-tuning-copy">
          <div className="steering-tuning-title">횡↔종 조향 전환 속도</div>
          <div className="steering-tuning-help">
            바퀴 방향이 종방향과 횡방향 사이에서 회전하는 최대 속도입니다.
            변경값은 재시작 없이 즉시 적용됩니다.
          </div>
          <div className={`steering-tuning-status ${steeringTuningAvailable ? 'available' : ''}`}>
            {steeringTuningStatus}
          </div>
        </div>
        <div className="steering-tuning-control">
          <div className="steering-tuning-value">
            <strong>{steeringRate.toFixed(2)}</strong> rad/s
            <span>{Math.round(steeringRate * 100)}%</span>
          </div>
          <input
            className="steering-tuning-slider"
            type="range"
            min="0.05"
            max="2.0"
            step="0.05"
            value={steeringRate}
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

    <div className="diag-monitor">
      {/* ── 왼쪽: 트리 패널 ── */}
      <div className="diag-tree">
        <div className="diag-tree-header">
          Device groups
          <span className="diag-count">{items.length}개 항목</span>
        </div>
        {groups.map(g => {
          const lvl = g.lvls[0];
          const isOpen = expanded[g.key];
          return (
            <div key={g.key}>
              <div
                className="diag-group-header"
                style={{ color: HEAD_COLOR[lvl] }}
                onClick={() => setExpanded(e => ({ ...e, [g.key]: !e[g.key] }))}
              >
                <span className="diag-arrow">{isOpen ? '▼' : '▶'}</span>
                {g.label} ({g.items.length})
              </div>
              {isOpen && g.items.map((item, i) => (
                <div
                  key={i}
                  className={`diag-item ${selected === item ? 'diag-item-selected' : ''}`}
                  onClick={() => setSelected(item)}
                >
                  <span className="diag-dot" style={{ background: DOT_COLOR[item.level] }} />
                  {item.name}
                </div>
              ))}
            </div>
          );
        })}
        {items.length === 0 && (
          /* HH_260617: UI diagnostics reads the namespaced system aggregator topic. */
          <div className="diag-empty">데이터 없음 — /system/diagnostics_agg 대기 중…</div>
        )}
      </div>

      {/* ── 오른쪽: 상세 패널 ── */}
      <div className="diag-detail">
        {selected ? (
          <>
            <div className="diag-detail-header">Detail — {selected.name}</div>
            <div className="diag-detail-status">
              <span className="diag-dot lg" style={{ background: DOT_COLOR[selected.level] }} />
              <span style={{ color: HEAD_COLOR[selected.level], fontWeight: 600 }}>
                {LEVEL_STR[selected.level]} (level {selected.level})
              </span>
            </div>
            <div className="diag-detail-label">Message:</div>
            <div className="diag-detail-msg" style={{ borderColor: DOT_COLOR[selected.level] }}>
              {selected.message || '—'}
            </div>
            {selected.values && selected.values.length > 0 && (
              <>
                <div className="diag-detail-label" style={{ marginTop: '0.8rem' }}>Key-value pairs:</div>
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
        </>
      ) : (
        <TelemetryWorkspace activeTab={activeTab} redockStatus={redockStatus} />
      )}
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
              <span>월악산 국립공원</span>
              <span className="park-banner-sep">|</span>
              <span>닷돈재 야영장</span>
              <span className="park-banner-sep">|</span>
              <span className="park-banner-current">시설 안내</span>
            </div>
          </div>
        </div>
      </div>

      {/* ── 기본 정보 카드 ── */}
      <div className="facility-section">
        <div className="facility-section-title">📋 기본 정보</div>
        <div className="facility-info-grid">
          <div className="facility-info-item">
            <span className="fi-icon">📍</span>
            <div>
              <div className="fi-label">주소</div>
              <div className="fi-value">충북 제천시 한수면 송계리 70-2</div>
            </div>
          </div>
          <div className="facility-info-item">
            <span className="fi-icon">📞</span>
            <div>
              <div className="fi-label">문의처</div>
              <div className="fi-value">043-653-3252</div>
            </div>
          </div>
          <div className="facility-info-item">
            <span className="fi-icon">🕐</span>
            <div>
              <div className="fi-label">입 / 퇴실 시간</div>
              <div className="fi-value">오후 2시 입실 · 익일 낮 12시(정오) 퇴실</div>
            </div>
          </div>
        </div>
      </div>

      {/* ── 이용 금액 ── */}
      <div className="facility-section">
        <div className="facility-section-title">💰 이용 금액</div>
        <div className="facility-fee-table">
          <div className="fee-row fee-header">
            <span>구분</span>
            <span>주말·성수기</span>
            <span>주중</span>
          </div>
          <div className="fee-row">
            <span>🚗 자동차 야영지</span>
            <span className="fee-peak">30,000원</span>
            <span className="fee-normal">20,000원</span>
          </div>
        </div>
        <div className="facility-notice-small">
          <p>• 주중: 법정공휴일 전일 제외, 매주 일~목요일</p>
          <p>• 주말: 매주 금·토요일, 법정공휴일 전일</p>
          <p>• 성수기: 매해 7.1~8.31, 10.1~11.15</p>
        </div>
      </div>

      {/* ── 편의시설 ── */}
      <div className="facility-section">
        <div className="facility-section-title">🏕️ 편의시설</div>
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
              <span className="amenity-label">{item.label}</span>
              <span className="amenity-desc">{item.desc}</span>
            </div>
          ))}
        </div>
      </div>

      {/* ── 영지 정보 ── */}
      <div className="facility-section">
        <div className="facility-section-title">🗺️ 영지 구성</div>
        <div className="facility-zone-grid">
          {zones.map((z, i) => (
            <div key={i} className={`zone-chip ${z.note ? 'zone-special' : ''}`}>
              <div className="zone-chip-info">
                <span className="zone-name">{z.zone}구역</span>
                <span className="zone-count">{z.count}</span>
                {z.note && <span className="zone-note">🌿 {z.note}</span>}
              </div>
              <button
                className="zone-map-btn"
                onClick={e => { e.stopPropagation(); setZoneMap(z.zone); }}
              >
                위치 안내
              </button>
            </div>
          ))}
        </div>
        <div className="facility-notice-small" style={{ marginTop: '0.7rem' }}>
          <p>• 영지 규격: 가로 5m × 세로 7m (마사토)</p>
          <p>• 총 113동 운영 (자동차 야영지)</p>
        </div>
      </div>

      {/* ── 운영 정책 안내 ── */}
      <div className="facility-policy">
        <div className="facility-section-title" style={{ marginBottom: '0.6rem' }}>⚠️ 운영 정책</div>
        <p>• 일부 <strong>무공해 영지</strong>를 지정 운영합니다. 전기·수소차, 대중교통·자전거·도보 이용객만 예약 가능합니다.</p>
        <p>• 운영정책 위반에 따른 책임은 예약자에게 있으니 예약 시 세부 사항을 꼼꼼히 확인해주세요.</p>
      </div>

      {/* ── 구역 배치도 오버레이 ── */}
      {zoneMap && (
        <div className="zone-map-overlay">
          <div className="zone-map-box">
            <button className="zone-map-back-btn" onClick={() => setZoneMap(null)}>뒤로가기</button>
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
                <span>월악산 국립공원</span>
                <span className="park-banner-sep">|</span>
                <span>닷돈재 야영장</span>
                <span className="park-banner-sep">|</span>
                <span className="park-banner-current">로봇 이용 방법</span>
              </div>
            </div>
          </div>
        </div>
        <div className="guide-grid">

        {/* ── 1. 목적지 선택 방법 ── */}
        <div className="guide-card">
          <div className="guide-card-header" style={{ background: 'linear-gradient(135deg,#2d6e40,#43a047)' }}>
            <svg viewBox="0 0 32 32" fill="none">
              <circle cx="16" cy="14" r="7" stroke="#fff" strokeWidth="2.2"/>
              <circle cx="16" cy="14" r="2.5" fill="#fff"/>
              <path d="M16 21 C10 26 6 29 6 29 L16 27 L26 29 C26 29 22 26 16 21Z" fill="#fff" opacity="0.8"/>
            </svg>
            <span>목적지 선택 방법</span>
          </div>
          <div className="guide-card-body">
            {[
              '대기 화면의 목적지 선택을 터치해 제어 패널로 이동합니다.',
              '배송 또는 이용객 호출을 선택한 뒤 원하는 사이트(B1 ~ B13)를 선택합니다.',
              '목적지 이미지와 "이동하시겠습니까?" 를 확인합니다.',
              '[예] 버튼을 누르고 사이트명을 입력해 로봇 출발을 확정합니다.',
            ].map((text, i) => (
              <div key={i} className="guide-step">
                <span className="guide-step-num" style={{ background: '#2d6e40' }}>{i + 1}</span>
                <span>{text}</span>
              </div>
            ))}
          </div>
        </div>

        {/* ── 2. 주의 사항 ── */}
        <div className="guide-card">
          <div className="guide-card-header" style={{ background: 'linear-gradient(135deg,#e65100,#ff8f00)' }}>
            <svg viewBox="0 0 32 32" fill="none">
              <polygon points="16,4 29,27 3,27" stroke="#fff" strokeWidth="2.2" strokeLinejoin="round" fill="none"/>
              <line x1="16" y1="13" x2="16" y2="20" stroke="#fff" strokeWidth="2.4" strokeLinecap="round"/>
              <circle cx="16" cy="23.5" r="1.4" fill="#fff"/>
            </svg>
            <span>주의 사항</span>
          </div>
          <div className="guide-card-body">
            {[
              '배송 로봇 이동 중에는 경로 접근을 최소화해 주세요.',
              '경로 위의 장애물은 미리 치워 주세요.',
              '어린이·반려동물이 배송 로봇 주변에 가까이 가지 않도록 주의해주세요.',
              '로봇이 이동 중일 때는 목적지를 변경할 수 없으니 신중히 선택해 주세요.',
            ].map((text, i) => (
              <div key={i} className="guide-bullet">
                <span className="guide-bullet-dot" style={{ color: '#e65100' }}>⚠</span>
                <span>{text}</span>
              </div>
            ))}
          </div>
        </div>

        {/* ── 3. 로봇 정지 방법 ── */}
        <div className="guide-card">
          <div className="guide-card-header" style={{ background: 'linear-gradient(135deg,#b71c1c,#e53935)' }}>
            <svg viewBox="0 0 32 32" fill="none">
              <circle cx="16" cy="16" r="11" stroke="#fff" strokeWidth="2.2"/>
              <rect x="11" y="11" width="10" height="10" rx="2" fill="#fff"/>
            </svg>
            <span>로봇 정지 방법</span>
          </div>
          <div className="guide-card-body">
            {[
              '배송 로봇 이동 중 왼쪽 프리뷰 패널 하단을 확인합니다.',
              '[운행 중지] 버튼을 누릅니다.',
              '배송 로봇이 즉시 운행을 멈추고 목적지 ON 상태가 해제됩니다.',
            ].map((text, i) => (
              <div key={i} className="guide-step">
                <span className="guide-step-num" style={{ background: '#b71c1c' }}>{i + 1}</span>
                <span>{text}</span>
              </div>
            ))}
          </div>
        </div>

        {/* ── 4. 재출발 방법 ── */}
        <div className="guide-card">
          <div className="guide-card-header" style={{ background: 'linear-gradient(135deg,#1565c0,#1e88e5)' }}>
            <svg viewBox="0 0 32 32" fill="none">
              <path d="M26 16 A10 10 0 1 1 20.5 7" stroke="#fff" strokeWidth="2.2" strokeLinecap="round" fill="none"/>
              <polygon points="20,3 26,8 20,13" fill="#fff"/>
            </svg>
            <span>재출발 방법</span>
          </div>
          <div className="guide-card-body">
            {[
              '운행 정지 또는 목적지 도착 후 ON 상태가 해제됩니다.',
              '목적지 선택 패널에서 원하는 목적지를 다시 선택합니다.',
              '프리뷰에서 이미지 확인 후 [예]를 눌러 재출발합니다.',
            ].map((text, i) => (
              <div key={i} className="guide-step">
                <span className="guide-step-num" style={{ background: '#1565c0' }}>{i + 1}</span>
                <span>{text}</span>
              </div>
            ))}
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
                <span>월악산 국립공원</span>
                <span className="park-banner-sep">|</span>
                <span>닷돈재 야영장</span>
                <span className="park-banner-sep">|</span>
                <span className="park-banner-current">탐방로 안내</span>
              </div>
            </div>
          </div>
        </div>

        {/* ── 통제 상황 경고 배너 ── */}
        <div className="trail-warning">
          <svg width="22" height="22" viewBox="0 0 24 24" fill="#e65100">
            <path d="M12 2L1 21h22L12 2zm0 3.5L20.5 20h-17L12 5.5zM11 10v4h2v-4h-2zm0 6v2h2v-2h-2z"/>
          </svg>
          <span>입산 이전 통제 상황을 반드시 확인해주세요</span>
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
              <div className="trail-card-name">{trail.name}</div>
              <div className="trail-card-meta">
                <span className="trail-meta-item">
                  <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2C8.13 2 5 5.13 5 9c0 5.25 7 13 7 13s7-7.75 7-13c0-3.87-3.13-7-7-7zm0 9.5c-1.38 0-2.5-1.12-2.5-2.5s1.12-2.5 2.5-2.5 2.5 1.12 2.5 2.5-1.12 2.5-2.5 2.5z"/></svg>
                  {trail.start}
                </span>
                <span className="trail-meta-item">
                  <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm0 18c-4.41 0-8-3.59-8-8s3.59-8 8-8 8 3.59 8 8-3.59 8-8 8zm.5-13H11v6l5.25 3.15.75-1.23-4.5-2.67V7z"/></svg>
                  {trail.time}
                </span>
                {trail.dist && (
                  <span className="trail-meta-item">
                    <svg width="13" height="13" viewBox="0 0 24 24" fill="currentColor"><path d="M1.5 4v5h5L12 3l5.5 6h5V4H1.5zM12 19l-5.5-6H1.5v5h21v-5h-5L12 19z"/></svg>
                    {trail.dist}
                  </span>
                )}
              </div>
              {trail.images && <TrailCarousel title={trail.name} images={trail.images} />}
            </div>
          ))}
        </div>

        {/* ── 하단 안내 ── */}
        <p className="trail-footer-note">※ 기상 상황 및 계절에 따라 입산이 통제될 수 있습니다. 안전한 탐방을 위해 공단 안내에 따라주세요.</p>
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
    : pct <= 15  ? '#e53935'
    : pct <= 35  ? '#f57c00'
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

function App() {
  // ── React 상태(State) 선언 ──────────────────────────────────────────────
  const [states, setStates] = useState(() => {
    const init = {};
    SITE_NAMES.forEach(site => { init[site] = false; });
    return init;
  });
  const [occupiedSites, setOccupiedSites] = useState([]);

  const [connected, setConnected] = useState(false);
  const [batteryPct, setBatteryPct] = useState(null); // null = 아직 수신 전
  const [togglePage, setTogglePage] = useState(0);   // 0: B1~B6, 1: B7~B12, 2: B13
  const [engageState, setEngageState] = useState(false);
  // HH_260721 - Display operational progress independently from diagnostic health.
  const [serviceStateName, setServiceStateName] = useState('PREPARING');
  const [systemHealth, setSystemHealth] = useState('STARTING');
  const [missionPhase, setMissionPhase] = useState('INITIALIZING');
  const [missionSource, setMissionSource] = useState('none');
  const [headlightState, setHeadlightState] = useState(false); // 260708: 전조등 토글
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
  const wsRef = useRef(null);
  const wsMountedRef = useRef(false);
  const wsGenerationRef = useRef(0);
  const wsReconnectTimerRef = useRef(null);
  const idleTimerRef = useRef(null);                    // 전체 OFF 시 10초 타이머

  const [outsideHoursMsg, setOutsideHoursMsg] = useState(false); // 운영시간 외 안내 메시지
  const [selectedSite, setSelectedSite] = useState(null);         // 이미지 프리뷰 대상 사이트
  // HH_260904 - Delivery enters the campsite; recall stops at the authored
  // roadside wait pose. Keep the intent explicit through both confirmations.
  const [destinationIntent, setDestinationIntent] = useState('delivery');
  const destinationIntentRef = useRef('delivery');
  const missionDispatchActiveRef = useRef(false);
  const missionDispatchGenerationRef = useRef(0);
  const missionDispatchSiteRef = useRef('');
  const missionDispatchOwnerRef = useRef('');
  const missionAuthorityRevisionRef = useRef(0);
  const recallRequestEpochRef = useRef(0);
  const [recallRequestPending, setRecallRequestPending] = useState(false);
  const [activeRecallSite, setActiveRecallSite] = useState(null);
  const [arrivedSite, setArrivedSite] = useState(null);           // 도착 완료 사이트
  const [showArrivalComplete, setShowArrivalComplete] = useState(false); // 이용 완료 팝업
  const [isReturning, setIsReturning] = useState(false);                 // 대기·충전 장소 복귀 중
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
  const [showMoveVerify, setShowMoveVerify] = useState(false);  // 사이트명 입력 확인 팝업
  const [moveVerifyInput, setMoveVerifyInput] = useState('');
  const [moveVerifyError, setMoveVerifyError] = useState(false);
  const [missionBlockMessage, setMissionBlockMessage] = useState('');
  const [batteryReturnMessage, setBatteryReturnMessage] = useState('');
  const [batteryReturnState, setBatteryReturnState] = useState(emptyBatteryReturnState);
  const [redockStatus, setRedockStatus] = useState({
    pending: false,
    waitingForCan: false,
    status: 'idle',
    message: '',
    received: false,
  });
  // HH_260819 - Public field-operation evidence stays lightweight and separate
  // from the administrator-only, high-bandwidth telemetry session.
  const serviceMetrics = useServiceMetricsSummary();

  const openSettingsLoginModal = () => {
    setActiveModal(null);
    setLoginId('');
    setLoginPw('');
    setLoginError('');
    setActiveField('id');
    setKbCaps(false);
    setShowLoginModal(true);
  };

  const handleLogin = () => {
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
  // RETURN_WITH_CARGO is shared by delivery and recall in the controller.
  // Require the accepted/replayed recall site as explicit request intent;
  // the service state alone must never turn an ordinary delivery into recall
  // wording after a refresh or during its return leg.
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

  // ── 대기 화면 터치 핸들러 (운영시간 체크) ──────────────────────────────
  const handleWaitingClick = () => {
    if (isWithinOperatingHours()) {
      setShowWaiting(false);
    } else {
      setOutsideHoursMsg(true);
      setTimeout(() => setOutsideHoursMsg(false), 3000);
    }
  };

  // ── 모든 토글이 OFF이면 10초 후 대기 화면으로 복귀 ──────────────────────
  const resetIdleTimer = useCallback(() => {
    if (idleTimerRef.current) {
      clearTimeout(idleTimerRef.current);
      idleTimerRef.current = null;
    }
    if (!anyOn && !manualDriveActive && !showWaiting && !isReturning) {
      idleTimerRef.current = setTimeout(() => {
        setShowWaiting(true);
      }, 10000);
    }
  }, [anyOn, manualDriveActive, showWaiting, isReturning]);
    useEffect(() => {
    if (anyOn || manualDriveActive) {
      // ON이 하나라도 있으면 타이머 해제 & 대기 화면 진입 방지
      if (idleTimerRef.current) {
        clearTimeout(idleTimerRef.current);
        idleTimerRef.current = null;
      }
    } else if (!showWaiting && !isReturning) {
      // 전부 OFF + 복귀 중 아닐 때 → 10초 타이머 시작
      idleTimerRef.current = setTimeout(() => {
        setShowWaiting(true);
      }, 10000);
    } else if (isReturning && idleTimerRef.current) {
      // 복귀 중이면 기존 타이머 취소
      clearTimeout(idleTimerRef.current);
      idleTimerRef.current = null;
    }
    return () => {
      if (idleTimerRef.current) clearTimeout(idleTimerRef.current);
    };
  }, [anyOn, manualDriveActive, showWaiting, isReturning]);

  // HJ_260804 - A Guest UI mission can start while the Robot UI is on its idle
  // screen. Expose the return status as soon as that mission starts returning.
  useEffect(() => {
    if (isReturning && showWaiting) {
      setShowWaiting(false);
    }
  }, [isReturning, showWaiting]);

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
        // A delayed browser frame must not alter a newer mission. Revert the
        // optimistic presentation; the authoritative mission snapshot and
        // site-state map carried in this same response are applied below.
        setIsReturning(false);
        setMissionBlockMessage(
          data.message || '현재 운행과 일치하지 않는 이전 요청이 취소되었습니다.'
        );
        const authoritativeWaitSite = String(
          data.return_wait_site || ''
        ).trim();
        const authoritativeWaitOwner = String(
          data.return_wait_owner || ''
        ).trim();
        if (
          data.return_wait_active === true
          && SITE_NAMES.includes(authoritativeWaitSite)
          && ['operator', 'robot'].includes(authoritativeWaitOwner)
        ) {
          // The optimistic click hid the modal before the backend could reject
          // it. Restore the exact still-active service wait from the rejection
          // snapshot; campsite WAIT phases are transitions, not heartbeats, so
          // waiting for another ROS message could otherwise strand the UI.
          setArrivedSite(authoritativeWaitSite);
          setShowArrivalComplete(true);
        } else {
          setArrivedSite(null);
          setShowArrivalComplete(false);
        }
      }
      if (data.departure_failed && data.mission_retryable) {
        const retrySite = String(data.mission_retry_site || '선택 사이트');
        const retryOwner = String(data.mission_retry_owner || '');
        setMissionBlockMessage(
          data.message || `${retrySite} 출차에 실패했습니다. 같은 사이트를 다시 선택해 주세요.`
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

      if ('battery_return_pending' in data) {
        if (data.battery_return_pending) {
          setBatteryReturnState({
            pending: true,
            started: Boolean(data.battery_return_started),
            waitingForUser: Boolean(data.battery_return_waiting_for_user),
          });
          setBatteryReturnMessage(formatBatteryReturnMessage(data));
        } else {
          setBatteryReturnState(emptyBatteryReturnState());
          setBatteryReturnMessage('');
        }
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

      // 초기 연결 시: {"states": {"B1": false, ...}} 전체 상태 수신
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
        if (dispatchActive) {
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
        } else if (!dispatchIntent) {
          destinationIntentRef.current = 'delivery';
          setDestinationIntent('delivery');
          setActiveRecallSite(null);
        }
      }
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
          // A reconnect can happen while the robot is still in the bounded
          // charger-departure dwell and the service state therefore remains
          // CHARGING.  Keep the accepted recall visible instead of allowing
          // that terminal heartbeat to reopen the standby screen.
          setShowWaiting(false);
        } else {
          // The backend sends an explicit empty replay when the recall-owned
          // site mirror reaches its terminal state.  Do not infer intent from
          // the shared RETURN_WITH_CARGO state after that point.
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
        // The public Guest UI owns its recall and Return button. Robot UI may
        // observe that lifecycle but must not expose a second authority that
        // the backend will correctly reject.
        setShowArrivalComplete(missionDispatchOwnerRef.current !== 'guest');
      }
      // Service lifecycle: 0=drop-zone wait, 6=site unload wait, 9/10=returning.
      if ('service_state' in data) {
        const serviceState = Number(data.service_state);
        const nextStateName = data.service_state_name || SERVICE_STATE_NAME_BY_ID[serviceState];
        if (nextStateName) setServiceStateName(nextStateName);
        if (
          serviceState === SERVICE_STATE.DROP_ZONE_WAIT
          || serviceState === SERVICE_STATE.WAITING_FOR_CHARGING
          || serviceState === SERVICE_STATE.CHARGING
        ) {
          // HH_260721 - Charging remains a stopped standby state with destination selection enabled.
          setArrivedSite(null);
          setShowArrivalComplete(false);
          setIsReturning(false);
          // CHARGING/DROP_ZONE_WAIT is also the state during the bounded
          // departure dwell immediately after a recall is accepted.  Only
          // the backend's explicit empty robot_recall_site is completion
          // authority; until then, retain the recall screen across heartbeats.
          if (
            destinationIntentRef.current !== 'recall'
            && !missionDispatchActiveRef.current
          ) {
            setShowWaiting(true);
          }
          setShowGuestRecall(false);
          setGuestNavigateSite(null);
          // Do not clear recall intent from a terminal-state heartbeat alone.
          // A newly accepted recall remains CHARGING/WAITING while its bounded
          // station-departure dwell runs.  The backend sends the authoritative
          // empty robot_recall_site only after a return/parking leg completes.
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
          );
          setIsReturning(false);
        } else if (RETURNING_STATES.has(serviceState) || data.returning) {
          setShowArrivalComplete(false);
          setArrivedSite(null);
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
    };

    // HH_260708 - Reconnect the operator WebSocket after transient disconnects.
    ws.onclose = () => {
      if (
        wsGenerationRef.current !== connectionGeneration
        || wsRef.current !== ws
      ) return;
      wsRef.current = null;
      setConnected(false);
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
  }, [connect]);

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
    // HH_260724 - Manual driving stop must cancel Nav2 and publish OPERATOR_STOPPED.
    fetch('/ui/stop', { method: 'POST' }).catch(() => {});
    setEngageState(false);
  };

  // 260708: 전조등 ON/OFF — /ui/headlight → /platform/headlight/command
  const handleHeadlight = () => {
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

  const requestCampingSiteRecall = async (site) => {
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

  // ── 이동 중 "운행 중지" 클릭 → 전체 운행 정지 ────────────────────────────
  const handleStopMove = () => {
    // Return, docking, and parking can continue after the active-site toggle
    // has already cleared. Use the authoritative full stop for every visible
    // service-motion stop button so all motion owners are cancelled together.
    fetch('/ui/stop', { method: 'POST' }).catch(() => {});
    setEngageState(false);
  };

  // ── 이용 완료 버튼 클릭 → state=3(RETURNING) publish 요청 ──────────────
  const handleArrivalComplete = () => {
    if (
      !missionDispatchSiteRef.current
      || missionDispatchGenerationRef.current <= 0
      || !['operator', 'robot'].includes(missionDispatchOwnerRef.current)
    ) {
      setMissionBlockMessage('현재 운행의 복귀 권한이 이 화면에 없습니다.');
      return;
    }
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({
        usage_complete: true,
        site: missionDispatchSiteRef.current,
        mission_generation: missionDispatchGenerationRef.current,
      }));
    }
    setShowArrivalComplete(false);
    setArrivedSite(null);
    setIsReturning(true);
  };

  // ── 실제 토글 적용 & WebSocket 전송 ─────────────────────────────────────
  const applyToggle = (site, newState) => {
    setMissionBlockMessage('');
    const updated = {};
    SITE_NAMES.forEach(s => { updated[s] = false; });
    if (newState) updated[site] = true;
    missionDispatchActiveRef.current = Boolean(newState);

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
  const missionSelectionLocked = Boolean(
    anyOn || isReturning || activeRecallSite || recallRequestPending
  );
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
      <div className="admin-runtime-shell">
        {showLoginModal ? (
          <div className="modal-overlay">
            <div
              className="modal-box login-modal-box"
              data-ui="operator-admin-login-modal"
              onClick={e => e.stopPropagation()}
            >
              <div className="modal-header">
                <span className="modal-title">진단 관리자 인증</span>
                <button
                  className="modal-back-btn"
                  data-ui="operator-admin-login-cancel"
                  onClick={() => setShowLoginModal(false)}
                >뒤로가기</button>
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
                <button
                  className="login-submit-btn"
                  data-ui="operator-admin-login"
                  onClick={handleLogin}
                >확인</button>
                {loginError && <p className="login-error">{loginError}</p>}
                <div className="vkb-wrap">
                  <div className="vkb-field-tabs">
                    <button className={`vkb-tab${activeField === 'id' ? ' active' : ''}`} onClick={() => setActiveField('id')}>아이디</button>
                    <button className={`vkb-tab${activeField === 'pw' ? ' active' : ''}`} onClick={() => setActiveField('pw')}>비밀번호</button>
                  </div>
                  {KB_ROWS.map((row, ri) => (
                    <div key={ri} className="vkb-row">
                      {row.map(key => (
                        <button
                          key={key + ri}
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
                <button
                  className="modal-back-btn"
                  data-ui="operator-admin-exit"
                  onClick={() => setActiveModal(null)}
                >뒤로가기</button>
              </div>
              <div className="modal-body"><DiagnosticsMonitor redockStatus={redockStatus} /></div>
            </div>
          </div>
        )}
      </div>
    );
  }

  // 대기 화면: 모든 토글 OFF 상태일 때 표시, 클릭/터치 시 토글 화면으로 전환
  if (showWaiting) {
    return (
      <div className="waiting-screen" data-ui="operator-waiting-screen">
        {/* ── 상단 헤더 배너 ── */}
        <div className="waiting-header">
          {/* [REMOVED decorative sky/sun/cloud/bird/mountain elements — replaced with clean white header] */}
          {false && <svg className="wh-mountains" viewBox="0 0 1200 160" preserveAspectRatio="none" xmlns="http://www.w3.org/2000/svg">
            <polygon points="0,160 80,70 160,110 260,50 360,95 460,45 520,80 600,30 680,75 780,40 860,80 960,35 1060,70 1140,50 1200,65 1200,160" fill="#6aa8d4" opacity="0.45"/>
            <polygon points="0,160 60,100 140,130 220,80 300,115 400,65 480,100 560,55 640,90 720,60 800,95 880,55 960,85 1040,60 1120,80 1200,65 1200,160" fill="#4a8c6a" opacity="0.6"/>
            <polygon points="560,55 575,38 590,30 605,38 620,55 600,50" fill="white" opacity="0.9"/>
            <polygon points="780,40 793,26 803,18 815,26 828,40 810,35" fill="white" opacity="0.85"/>
            <polygon points="956,35 967,22 976,15 986,22 998,35 978,30" fill="white" opacity="0.8"/>
            <ellipse cx="200" cy="160" rx="260" ry="70" fill="#2d6e40"/>
            <ellipse cx="650" cy="165" rx="320" ry="75" fill="#1e5930"/>
            <ellipse cx="1050" cy="165" rx="280" ry="72" fill="#265938"/>
            <rect x="0" y="130" width="1200" height="30" fill="#2a6636"/>
            <ellipse cx="0" cy="138" rx="80" ry="18" fill="#2e7040"/>
            <ellipse cx="300" cy="140" rx="100" ry="16" fill="#347a44"/>
            <ellipse cx="700" cy="138" rx="120" ry="18" fill="#2e7040"/>
            <ellipse cx="1100" cy="140" rx="90" ry="16" fill="#2a6636"/>
            <g transform="translate(60,80)"><rect x="-3" y="30" width="6" height="22" fill="#5a3010"/><polygon points="0,0 16,35 -16,35" fill="#1a6030"/><polygon points="0,8 13,38 -13,38" fill="#228040"/></g>
            <g transform="translate(95,88)"><rect x="-2.5" y="26" width="5" height="18" fill="#5a3010"/><polygon points="0,0 13,30 -13,30" fill="#1a6030"/><polygon points="0,6 11,32 -11,32" fill="#228040"/></g>
            <g transform="translate(36,92)"><rect x="-2" y="22" width="4" height="16" fill="#5a3010"/><polygon points="0,0 11,26 -11,26" fill="#155028"/><polygon points="0,5 9,28 -9,28" fill="#1a6030"/></g>
            <g transform="translate(1100,75)"><rect x="-3" y="32" width="6" height="24" fill="#5a3010"/><polygon points="0,0 17,38 -17,38" fill="#1a6030"/><polygon points="0,8 14,40 -14,40" fill="#228040"/></g>
            <g transform="translate(1140,84)"><rect x="-2.5" y="26" width="5" height="20" fill="#5a3010"/><polygon points="0,0 14,32 -14,32" fill="#155028"/><polygon points="0,6 11,34 -11,34" fill="#1a6030"/></g>
            <g transform="translate(1165,90)"><rect x="-2" y="20" width="4" height="16" fill="#5a3010"/><polygon points="0,0 10,24 -10,24" fill="#228040"/></g>
            <g transform="translate(830,105)"><polygon points="0,35 20,-5 40,35" fill="#e05020"/><polygon points="8,35 20,8 32,35" fill="#ff6a30"/><rect x="12" y="25" width="16" height="10" fill="#1a1a1a" opacity="0.7" rx="2"/></g>
            <g transform="translate(870,110)"><polygon points="0,28 15,-2 30,28" fill="#2060c0"/><polygon points="6,28 15,4 24,28" fill="#3a80e0"/><rect x="10" y="20" width="10" height="8" fill="#1a1a1a" opacity="0.7" rx="1"/></g>
            <g transform="translate(856,128)"><ellipse cx="0" cy="5" rx="8" ry="3" fill="#5a3010"/><polygon points="-3,4 0,-8 3,4" fill="#ff4400" opacity="0.9"/><polygon points="-2,4 0,-5 2,4" fill="#ffaa00" opacity="0.85"/><polygon points="-1,4 0,-3 1,4" fill="#fff0a0" opacity="0.8"/></g>
            <g transform="translate(180,92)"><circle cx="0" cy="-2" r="5" fill="#f4c078"/><rect x="-4" y="3" width="8" height="12" fill="#3a6ab0" rx="1"/><rect x="3" y="3" width="5" height="9" fill="#8b6040" rx="1"/><line x1="-4" y1="15" x2="-6" y2="26" stroke="#3a4060" strokeWidth="2" strokeLinecap="round"/><line x1="4" y1="15" x2="6" y2="26" stroke="#3a4060" strokeWidth="2" strokeLinecap="round"/><line x1="8" y1="5" x2="12" y2="28" stroke="#888" strokeWidth="1.5" strokeLinecap="round"/><ellipse cx="0" cy="-6" rx="6" ry="2.5" fill="#c08030"/><ellipse cx="0" cy="-8" rx="3.5" ry="3" fill="#c08030"/></g>
            <g transform="translate(205,97)"><circle cx="0" cy="-2" r="4.5" fill="#f0a878"/><rect x="-3.5" y="3" width="7" height="11" fill="#c04060" rx="1"/><rect x="2.5" y="3" width="4.5" height="8" fill="#7a5030" rx="1"/><line x1="-3.5" y1="14" x2="-5" y2="24" stroke="#3a3050" strokeWidth="2" strokeLinecap="round"/><line x1="3.5" y1="14" x2="5" y2="24" stroke="#3a3050" strokeWidth="2" strokeLinecap="round"/><line x1="6" y1="5" x2="9" y2="26" stroke="#888" strokeWidth="1.5" strokeLinecap="round"/><ellipse cx="0" cy="-6" rx="5.5" ry="2" fill="#206040"/><ellipse cx="0" cy="-7.5" rx="3" ry="2.8" fill="#206040"/></g>
            <path d="M 140,160 Q 200,145 260,155 Q 320,148 380,158" stroke="#c8a870" strokeWidth="4" fill="none" opacity="0.7" strokeLinecap="round"/>
          </svg>}

          <div className="wh-content">
            <div className="wh-left">
              <div className="wh-logo">
                <img src="/월악산_국립공원_로고.jpg" alt="월악산 국립공원 로고" />
              </div>
              <div className="wh-title-block">
                <span className="wh-subtitle">국립공원공단 · Woraksan National Park</span>
                <span className="wh-main-title">월악산 <em>국립공원</em> 배송 로봇</span>
              </div>
            </div>
            <div className="wh-right-group">
              <RuntimeStatus
                systemHealth={systemHealth}
                missionPhase={missionPhase}
                batteryPolicy={currentBatteryPolicy}
              />
              <div className="wh-wifi">
                <WifiIcon level={signalLevel} />
                <span className="wh-wifi-label">WIFI</span>
              </div>
              <div className="wh-battery">
                <BatteryIcon pct={batteryPct} />
                <span className="wh-battery-label">
                  {batteryPct === null ? '–%' : `${batteryPct}%`}
                </span>
              </div>
              <LiveClock />
            </div>
          </div>
        </div>

        {/* ── 하단 콘텐츠 영역: 실증 요약 + 기존 4개 버튼 2×2 ── */}
        <div className="waiting-body">

          {/* ── 공개 실증 운행 요약: 기존 2×2 서비스 버튼은 그대로 유지 ── */}
          <ServiceEvidenceSummary
            data={serviceMetrics.data}
            loading={serviceMetrics.loading}
            error={serviceMetrics.error}
            onOpen={() => setActiveModal('service-evidence')}
          />

          {/* ── 목적지 선택 ── */}
          <button
            className="waiting-grid-btn"
            data-ui="operator-open-destination"
            onClick={handleWaitingClick}
          >
            <span className="waiting-grid-btn-icon">
              <div style={{ width: '100%', height: '100%' }}>
                <RobotAnimation />
              </div>
            </span>
            {outsideHoursMsg
              ? <span className="waiting-text outside-hours">현재는 운영시간이 아닙니다.</span>
              : <span className="waiting-grid-btn-label">목적지 선택</span>
            }
          </button>

          {/* ── 시설 안내 / 탐방로 ── */}
          {SIDE_BUTTONS.filter(btn => btn.id !== 'settings').map(btn => (
            <button
              key={btn.id}
              className="waiting-grid-btn"
              onClick={e => {
                e.stopPropagation();
                setActiveModal(btn.id);
              }}
            >
              <span className="waiting-grid-btn-icon">{btn.icon}</span>
              <span className="waiting-grid-btn-label">{btn.label}</span>
            </button>
          ))}

          {/* ── 숨겨진 진단 진입 영역 (모든 서비스 화면 공통) ── */}
          {adminEntryZone}

        </div>

        {/* ── 모달 오버레이 ── */}
        {serviceEvidenceModal}
        {activeModal && activeModal !== 'settings' && modalData && (
          <div className="modal-overlay">
            <div className="modal-box" onClick={e => e.stopPropagation()}>
              <div className="modal-header">
                <span className="modal-title">{modalData.title}</span>
                <button className="modal-back-btn" onClick={() => setActiveModal(null)}>뒤로가기</button>
              </div>
              <div className="modal-body">{modalData.content}</div>
            </div>
          </div>
        )}

        {/* HJ_260601: 게스트 호출 알림 팝업 (대기 화면) */}
        {showGuestRecall && (
          <div className="guest-recall-overlay">
            <div className="guest-recall-box">
              <p className="guest-recall-msg">이용객 호출 요청을 받았습니다</p>
            </div>
          </div>
        )}
        {/* 게스트 사이트 이동 알림 팝업 (대기 화면) */}
        {guestNavigateSite && (
          <div className="guest-recall-overlay">
            <div className="guest-recall-box">
              <p className="guest-recall-msg">{guestNavigateSite} 사이트 이용객이 도로 대기점으로 로봇을 호출했습니다</p>
            </div>
          </div>
        )}
        {batteryReturnMessage && (
          <div className="move-confirm-overlay mission-block-overlay" onClick={() => setBatteryReturnMessage('')}>
            <div className="move-confirm-box mission-block-box" onClick={e => e.stopPropagation()}>
              <p className="move-confirm-msg mission-block-msg">{batteryReturnMessage}</p>
              <div className="move-confirm-btns">
                <button
                  className="move-confirm-no mission-block-close"
                  onClick={() => setBatteryReturnMessage('')}
                >
                  확인
                </button>
              </div>
            </div>
          </div>
        )}
      </div>
    );
  }

  return (
    <div
      className="main-layout"
      data-ui="operator-control-screen"
      onClick={resetIdleTimer}
      onTouchStart={resetIdleTimer}
    >
      {adminEntryZone}

      {/* ── 상단 헤더 배너 ── */}
      <div className="control-header">
        <div className="ch-content">
          <div className="ch-left">
            <div className="ch-logo">
              <img src="/월악산_국립공원_로고.jpg" alt="월악산 국립공원 로고" />
            </div>
            <div className="ch-title-block">
              <span className="wh-subtitle">국립공원공단 · Woraksan National Park</span>
              <span className="wh-main-title">월악산 <em>국립공원</em> 배송 로봇</span>
            </div>
          </div>
          <div className="ch-right">
            <RuntimeStatus
              systemHealth={systemHealth}
              missionPhase={missionPhase}
              batteryPolicy={currentBatteryPolicy}
            />
            <div className="wh-wifi ch-wifi">
              <WifiIcon level={signalLevel} />
              <span className="wh-wifi-label">WIFI</span>
            </div>
            <div className="wh-battery ch-wifi">
              <BatteryIcon pct={batteryPct} />
              <span className="wh-battery-label">
                {batteryPct === null ? '–%' : `${batteryPct}%`}
              </span>
            </div>
            <LiveClock />
          </div>
        </div>
      </div>

      {/* ── 바디: 프리뷰 + 컨트롤 패널 ── */}
      <div className="control-body">

        {/* ── 왼쪽: 사이트 이미지 프리뷰 패널 ── */}
        <div className="preview-panel">
          {missionPhase === 'INITIALIZING' ? (
            <>
              <span className="preview-placeholder-title">초기화 중</span>
              <span className="preview-placeholder">
                센서, 위치, 지도 및 주행 시스템을 확인하고 있습니다
              </span>
            </>
          ) : selectedSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[selectedSite]}`}
                alt={`${selectedSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{selectedSite}</p>
              <p className="preview-question">
                {destinationIntent === 'recall'
                  ? '사이트 내부로 들어가지 않고 도로 측 대기점으로 로봇을 호출하시겠습니까?'
                  : '배송을 위해 사이트 내부로 이동하시겠습니까?'}
              </p>
              <div className="preview-yn-btns">
                <button
                  className="preview-yes-btn"
                  data-ui="operator-site-preview-confirm"
                  onClick={() => setShowMoveConfirm(true)}
                  disabled={recallRequestPending}
                >
                  {recallRequestPending ? '요청 중…' : '예'}
                </button>
                <button className="preview-no-btn" onClick={() => setSelectedSite(null)}>아니오</button>
              </div>
            </>
          ) : arrivedSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[arrivedSite]}`}
                alt={`${arrivedSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{arrivedSite}</p>
              <p className="preview-arrived">
                {recallArrivalPresentation
                  ? '도로 측 대기점에 도착했습니다.'
                  : '배송 로봇이 사이트 내부 목적지에 도착했습니다.'}
              </p>
              <button
                className="preview-return-btn"
                data-ui="operator-arrival-return"
                onClick={handleArrivalComplete}
              >
                {recallArrivalPresentation
                  ? '적재 완료 · 복귀'
                  : '수령 완료 · 복귀'}
              </button>
            </>
          ) : displayedReturning ? (
            <>
              <span className="preview-placeholder-title">대기·충전 장소</span>
              <p className="preview-returning">
                {serviceStateName === 'WAITING_FOR_CHARGING'
                  ? '주차를 마치고 충전기 연결을 기다리고 있습니다.'
                  : serviceStateName === 'DROP_ZONE_PARKING'
                    ? '대기·충전 장소에서 주차 중입니다.'
                    : recallReturnPresentation
                      ? '이용객의 짐을 싣고 대기·충전 장소로 복귀 중입니다.'
                      : '배송을 마치고 대기·충전 장소로 복귀 중입니다.'}
              </p>
              {serviceStateName !== 'WAITING_FOR_CHARGING' && (
                <>
                  <p className="preview-question">필요하면 아래 버튼으로 운행을 중지할 수 있습니다.</p>
                  <div className="preview-yn-btns">
                    <button className="preview-stop-btn" onClick={handleStopMove}>운행 중지</button>
                  </div>
                </>
              )}
            </>
          ) : activeSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[activeSite]}`}
                alt={`${activeSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{activeSite}</p>
              <p className="preview-moving">배송을 위해 사이트 내부로 이동 중입니다.</p>
              <p className="preview-question">필요하면 아래 버튼으로 운행을 중지할 수 있습니다.</p>
              <div className="preview-yn-btns">
                <button className="preview-stop-btn" onClick={handleStopMove}>운행 중지</button>
              </div>
            </>
          ) : activeRecallSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[activeRecallSite]}`}
                alt={`${activeRecallSite} recall site`}
                className="preview-image"
              />
              <p className="preview-site-name">{activeRecallSite} 이용객 호출</p>
              <p className="preview-moving">사이트 도로 측 대기점으로 이동 중입니다.</p>
              <p className="preview-question">필요하면 아래 버튼으로 운행을 중지할 수 있습니다.</p>
              <div className="preview-yn-btns">
                <button className="preview-stop-btn" onClick={handleStopMove}>운행 중지</button>
              </div>
            </>
          ) : manualDriveActive ? (
            <>
              <span className="preview-placeholder-title">수동 RViz 목표</span>
              <p className="preview-moving">
                {MISSION_PHASE_LABELS[missionPhase] || missionPhase}
              </p>
              <p className="preview-question">필요하면 아래 버튼으로 운행을 중지할 수 있습니다.</p>
              <div className="preview-yn-btns">
                <button className="preview-stop-btn" onClick={handleManualStop}>운행 중지</button>
              </div>
            </>
          ) : serviceStateName === 'OPERATOR_STOPPED' ? (
            <>
              <span className="preview-placeholder-title">관리자 운행 정지</span>
              <p className="preview-returning">운행이 정지되었습니다.</p>
            </>
          ) : (
            <>
              <span className="preview-placeholder-title">캠핑 사이트 선택</span>
              <span className="preview-placeholder">목적지 선택 버튼을 눌러주세요</span>
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

        {/* ── 오른쪽: 컨트롤 패널 ── */}
        <div className="app">
          <h1>{destinationIntent === 'recall' ? '이용객 호출 사이트 선택' : '배송 목적지 선택'}</h1>
          <div className="preview-yn-btns" role="group" aria-label="사이트 운행 목적">
            <button
              type="button"
              data-ui="operator-intent-delivery"
              className={destinationIntent === 'delivery' ? 'preview-yes-btn' : 'preview-no-btn'}
              aria-pressed={destinationIntent === 'delivery'}
              disabled={missionSelectionLocked}
              onClick={() => selectDestinationIntent('delivery')}
            >
              배송 · 사이트 내부 진입
            </button>
            <button
              type="button"
              data-ui="operator-intent-recall"
              className={destinationIntent === 'recall' ? 'preview-yes-btn' : 'preview-no-btn'}
              aria-pressed={destinationIntent === 'recall'}
              disabled={missionSelectionLocked}
              onClick={() => selectDestinationIntent('recall')}
            >
              이용객 호출 · 도로 대기
            </button>
          </div>
          <p className="preview-question" style={{ margin: 0, fontSize: '0.9rem' }}>
            {destinationIntent === 'recall'
              ? '텐트가 설치된 사이트를 선택하면 내부 진입 없이 도로 측 대기점으로 이동합니다.'
              : '빈 사이트를 선택하면 배송을 위해 사이트 내부로 진입합니다.'}
          </p>

          {/* ── 페이지네이션 래퍼 ── */}
          <div className="toggle-paginator">
            {/* 왼쪽 화살표 */}
            <button
              className="page-arrow left"
              onClick={() => setTogglePage(p => Math.max(0, p - 1))}
              disabled={togglePage === 0}
            >
              ◀
            </button>

            {/* 토글 그리드 (페이지별 6개 또는 나머지) */}
            <div className="toggle-grid">
              {SITE_NAMES.slice(togglePage * 6, togglePage * 6 + 6).map(site => (
                <button
                  key={site}
                  data-ui={`operator-site-${site}`}
                  className={`toggle-card ${states[site] ? 'on' : ''} ${selectedSite === site ? 'selected' : ''} ${missionSelectionLocked && !states[site] ? 'locked' : ''} ${destinationIntent === 'delivery' && occupiedSites.includes(site) ? 'occupied' : ''}`}
                  onClick={() => handleToggle(site)}
                  disabled={
                    missionSelectionLocked
                    || (
                      destinationIntent === 'delivery'
                      && occupiedSites.includes(site)
                    )
                  }
                >
                  <span className="site-label">{site}</span>
                  {states[site] && <span className="site-on-badge">ON</span>}
                  {occupiedSites.includes(site) && (
                    <span className="site-occupied-badge">
                      {destinationIntent === 'recall' ? '텐트 · 호출 가능' : '사용 중'}
                    </span>
                  )}
                </button>
              ))}
              {togglePage === Math.ceil(SITE_NAMES.length / 6) - 1 && (
                <button
                  className={`toggle-card engage-card ${engageState ? 'engage-on' : ''} ${isReturning ? 'locked' : ''}`}
                  onClick={handleEngage}
                  disabled={isReturning}
                >
                  <span className="site-label">ENGAGE</span>
                  {engageState && <span className="site-on-badge">ON</span>}
                </button>
              )}
              {togglePage === Math.ceil(SITE_NAMES.length / 6) - 1 && (
                <button
                  className={`toggle-card engage-card ${headlightState ? 'engage-on' : ''}`}
                  onClick={handleHeadlight}
                >
                  <span className="site-label">LIGHT</span>
                  {headlightState && <span className="site-on-badge">ON</span>}
                </button>
              )}
            </div>

            {/* 오른쪽 화살표 */}
            <button
              className="page-arrow right"
              onClick={() => setTogglePage(p => Math.min(Math.ceil(SITE_NAMES.length / 6) - 1, p + 1))}
              disabled={togglePage === Math.ceil(SITE_NAMES.length / 6) - 1}
            >
              ▶
            </button>
          </div>

          {/* 페이지 인디케이터 (점) */}
          <div className="page-dots">
            {Array.from({ length: Math.ceil(SITE_NAMES.length / 6) }).map((_, i) => (
              <span
                key={i}
                data-ui={`operator-site-page-${i}`}
                className={`page-dot ${togglePage === i ? 'active' : ''}`}
                onClick={() => setTogglePage(i)}
              />
            ))}
          </div>

          <button
            className="control-back-btn"
            onClick={() => setShowWaiting(true)}
            disabled={Boolean(activeSite || activeRecallSite || isReturning)}
          >
            ← 뒤로가기
          </button>
        </div>

      </div>

      {/* 운행 중에도 현재 미션을 중단하지 않고 누적 운행 기록을 조회한다. */}
      {serviceEvidenceModal}

      {/* ── 출발 최종 확인 팝업 ── */}
      {showArrivalComplete && arrivedSite && (
        <div className="arrival-complete-overlay">
          <div className="arrival-complete-box">
            <p className="arrival-complete-msg">
              {recallArrivalPresentation ? (
                <>짐을 모두 실은 후,<br /><strong>[적재 완료 · 복귀]</strong> 버튼을 눌러주세요</>
              ) : (
                <>배송 물품을 모두 내린 후,<br /><strong>[수령 완료 · 복귀]</strong> 버튼을 눌러주세요</>
              )}
            </p>
            <p className="arrival-complete-sub">
              버튼을 누르면 로봇이 대기·충전 장소로 복귀합니다.
            </p>
            <button
              className="arrival-complete-btn"
              data-ui="operator-arrival-return-confirm"
              onClick={handleArrivalComplete}
            >
              {recallArrivalPresentation
                ? '적재 완료 · 복귀'
                : '수령 완료 · 복귀'}
            </button>
          </div>
        </div>
      )}

      {showMoveConfirm && (
        <div className="move-confirm-overlay" onClick={() => setShowMoveConfirm(false)}>
          <div className="move-confirm-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg">
              {destinationIntent === 'recall' ? (
                <>
                  이용객 호출은 사이트 내부로 진입하지 않습니다.<br />
                  도로 측 대기점으로 호출하시겠습니까?
                </>
              ) : (
                <>
                  로봇이 사이트 내부까지 배송 운행을 시작합니다.<br />
                  계속하시겠습니까?
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
                예
              </button>
              <button
                className="move-confirm-no"
                onClick={() => setShowMoveConfirm(false)}
              >
                아니오
              </button>
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
                ? '지금 로봇을 호출할 사이트는'
                : '지금 배송할 사이트는'}<br />
              <span className="move-verify-site">{selectedSite}</span><br />
              입니다
            </p>
            <p className="move-verify-sub">아래에 사이트명을 입력해주세요</p>
            {moveVerifyError && (
              <div className="move-verify-error-box">
                <span className="move-verify-wrong">"{moveVerifyInput}"</span>
                <span className="move-verify-error-msg">로 이동하는 것이 맞습니까?</span>
                <p className="move-verify-retry">숫자를 다시 입력해주세요</p>
              </div>
            )}
            <input
              data-ui="operator-site-code-input"
              className={`move-verify-input${moveVerifyError ? ' error' : ''}`}
              value={moveVerifyInput}
              onChange={e => { setMoveVerifyInput(e.target.value.toUpperCase()); setMoveVerifyError(false); }}
              onFocus={() => setActiveField('moveVerify')}
              placeholder={`예: ${selectedSite}`}
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
                취소
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
                확인
              </button>
            </div>
          </div>
        </div>
      )}

      {missionBlockMessage && (
        <div className="move-confirm-overlay mission-block-overlay" onClick={() => setMissionBlockMessage('')}>
          <div className="move-confirm-box mission-block-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg mission-block-msg">{missionBlockMessage}</p>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-no mission-block-close"
                onClick={() => setMissionBlockMessage('')}
              >
                확인
              </button>
            </div>
          </div>
        </div>
      )}

      {batteryReturnMessage && (
        <div className="move-confirm-overlay mission-block-overlay" onClick={() => setBatteryReturnMessage('')}>
          <div className="move-confirm-box mission-block-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg mission-block-msg">{batteryReturnMessage}</p>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-no mission-block-close"
                onClick={() => setBatteryReturnMessage('')}
              >
                확인
              </button>
            </div>
          </div>
        </div>
      )}

      {/* HJ_260601: 게스트 호출 알림 팝업 */}
      {showGuestRecall && (
        <div className="guest-recall-overlay">
          <div className="guest-recall-box">
            <p className="guest-recall-msg">이용객 호출 요청을 받았습니다</p>
          </div>
        </div>
      )}
      {/* 게스트 사이트 이동 알림 팝업 */}
      {guestNavigateSite && (
        <div className="guest-recall-overlay">
          <div className="guest-recall-box">
            <p className="guest-recall-msg">{guestNavigateSite} 사이트 이용객이 도로 대기점으로 로봇을 호출했습니다</p>
          </div>
        </div>
      )}

    </div>
  );
}

export default App;

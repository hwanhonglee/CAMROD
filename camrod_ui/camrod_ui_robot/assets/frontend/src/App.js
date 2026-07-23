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
import RobotAnimation from './RobotAnimation';

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
});
const SERVICE_STATE_NAME_BY_ID = Object.freeze(
  Object.fromEntries(Object.entries(SERVICE_STATE).map(([name, id]) => [id, name]))
);
const SERVICE_STATE_LABELS = Object.freeze({
  // HH_260721 - Display operational progress in English on every robot UI screen.
  PREPARING: 'Preparing',
  DROP_ZONE_WAIT: 'Waiting at drop zone',
  MOVING_TO_SITE: 'Moving to site',
  SITE_ARRIVED: 'Arrived at site',
  RETURNING_TO_DROP_ZONE: 'Returning to drop zone',
  GUEST_RECALL_SERVICE: 'Guest recall requested',
  SITE_ENTRY: 'Entering site',
  UNLOAD_WAIT: 'Waiting for unloading',
  RECALL_TO_SITE_ROAD: 'Moving to recall point',
  GUEST_LOADING_WAIT: 'Waiting for loading',
  RETURN_WITH_CARGO: 'Leaving site with cargo',
  DROP_ZONE_PARKING: 'Reverse parking',
  WAITING_FOR_RETURN_REQUEST: 'Waiting for return request',
  WAITING_FOR_CHARGING: 'Waiting for charger connection',
  CHARGING: 'Charging',
  DEPARTING_CHARGER: 'Departing charger',
  DEPARTING_DROP_ZONE: 'Departing drop zone',
});
const SYSTEM_HEALTH_LABELS = Object.freeze({
  // HH_260721 - Keep health labels English and independent from service progress.
  STARTING: 'System starting',
  OK: 'System normal',
  WARNING: 'System warning',
  ERROR: 'System error',
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

// HH_260721 - Reuse one health/service presentation on waiting and destination screens.
function RuntimeStatus({ systemHealth, serviceStateName }) {
  return (
    <div className="ch-runtime-status" aria-live="polite">
      <span className={`ch-runtime-line health-${systemHealth.toLowerCase()}`}>
        <span className="ch-runtime-dot" />
        {SYSTEM_HEALTH_LABELS[systemHealth] || SYSTEM_HEALTH_LABELS.STARTING}
      </span>
      <span className="ch-runtime-line service-state">
        <span className="ch-runtime-dot" />
        {SERVICE_STATE_LABELS[serviceStateName] || serviceStateName}
      </span>
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
function DiagnosticsMonitor() {
  const [items, setItems] = useState([]);
  const [selected, setSelected] = useState(null);
  const [expanded, setExpanded] = useState({ error: true, warn: true, ok: false });
  // HH_260720 - Keep only the parking control; the removed docking stack has no UI actions.
  const [parkingOn, setParkingOn] = useState(false);

  useEffect(() => {
    const fetch_ = () =>
      fetch('/api/diagnostics')
        .then(r => r.json())
        .then(j => setItems(j.status || []))
        .catch(() => {});
    fetch_();
    const t = setInterval(fetch_, 2000);
    return () => clearInterval(t);
  }, []);

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
      {/* ── 상단 컨트롤 바 ── */}
      <div className="diag-control-bar">
        <span className="diag-control-label">Manual Parking</span>
        <button
          className={`parking-toggle-btn ${parkingOn ? 'on' : 'off'}`}
          onClick={() => setParkingOn(prev => !prev)}
        >
          <span className="parking-toggle-knob" />
          <span className="parking-toggle-text">
            {parkingOn ? 'Parking ON' : 'Parking OFF'}
          </span>
        </button>

      </div>

    <div className="diag-monitor">
      {/* ── 왼쪽: 트리 패널 ── */}
      <div className="diag-tree">
        <div className="diag-tree-header">
          Device groups
          <span className="diag-count">{items.length} items</span>
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
              '원하는 목적지(B1 ~ B12)를 선택합니다.',
              '목적지 이미지와 "이동하시겠습니까?" 를 확인합니다.',
              '[예] 버튼을 눌러 배송 로봇 출발을 확정합니다.',
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
              '로봇이 이동중에 목적지 변경은 불가능하니 신중히 선택해 주세요.',
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
              '"운행을 정지하시겠습니까?" 아래의 [예] 버튼을 누릅니다.',
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
      <svg viewBox="0 0 100 100" fill="none">
        <defs>
          <linearGradient id="facilityBg" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="#4ab0f5"/>
            <stop offset="100%" stopColor="#1565c0"/>
          </linearGradient>
          <clipPath id="facilityCircle">
            <circle cx="50" cy="50" r="47"/>
          </clipPath>
        </defs>
        <g clipPath="url(#facilityCircle)">
          <circle cx="50" cy="50" r="47" fill="url(#facilityBg)"/>
          {/* Head */}
          <circle cx="50" cy="22" r="7" fill="white"/>
          {/* Body */}
          <path d="M34 57 L34 37 Q50 22 66 37 L66 57 Z" fill="white"/>
          {/* Desk top surface */}
          <rect x="15" y="55" width="70" height="8" rx="3" fill="white"/>
          {/* Desk front panel */}
          <rect x="19" y="63" width="62" height="23" rx="4" fill="white" opacity="0.92"/>
          {/* "i" dot */}
          <circle cx="50" cy="69" r="2.5" fill="url(#facilityBg)"/>
          {/* "i" bar */}
          <rect x="47.5" y="73" width="5" height="9" rx="2" fill="url(#facilityBg)"/>
        </g>
      </svg>
    ),
    title: '시설 안내',
    content: <FacilityContent />,
  },
  {
    id: 'routes',
    label: '탐방로',
    icon: (
      <svg viewBox="0 0 100 100" fill="none">
        <defs>
          <radialGradient id="compassBezel" cx="35%" cy="35%" r="65%">
            <stop offset="0%" stopColor="#dde6ea"/>
            <stop offset="100%" stopColor="#a4b4bc"/>
          </radialGradient>
        </defs>
        {/* Ping ripple outer */}
        <circle cx="50" cy="50" r="6" fill="none" stroke="#ef4444" strokeWidth="1.8">
          <animate attributeName="r" values="6;50;6" dur="2.2s" repeatCount="indefinite"/>
          <animate attributeName="opacity" values="0.7;0;0.7" dur="2.2s" repeatCount="indefinite"/>
        </circle>
        {/* Ping ripple inner */}
        <circle cx="50" cy="50" r="6" fill="none" stroke="#ef4444" strokeWidth="2">
          <animate attributeName="r" values="6;34;6" dur="2.2s" begin="0.7s" repeatCount="indefinite"/>
          <animate attributeName="opacity" values="0.5;0;0.5" dur="2.2s" begin="0.7s" repeatCount="indefinite"/>
        </circle>
        {/* Bezel depth shadow */}
        <circle cx="52" cy="53" r="44" fill="#8a9aa4" opacity="0.65"/>
        {/* Outer bezel */}
        <circle cx="50" cy="50" r="44" fill="url(#compassBezel)"/>
        {/* Bezel inner groove */}
        <circle cx="50" cy="50" r="39" fill="none" stroke="#b0bec8" strokeWidth="1"/>
        {/* Blue face */}
        <circle cx="50" cy="50" r="38" fill="#4ab4e6"/>
        {/* Needle shadow */}
        <g transform="rotate(35, 50, 50)">
          <polygon points="50,14 58,52 50,58 42,52" fill="#0a1a2a" opacity="0.18"/>
        </g>
        {/* Compass needle (rotated ~35° → NE direction) */}
        <g transform="rotate(35, 50, 50)">
          {/* North (red) */}
          <polygon points="50,14 55,50 50,56 45,50" fill="#e53935"/>
          {/* North shadow edge */}
          <polygon points="50,14 55,50 50,56" fill="#b71c1c" opacity="0.3"/>
          {/* South (white/light) */}
          <polygon points="50,86 55,50 50,44 45,50" fill="#eceff1"/>
          {/* South shadow edge */}
          <polygon points="50,86 45,50 50,44" fill="#90a4ae" opacity="0.45"/>
        </g>
        {/* Center pivot */}
        <circle cx="50" cy="50" r="7" fill="#8a9aa4"/>
        <circle cx="50" cy="50" r="5" fill="#b8c8d0"/>
        <circle cx="48.5" cy="48.5" r="1.8" fill="rgba(255,255,255,0.65)"/>
      </svg>
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
    : pct <= 30  ? '#f57c00'
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
  const idleTimerRef = useRef(null);                    // 전체 OFF 시 10초 타이머

  const [outsideHoursMsg, setOutsideHoursMsg] = useState(false); // 운영시간 외 안내 메시지
  const [selectedSite, setSelectedSite] = useState(null);         // 이미지 프리뷰 대상 사이트
  const [arrivedSite, setArrivedSite] = useState(null);           // 도착 완료 사이트
  const [showArrivalComplete, setShowArrivalComplete] = useState(false); // 이용 완료 팝업
  const [isReturning, setIsReturning] = useState(false);                 // Drop Zone 복귀 중
  const [activeModal, setActiveModal] = useState(null);           // 현재 열린 모달 ID
  const [currentTime, setCurrentTime] = useState(new Date());    // 실시간 날짜/시간
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
    const setter = activeField === 'id' ? setLoginId : setLoginPw;
    const current = activeField === 'id' ? loginId : loginPw;
    if (key === '⌫') {
      setter(current.slice(0, -1));
    } else if (key === '⇧') {
      setKbCaps(prev => !prev);
    } else if (key === 'SPACE') {
      setter(current + ' ');
    } else {
      setter(current + (kbCaps ? key.toUpperCase() : key));
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

  // ── 1초마다 시간 갱신 ────────────────────────────────────────────────────
  useEffect(() => {
    const timer = setInterval(() => setCurrentTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  // 하나라도 ON인지 확인
  const anyOn = Object.values(states).some(v => v);

  // 현재 ON인 사이트 (이동 중인 사이트)
  const activeSite = SITE_NAMES.find(s => states[s]) || null;

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
    if (!anyOn && !showWaiting && !isReturning) {
      idleTimerRef.current = setTimeout(() => {
        setShowWaiting(true);
      }, 10000);
    }
  }, [anyOn, showWaiting, isReturning]);
    useEffect(() => {
    if (anyOn) {
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
  }, [anyOn, showWaiting, isReturning]);

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
    const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
    const host = window.location.host || 'localhost:8010';

    // FastAPI 백엔드의 WebSocket 엔드포인트에 연결 (same host:port as HTTP)
    const ws = new WebSocket(`${protocol}://${host}/ws`);

    // 연결 성공 시 → 연결 상태 녹색 표시
    ws.onopen = () => setConnected(true);

    // 서버에서 메시지 수신 시 → 상태 업데이트
    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);

      // 초기 연결 시: {"states": {"B1": false, ...}} 전체 상태 수신
      if ('states' in data) {
        setStates(prev => ({ ...prev, ...data.states }));
      }
      // 개별 토글 변경 시: {"site": "B1", "state": true} 수신
      if ('site' in data && 'state' in data) {
        setStates(prev => ({ ...prev, [data.site]: data.state }));
      }
      if ('occupied_sites' in data && Array.isArray(data.occupied_sites)) {
        setOccupiedSites(data.occupied_sites);
        setSelectedSite(prev => data.occupied_sites.includes(prev) ? null : prev);
      }
      // HH_260721 - Handle arrival and lifecycle updates through the shared service contract.
      if ('arrived' in data) {
        setArrivedSite(data.arrived);
        setShowArrivalComplete(true);
      }
      // Service lifecycle: 0=drop-zone wait, 6=site unload wait, 9/10=returning.
      if ('service_state' in data) {
        const serviceState = Number(data.service_state);
        const nextStateName = data.service_state_name || SERVICE_STATE_NAME_BY_ID[serviceState];
        if (nextStateName) setServiceStateName(nextStateName);
        if (
          serviceState === SERVICE_STATE.DROP_ZONE_WAIT
          || serviceState === SERVICE_STATE.CHARGING
        ) {
          // HH_260721 - Charging remains a stopped standby state with destination selection enabled.
          setArrivedSite(null);
          setShowArrivalComplete(false);
          setIsReturning(false);
          setShowWaiting(true);
          setShowGuestRecall(false);
          setGuestNavigateSite(null);
        } else if (ARRIVAL_STATES.has(serviceState) && data.site) {
          setArrivedSite(data.site);
          setShowArrivalComplete(true);
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
      }
      // HH_260708 - Mirror planning engage state broadcast by the backend.
      if ('engage' in data) {
        setEngageState(data.engage);
      }
      // HH_260708 - Mirror exterior headlight state from the platform light bridge.
      if ('headlight' in data) {
        setHeadlightState(data.headlight);
      }
    };

    // HH_260708 - Reconnect the operator WebSocket after transient disconnects.
    ws.onclose = () => {
      setConnected(false);
      setTimeout(connect, 2000);
    };

    ws.onerror = () => ws.close();
    wsRef.current = ws;
  }, []);

  // ── 컴포넌트 마운트/언마운트 시 WebSocket 관리 ──────────────────────────
  useEffect(() => {
    connect();
    return () => { if (wsRef.current) wsRef.current.close(); };
  }, [connect]);

  useEffect(() => {
    if (showWaiting) {
      setActiveModal(null);
      setShowLoginModal(false);
      setLoginError('');
    }
  }, [showWaiting]);

  // ── 개별 사이트 토글 핸들러 ─────────────────────────────────────────────
  const handleEngage = () => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ engage: !engageState }));
    }
  };

  // 260708: 전조등 ON/OFF — /ui/headlight → /platform/headlight/command
  const handleHeadlight = () => {
    const next = !headlightState;
    fetch(`/ui/headlight?value=${next}`, { method: 'POST' })
      .then((res) => { if (res.ok) setHeadlightState(next); })
      .catch(() => {});
  };

  const handleToggle = (site) => {
    if (occupiedSites.includes(site)) {
      return;
    }
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
  const handleConfirmMove = () => {
    if (selectedSite && !occupiedSites.includes(selectedSite)) {
      applyToggle(selectedSite, true);
      setSelectedSite(null);
    }
  };

  // ── 이동중 "Yes" 클릭 → 운행 정지 (OFF) ─────────────────────────────────
  const handleStopMove = () => {
    if (activeSite) {
      applyToggle(activeSite, false);
    }
  };

  // ── 이용 완료 버튼 클릭 → state=3(RETURNING) publish 요청 ──────────────
  const handleArrivalComplete = () => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify({ usage_complete: true }));
    }
    setShowArrivalComplete(false);
    setArrivedSite(null);
    setIsReturning(true);
  };

  // ── 실제 토글 적용 & WebSocket 전송 ─────────────────────────────────────
  const applyToggle = (site, newState) => {
    const updated = {};
    SITE_NAMES.forEach(s => { updated[s] = false; });
    if (newState) updated[site] = true;

    setStates(updated);

    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      SITE_NAMES.forEach(s => {
        const st = updated[s];
        if (st !== states[s]) {
          wsRef.current.send(JSON.stringify({ site: s, state: st }));
        }
      });
    }
  };

  // ── JSX 렌더링 ─────────────────────────────────────────────────────────
  // ── 공통 시간 문자열 (대기/컨트롤 화면 공유) ──────────────────────────────
  const dateStr = currentTime.toLocaleDateString('ko-KR', { year: 'numeric', month: 'long', day: 'numeric', weekday: 'short' });
  const timeStr = currentTime.toLocaleTimeString('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });

  // 대기 화면: 모든 토글 OFF 상태일 때 표시, 클릭/터치 시 토글 화면으로 전환
  if (showWaiting) {
    const modalData = SIDE_BUTTONS.find(b => b.id === activeModal);
    return (
      <div className="waiting-screen">
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
                serviceStateName={serviceStateName}
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
              <div className="wh-right">
                <span className="wh-date">{dateStr}</span>
                <span className="wh-time">{timeStr}</span>
              </div>
            </div>
          </div>
        </div>

        {/* ── 하단 콘텐츠 영역: 3개 버튼 1×3 ── */}
        <div className="waiting-body">

          {/* ── 목적지 선택 ── */}
          <button className="waiting-grid-btn" onClick={handleWaitingClick}>
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

          {/* ── 숨겨진 진단 진입 영역 (우측 끝 3초 장누르기) ── */}
          <div
            className="diag-secret-zone"
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

        </div>

        {/* ── 진단 로그인 모달 ── */}
        {showLoginModal && (
          <div className="modal-overlay">
            <div className="modal-box login-modal-box" onClick={e => e.stopPropagation()}>
              <div className="modal-header">
                <span className="modal-title">진단 관리자 인증</span>
                <button className="modal-back-btn" onClick={() => setShowLoginModal(false)}>뒤로가기</button>
              </div>
              <div className="modal-body login-modal-body">
                <div className="login-fields-row">
                  <div className="login-field">
                    <label>아이디</label>
                    <input
                      type="text"
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
                  onClick={handleLogin}
                >
                  확인
                </button>
                {loginError && <p className="login-error">{loginError}</p>}

                {/* ── 가상 키보드 ── */}
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
        )}

        {/* ── 모달 오버레이 ── */}
        {activeModal && modalData && (
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
              <p className="guest-recall-msg">로봇이 이용객의 위치로 이동 중입니다</p>
            </div>
          </div>
        )}
        {/* 게스트 사이트 이동 알림 팝업 (대기 화면) */}
        {guestNavigateSite && (
          <div className="guest-recall-overlay">
            <div className="guest-recall-box">
              <p className="guest-recall-msg">{guestNavigateSite} 사이트에서 호출되었습니다</p>
            </div>
          </div>
        )}
      </div>
    );
  }

  return (
    <div className="main-layout" onClick={resetIdleTimer} onTouchStart={resetIdleTimer}>

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
              serviceStateName={serviceStateName}
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
            <div className="wh-right">
              <span className="wh-date">{dateStr}</span>
              <span className="wh-time">{timeStr}</span>
            </div>
          </div>
        </div>
      </div>

      {/* ── 바디: 프리뷰 + 컨트롤 패널 ── */}
      <div className="control-body">

        {/* ── 왼쪽: 사이트 이미지 프리뷰 패널 ── */}
        <div className="preview-panel">
          {selectedSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[selectedSite]}`}
                alt={`${selectedSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{selectedSite}</p>
              <p className="preview-question">이동하시겠습니까?</p>
              <div className="preview-yn-btns">
                <button className="preview-yes-btn" onClick={() => setShowMoveConfirm(true)}>예</button>
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
              <p className="preview-arrived">배송 로봇이 목적지에 도착했습니다.</p>
              <button className="preview-return-btn" onClick={handleArrivalComplete}>
                복귀
              </button>
            </>
          ) : isReturning ? (
            <>
              <span className="preview-placeholder-title">Drop Zone</span>
              <p className="preview-returning">로봇이 Drop Zone (대기 장소)로 이동중입니다.</p>
            </>
          ) : activeSite ? (
            <>
              <img
                src={`${process.env.PUBLIC_URL}/${SITE_IMAGES[activeSite]}`}
                alt={`${activeSite} site`}
                className="preview-image"
              />
              <p className="preview-site-name">{activeSite}</p>
              <p className="preview-moving">배송 로봇이 이동중 입니다.</p>
              <p className="preview-question">운행을 정지하시겠습니까?</p>
              <div className="preview-yn-btns">
                <button className="preview-stop-btn" onClick={handleStopMove}>예</button>
              </div>
            </>
          ) : (
            <>
              <span className="preview-placeholder-title">Camping Site Viewer</span>
              <span className="preview-placeholder">목적지 선택 버튼을 눌러주세요</span>
            </>
          )}
        </div>

        {/* ── 오른쪽: 컨트롤 패널 ── */}
        <div className="app">
          <h1>목적지 선택</h1>

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
                  className={`toggle-card ${states[site] ? 'on' : ''} ${selectedSite === site ? 'selected' : ''} ${anyOn && !states[site] ? 'locked' : ''} ${isReturning ? 'locked' : ''} ${occupiedSites.includes(site) ? 'occupied' : ''}`}
                  onClick={() => handleToggle(site)}
                  disabled={isReturning || occupiedSites.includes(site)}
                >
                  <span className="site-label">{site}</span>
                  {states[site] && <span className="site-on-badge">ON</span>}
                  {occupiedSites.includes(site) && <span className="site-occupied-badge">사용 중</span>}
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
              <span key={i} className={`page-dot ${togglePage === i ? 'active' : ''}`} onClick={() => setTogglePage(i)} />
            ))}
          </div>

          <button className="control-back-btn" onClick={() => setShowWaiting(true)} disabled={!!activeSite || isReturning}>← 뒤로가기</button>
        </div>

      </div>

      {/* ── 출발 최종 확인 팝업 ── */}
      {showArrivalComplete && arrivedSite && (
        <div className="arrival-complete-overlay">
          <div className="arrival-complete-box">
            <p className="arrival-complete-msg">
              짐을 내려놓은 후,<br /><strong>[이용 완료]</strong> 버튼을 눌러주세요
            </p>
            <p className="arrival-complete-sub">
              이용 완료 버튼을 누르면 로봇이 이동합니다.
            </p>
            <button className="arrival-complete-btn" onClick={handleArrivalComplete}>
              복귀
            </button>
          </div>
        </div>
      )}

      {showMoveConfirm && (
        <div className="move-confirm-overlay" onClick={() => setShowMoveConfirm(false)}>
          <div className="move-confirm-box" onClick={e => e.stopPropagation()}>
            <p className="move-confirm-msg">
              로봇이 출발하면 정차할 수 없습니다.<br />이동하시겠습니까?
            </p>
            <div className="move-confirm-btns">
              <button
                className="move-confirm-yes"
                onClick={() => {
                  setShowMoveConfirm(false);
                  setMoveVerifyInput('');
                  setMoveVerifyError(false);
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
              지금 이동하시는 사이트는<br />
              <span className="move-verify-site">{selectedSite}</span><br />
              입니다
            </p>
            <p className="move-verify-sub">아래에 사이트명을 입력해주세요</p>
            {moveVerifyError && (
              <div className="move-verify-error-box">
                <span className="move-verify-wrong">"{moveVerifyInput}"</span>
                <span className="move-verify-error-msg">로 이동하시는게 맞으십니까?</span>
                <p className="move-verify-retry">숫자를 다시 입력해주세요</p>
              </div>
            )}
            <input
              className={`move-verify-input${moveVerifyError ? ' error' : ''}`}
              value={moveVerifyInput}
              onChange={e => { setMoveVerifyInput(e.target.value); setMoveVerifyError(false); }}
              placeholder={`예: ${selectedSite}`}
              autoFocus
            />
            <div className="move-confirm-btns">
              <button
                className="move-confirm-no"
                onClick={() => { setShowMoveVerify(false); setMoveVerifyError(false); }}
              >
                취소
              </button>
              <button
                className="move-confirm-yes"
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

      {/* HJ_260601: 게스트 호출 알림 팝업 */}
      {showGuestRecall && (
        <div className="guest-recall-overlay">
          <div className="guest-recall-box">
            <p className="guest-recall-msg">로봇이 이용객의 위치로 이동 중입니다</p>
          </div>
        </div>
      )}
      {/* 게스트 사이트 이동 알림 팝업 */}
      {guestNavigateSite && (
        <div className="guest-recall-overlay">
          <div className="guest-recall-box">
            <p className="guest-recall-msg">{guestNavigateSite} 사이트에서 호출되었습니다</p>
          </div>
        </div>
      )}

    </div>
  );
}

export default App;

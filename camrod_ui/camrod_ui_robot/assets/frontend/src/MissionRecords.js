import React, { useEffect, useMemo, useState } from 'react';
import './MissionRecords.css';

const ENDPOINT = '/api/mission-records?limit=100';
const isObject = value => value !== null && typeof value === 'object' && !Array.isArray(value);
const number = value => typeof value === 'number' && Number.isFinite(value) && value >= 0 ? value : null;
const count = value => number(value) === null ? '—' : Math.trunc(value).toLocaleString('ko-KR');
// HH_260915 - A measured nonzero boundary interval must not round to "0 m".
const metres = value => number(value) === null ? '—' : `${value.toLocaleString('ko-KR',
  value > 0 && value < 0.1 ? { maximumSignificantDigits: 3 } : { maximumFractionDigits: 1 })} m`;
const kilometres = value => {
  if (number(value) === null) return '—';
  const km = value / 1000;
  return `${km > 0 && km < 0.001 ? km.toLocaleString('ko-KR', { maximumSignificantDigits: 3 }) : km.toFixed(3)} km`;
};
const seconds = value => number(value) === null ? '—' : `${Math.round(value).toLocaleString('ko-KR')}초`;
const modeLabel = value => ({ auto: '자율', manual: '수동', unknown: '미확인' }[value] || '미확인');
const intentLabel = value => ({ delivery: '일반 주행', recall: '호출', return: '복귀' }[value] || '유형 미확인');
const resultLabel = value => ({ active: '진행 중', paused: '일시 중지', completed: '완료',
  interrupted: '중단', cancelled: '취소' }[value] || value || '미수신');
// HH_260915 - These labels describe recorded evidence, never commanded motion.
const phaseLabel = value => ({ outbound: '사이트로 이동', site: '사이트 작업', return: '복귀',
  parking: '후진 주차', outside: '미션 외', MOVING_TO_SITE: '사이트로 이동',
  OPERATOR_STOPPED: '운영자 정지', ROAD_HANDOFF_READY: '도로 주행 인계' }[value] || value || '단계 미수신');
const eventLabel = value => ({ mission_started: '미션 시작', attempt_started: '요청 접수',
  phase: '단계 변경', return_requested: '복귀 요청', stop_requested: '정지 요청',
  mission_cancelled: '미션 취소 요청', mode_changed: '제어 모드 전환', gate_changed: '안전 게이트 상태 변경',
  stop_reason_changed: '정지 사유 변경', stopped: '정지 감지', resumed: '이동 재개',
  completed: '미션 완료', interrupted: '미션 중단', cancelled: '미션 취소' }[value] || value || '이벤트 미수신');
const importantEvents = new Set(['mission_started', 'phase', 'return_requested', 'stop_requested',
  'mission_cancelled', 'mode_changed', 'stop_reason_changed', 'stopped', 'resumed', 'completed', 'interrupted', 'cancelled']);
const knownReasons = [
  [/operator_stop|operator.*stop|manual_stop|ws_stop/i, '운영자 정지 요청'],
  [/obstacle|collision/i, '장애물·충돌 관련 정지 보고'],
  [/estop|emergency_stop/i, '비상 정지'],
  [/not_engaged|engage_false|disengaged/i, '자율주행 실행 해제'],
  [/drive_enable/i, '주행 허가 상태 관련'],
  [/stale|timeout|missing/i, '입력 지연·누락'],
  [/control_mode|manual|rc_mode/i, '제어 모드·수동 개입 관련'],
  [/error|fault|unhealthy/i, '장치·시스템 오류 보고'],
];
function reasonLabel(value) {
  if (!value) return '사유가 함께 보고되지 않음';
  const text = String(value);
  if (/^(?:reasons?\s*=\s*)?none\s*$/i.test(text.trim())) return '구체적인 정지 사유 없음으로 보고됨';
  if (text.startsWith('CAN(1)/RC(0) observed mode;')) return '플랫폼 CAN/RC 모드 변경 관측 (명령 소유자 인증 아님)';
  const reasons = knownReasons.filter(([pattern]) => pattern.test(text)).map(([, label]) => label);
  return reasons.length ? reasons.join(' · ') : text;
}

function DistanceBar({ values }) {
  const parts = [['auto', '자율', values?.autonomous_m], ['manual', '수동', values?.manual_m],
    ['unknown', '모드 미확인', values?.unknown_m]];
  const sum = parts.reduce((total, [, , value]) => total + (number(value) || 0), 0);
  const complete = parts.every(([, , value]) => number(value) !== null);
  // Missing buckets or inconsistent totals must not masquerade as a 100% split.
  const consistent = complete && number(values?.total_m) !== null
    && Math.abs(sum - values.total_m) <= Math.max(0.000001, sum * 0.000001);
  return <div className="mission-records-distance-visual">
    {consistent && sum > 0 ? <div className="mission-records-distance-bar" role="img"
      aria-label={`거리 구성: ${parts.map(([, label, value]) => `${label} ${metres(value)}`).join(', ')}`}>
      {parts.map(([key, label, value]) => value > 0 && <span key={key} className={`distance-${key}`}
        style={{ width: `${value / sum * 100}%` }} title={`${label} ${metres(value)}`} />)}
    </div> : <p className="mission-records-note">{consistent ? '기록된 이동 거리 0 m · 이동 비율 없음' : '거리 구성 확인 대기 · 일부 값 누락 또는 합계 불일치'}</p>}
    <div className="mission-records-distance-legend">{parts.map(([key, label, value]) => <span key={key}>
      <i className={`distance-${key}`} aria-hidden="true" />{label} <strong>{metres(value)}</strong>
    </span>)}</div>
  </div>;
}

function ObservedJourney({ mission, events }) {
  const stages = [];
  events.forEach(event => {
    if (event.phase && stages[stages.length - 1]?.phase !== event.phase) stages.push({ phase: event.phase, at: event.at });
  });
  if (mission.phase && stages[stages.length - 1]?.phase !== mission.phase) stages.push({ phase: mission.phase, at: null });
  return <section className="mission-records-journey" aria-label="관측된 미션 단계">
    <h5>실제로 기록된 진행 단계</h5>
    <p className="mission-records-note">위치 경로가 아닌 상태 기록입니다. 확인되지 않은 도착·주차·도킹은 표시하지 않습니다.</p>
    {stages.length ? <ol>{stages.slice(-8).map((stage, index) => <li key={`${stage.phase}-${index}`} data-observed-stage={stage.phase}>
      <span className="mission-records-stage-number">{index + 1}</span><strong>{phaseLabel(stage.phase)}</strong>
      <small>{stage.at ? missionDate(stage.at) : '최종 단계 필드에서 확인'}</small>
    </li>)}</ol> : <p>관측된 단계 없음</p>}
    {(mission.events_truncated || stages.length > 8) && <p className="mission-records-warning">최근 관측 단계만 표시합니다. 전체 이력은 events JSONL에서 확인하세요.</p>}
  </section>;
}

// HH_260915 - ISO timestamps are authoritative. Numeric strings are not dates.
export function missionDate(value) {
  if (typeof value !== 'string' || !/^\d{4}-\d{2}-\d{2}(?:T.*)?$/.test(value)) return '미수신';
  if (/^\d{4}-\d{2}-\d{2}$/.test(value)) return value;
  const stamp = Date.parse(value);
  return Number.isFinite(stamp) ? new Date(stamp).toLocaleString('ko-KR', {
    timeZone: 'Asia/Seoul', hour12: false,
  }) : '미수신';
}

function elapsedLabel(value, now) {
  const stamp = typeof value === 'string' && /^\d{4}-\d{2}-\d{2}T/.test(value) ? Date.parse(value) : NaN;
  if (!Number.isFinite(stamp)) return '시각 미수신';
  if (stamp > now + 1000) return '서버 시각이 현재보다 앞섬';
  return `${Math.max(0, Math.floor((now - stamp) / 1000))}초 전`;
}

function rawCanLabel(value) {
  if (value === 'disabled') return '원시 CAN 저장 꺼짐 · 디코딩된 주행 정보와 별도';
  if (value === 'active' || value === 'listening') return '원시 CAN 저장 활성 · 버스 수신 대기/기록';
  if (value === 'error') return '원시 CAN 저장 오류';
  if (value === 'unavailable' || value === 'not_available') return '원시 CAN 입력 사용 불가';
  return `원시 CAN 상태: ${typeof value === 'string' && value ? value : '미수신'}`;
}

function useMissionRecords() {
  const [data, setData] = useState(null);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(true);
  const [requestKey, setRequestKey] = useState(0);
  useEffect(() => {
    let active = true;
    let timer;
    let timeout;
    let controller;
    const load = async () => {
      controller = new AbortController();
      setLoading(true);
      try {
        // A timed-out fetch must not publish a late result even if a transport
        // ignores AbortSignal; Promise.race also bounds the JSON decode wait.
        const body = await Promise.race([
          (async () => {
            const response = await fetch(ENDPOINT, { method: 'GET', cache: 'no-store', signal: controller.signal });
            const result = await response.json();
            if (!response.ok) throw new Error(`기록기 사용 불가 (HTTP ${response.status})`);
            if (!isObject(result) || result.schema_version !== 1 || !isObject(result.recorder)
                || typeof result.recorder.status !== 'string'
                || !isObject(result.lifetime) || !Array.isArray(result.missions) || !Array.isArray(result.sites)) {
              throw new Error('미션 기록 응답 형식 오류');
            }
            return result;
          })(),
          new Promise((_, reject) => {
            timeout = setTimeout(() => { controller.abort(); reject(new Error('미션 기록 응답 시간 초과 (8초)')); }, 8000);
          }),
        ]);
        if (active) { setData(body); setError(''); }
      } catch (cause) {
        if (active) setError(cause.message || '미션 기록을 불러오지 못했습니다.');
      } finally {
        clearTimeout(timeout);
        if (active) { setLoading(false); timer = setTimeout(load, 3000); }
      }
    };
    load();
    return () => { active = false; clearTimeout(timer); clearTimeout(timeout); controller?.abort(); };
  }, [requestKey]);
  return { data, error, loading, refresh: () => setRequestKey(key => key + 1) };
}

function Distances({ values }) {
  return <dl className="mission-records-distances">
    <div><dt>자율</dt><dd>{metres(values?.autonomous_m)}</dd></div>
    <div><dt>수동</dt><dd>{metres(values?.manual_m)}</dd></div>
    <div><dt>모드 미확인</dt><dd>{metres(values?.unknown_m)}</dd></div>
    <div><dt>합계</dt><dd>{metres(values?.total_m)} <small>{kilometres(values?.total_m)}</small></dd></div>
  </dl>;
}

function MissionDetails({ mission }) {
  const events = Array.isArray(mission.events) ? mission.events : [];
  const files = isObject(mission.files) ? Object.entries(mission.files) : [];
  const [showAll, setShowAll] = useState(false);
  useEffect(() => { setShowAll(false); }, [mission.id]);
  const shownEvents = showAll ? events : events.filter(event => importantEvents.has(event.event)
    // Unrecognized future event types stay visible instead of silently vanishing.
    || !['attempt_started', 'gate_changed'].includes(event.event));
  const stopEvents = events.filter(event => ['stopped', 'stop_reason_changed', 'stop_requested'].includes(event.event));
  const stopReasons = [...new Set(stopEvents.map(event => reasonLabel(event.reason)))];
  const modeEvents = events.filter(event => event.event === 'mode_changed');
  const finished = ['completed', 'interrupted', 'cancelled'].includes(mission.result);
  const fileLabels = {
    decoded_can: ['CAN 해독 표본', 'telemetry JSONL · 속도, 모드, 배터리, 모터 값 등 플랫폼 메시지'],
    telemetry: ['CAN 해독 표본', 'telemetry JSONL · 속도, 모드, 배터리, 모터 값 등 플랫폼 메시지'],
    events: ['상태·이벤트 이력', 'events JSONL · 단계, 모드 전환, 정지·재개와 보고된 사유'],
    raw_can: ['원시 CAN 프레임', 'raw CAN JSONL · 별도 CAN 인터페이스를 켠 경우에만 ID와 바이트 저장'],
  };
  return <section className="mission-records-detail" aria-label="선택 미션 상세">
    <div className="mission-records-detail-heading"><div>
      <p className="mission-records-eyebrow">선택한 왕복 미션 · {missionDate(mission.date)} · #{count(mission.sequence)}</p>
      <h4>{mission.site || '위치 미수신'} · {mission.intent === 'return' ? '단독 복귀' : `${intentLabel(mission.intent)} · 복귀 묶음`}</h4>
      <p>{mission.name || mission.id || '이름 미수신'}</p>
    </div><span className={`mission-records-result result-${mission.result}`}>{resultLabel(mission.result)}</span></div>
    <p>관측 단계 <strong>{phaseLabel(mission.phase)}</strong> · {finished ? '마지막 관측 모드' : '현재 모드'} <strong>{modeLabel(mission.current_mode)}</strong></p>
    {mission.incomplete && <p className="mission-records-warning" role="alert">일부 기록이 누락된 미션입니다. 거리·이벤트를 완전한 실증 결과로 확정하지 않습니다.</p>}
    <Distances values={mission} />
    <DistanceBar values={mission} />
    <ObservedJourney mission={mission} events={events} />
    <section className="mission-records-stop-summary" aria-label="정지와 수동 개입 요약">
      <div><span className="mission-records-eyebrow">정지 기록</span><strong>{count(mission.stop_count)}회 <small>· {seconds(mission.stop_duration_s)}</small></strong>
        <p>멈춰 있는 동안 거리는 늘지 않고 정지 시간이 기록됩니다.</p></div>
      <div><span className="mission-records-eyebrow">수동 개입</span><strong>{count(mission.manual_interventions)}회 <small>· {metres(mission.manual_m)}</small></strong>
        <p>관측 모드 전환 {modeEvents.length}건{mission.events_truncated ? ' (최근 이력)' : ''}</p></div>
      <div className="mission-records-reasons"><span className="mission-records-eyebrow">보고된 정지 사유</span>
        {stopReasons.length ? <ul>{stopReasons.map(reason => <li key={reason}>{reason}</li>)}</ul>
          : <p>{number(mission.stop_count) > 0 ? '최근 이벤트에 정지 사유가 없습니다. 전체 events JSONL을 확인하세요.' : '최근 이벤트에서 정지 사유 보고 없음'}</p>}
        <small>메시지에 보고된 내용이며 물리적 원인을 추정하지 않습니다.</small>
      </div>
    </section>
    <dl className="mission-records-facts">
      <div><dt>시작</dt><dd>{missionDate(mission.started_at)}</dd></div>
      <div><dt>종료</dt><dd>{missionDate(mission.ended_at)}</dd></div>
      <div><dt>경과시간</dt><dd>{seconds(mission.duration_s)}</dd></div>
      <div><dt>요청 시도</dt><dd>{count(mission.attempt_count)}회</dd></div>
    </dl>
    <div className="mission-records-timeline-heading"><h5>주요 사건 타임라인</h5>
      <label><input type="checkbox" aria-label="전체 이벤트 표시" checked={showAll} onChange={event => setShowAll(event.target.checked)} /> 전체 이벤트 표시</label>
    </div>
    <p className="mission-records-note">{showAll ? '수신된 이벤트' : '정지·재개·모드 전환·미션 단계 중심'} {shownEvents.length}건 표시
      {mission.events_truncated ? ` · 전체 ${count(mission.event_count)}건 중 최근 ${events.length}건을 조회한 화면입니다.` : ''}</p>
    {shownEvents.length ? <ol className="mission-records-events">{shownEvents.map((event, index) => <li key={`${event.at || ''}-${index}`} data-event-kind={event.event}>
      <time>{missionDate(event.at)}</time><strong>{eventLabel(event.event)}</strong>
      <span>{event.event === 'mode_changed' && event.previous_mode ? `${modeLabel(event.previous_mode)} → ` : ''}{modeLabel(event.current_mode || event.mode)} · {phaseLabel(event.phase)}</span>
      {(event.reason || ['stopped', 'stop_reason_changed'].includes(event.event)) && <span className="mission-records-event-reason">{reasonLabel(event.reason)}</span>}
      {number(event.stop_duration_s) !== null && <span>앞선 정지 시간 {seconds(event.stop_duration_s)}</span>}
      <details><summary>원문·출처 확인</summary><dl><div><dt>이벤트 / 단계</dt><dd><code>{event.event || '미수신'} / {event.phase || '미수신'}</code></dd></div>
        <div><dt>사유 원문</dt><dd><code>{event.reason || '기록 없음'}</code></dd></div><div><dt>출처</dt><dd><code>{event.source || '미수신'}</code></dd></div></dl></details>
    </li>)}</ol> : <p>이 미션의 이벤트 기록이 없습니다.</p>}
    <details className="mission-records-technical"><summary>CAN·상태 정보는 어디에 저장되나요? 파일과 원본 확인</summary>
    <p>플랫폼 메시지의 <strong>CAN 해독 값</strong>과 미션의 <strong>상태·이벤트</strong>는 종류별 JSONL에 한 줄씩 저장됩니다.
      <strong> 원시 CAN ID·바이트</strong>는 별도 수집을 켜야 저장되며 해독 값과 같은 파일이 아닙니다.</p>
    <p className="mission-records-note">이 화면은 snapshot.json의 요약을 읽습니다. CAN 표본 전체를 직접 표시하는 화면은 아닙니다. 저장 위치만 표시하며 다운로드·제어 요청은 보내지 않습니다.</p>
    {files.length ? <dl className="mission-records-files">{files.map(([name, path]) => <div key={name}>
      <dt>{fileLabels[name]?.[0] || name}</dt><dd>{fileLabels[name]?.[1] || '기록기가 제공한 추가 파일'}
        {(Array.isArray(path) ? path : [path]).map((file, index) => <code key={index}>{typeof file === 'string' ? file : '경로 형식 미확인'}</code>)}
      </dd>
    </div>)}</dl> : <p>파일 경로 미수신</p>}
    </details>
  </section>;
}

function MissionRecordsBody() {
  const { data, error, loading, refresh } = useMissionRecords();
  const [site, setSite] = useState('');
  const [intent, setIntent] = useState('');
  const [selectedId, setSelectedId] = useState(null);
  const [now, setNow] = useState(Date.now());
  useEffect(() => { const timer = setInterval(() => setNow(Date.now()), 1000); return () => clearInterval(timer); }, []);
  const records = useMemo(() => {
    const unique = new Map();
    if (isObject(data?.current_mission)) unique.set(data.current_mission.id, data.current_mission);
    (data?.missions || []).forEach(item => { if (isObject(item) && !unique.has(item.id)) unique.set(item.id, item); });
    return [...unique.values()];
  }, [data]);
  const sites = [...new Set([...records, ...(data?.sites || [])].map(item => item.site).filter(Boolean))].sort((a, b) => a.localeCompare(b, 'ko', { numeric: true }));
  const accepts = item => (!site || item.site === site) && (!intent || item.intent === intent);
  const filtered = records.filter(accepts);
  const selected = filtered.find(item => item.id === selectedId) || filtered[0];
  const recorder = data?.recorder;
  const unavailable = !data || ['NOT_STARTED', 'ERROR'].includes(recorder?.status);
  const hasIssue = error || (recorder && recorder.status !== 'READY' && recorder.status !== 'CLOSED');
  return <div className="mission-records-body">
    <div className="mission-records-status" aria-live="polite">
      <div><strong>기록기: {recorder?.status || (loading ? '조회 중' : '미수신')}</strong>
        <span>스냅샷 {missionDate(data?.generated_at)} · {elapsedLabel(data?.generated_at, now)}</span>
        <span>마지막 주행 표본 {missionDate(recorder?.last_sample_at)} · {elapsedLabel(recorder?.last_sample_at, now)}</span>
        <span>{rawCanLabel(recorder?.raw_can_status)}</span>
        <span>환경 {recorder?.environment || '미수신'} · 로봇 {recorder?.robot_id || '미수신'}</span>
        <details className="mission-records-storage"><summary>저장 위치 보기</summary><span className="mission-records-path">저장 루트: {recorder?.storage_root || '미수신'}</span></details>
      </div>
      <button type="button" onClick={refresh}>기록 새로고침</button>
    </div>
    {loading && !data && <p>별도 미션 기록을 조회하고 있습니다.</p>}
    {hasIssue && <p className="mission-records-warning" role="alert">
      {error || recorder.error || `기록기 상태: ${recorder.status}`} · {data ? '최근 응답을 표시합니다. 최신 기록으로 확정하지 않습니다.' : '기록기 미응답을 0건으로 표시하지 않습니다.'}
    </p>}
    {recorder?.status === 'CLOSED' && <p className="mission-records-warning">기록기가 종료되어 저장된 마지막 기록을 표시합니다.</p>}
    {!unavailable && <>
      <div className="mission-records-totals" aria-label="별도 미션 누적 지표">
        <h4>새 기록기 누적 · 기존 실증 누적과 별도</h4>
        <div className="mission-records-headline"><div><span>총 이동 거리</span><strong>{kilometres(data.lifetime.total_m)}</strong><small>{metres(data.lifetime.total_m)}</small></div>
          <div><span>완료한 미션</span><strong>{count(data.lifetime.completed_count)} <small>/ {count(data.lifetime.mission_count)}건</small></strong><small>완료 / 전체 기록</small></div>
          <div><span>수동 개입</span><strong>{count(data.lifetime.manual_interventions)}<small>회</small></strong><small>수동 거리 {metres(data.lifetime.manual_m)}</small></div>
        </div><DistanceBar values={data.lifetime} />
        <details className="mission-records-number-details"><summary>거리 수치 상세</summary><Distances values={data.lifetime} /></details>
        <p>미션 {count(data.lifetime.mission_count)}건 · 완료 {count(data.lifetime.completed_count)}건 · 수동 개입 {count(data.lifetime.manual_interventions)}회</p>
      </div>
      <div className="mission-records-filters">
        <label>사이트 <select aria-label="미션 사이트 필터" value={site} onChange={event => setSite(event.target.value)}>
          <option value="">전체 사이트</option>{sites.map(name => <option key={name} value={name}>{name}</option>)}
        </select></label>
        <label>요청 유형 <select aria-label="미션 요청 유형 필터" value={intent} onChange={event => setIntent(event.target.value)}>
          <option value="">전체 유형</option><option value="delivery">배송</option><option value="recall">호출</option><option value="return">복귀</option>
        </select></label><span>최근 최대 100건 · 선택은 기록 조회만 수행합니다.</span>
      </div>
      <div className="mission-records-table-wrap"><table className="mission-records-table">
        <caption>날짜·순번별 왕복 미션</caption><thead><tr><th>날짜 / 순번 / 이름</th><th>사이트 / 유형</th><th>자율</th><th>수동</th><th>미확인</th><th>합계</th><th>결과 / 수동 개입</th></tr></thead>
        <tbody>{filtered.map(mission => <tr key={mission.id} className={selected?.id === mission.id ? 'selected' : ''}>
          <td><button type="button" aria-pressed={selected?.id === mission.id} onClick={() => setSelectedId(mission.id)}>
            {missionDate(mission.date)} · #{count(mission.sequence)}<br/>{mission.name || mission.id}
          </button></td><td>{mission.site || '—'} · {intentLabel(mission.intent)}</td>
          <td>{metres(mission.autonomous_m)}</td><td>{metres(mission.manual_m)}</td><td>{metres(mission.unknown_m)}</td>
          <td>{metres(mission.total_m)}</td><td>{resultLabel(mission.result)} · {count(mission.manual_interventions)}회</td>
        </tr>)}</tbody>
      </table></div>
      {!filtered.length && <p>선택한 조건에 해당하는 미션 기록이 없습니다.</p>}
      {selected && <MissionDetails mission={selected} />}
      <details className="mission-records-sites"><summary>사이트·요청 유형별 집계와 미션 외 주행</summary>
        {(data.sites || []).filter(accepts).map((item, index) => <div key={`${item.site}-${item.intent}-${index}`}>
          <h5>{item.site || '위치 미수신'} · {intentLabel(item.intent)} · {count(item.completed_count)}/{count(item.mission_count)}건 완료 · 수동 개입 {count(item.manual_interventions)}회</h5>
          <Distances values={item} />
        </div>)}
        <h5>미션 외 주행</h5><Distances values={data.outside_missions} />
        <p>미션 외 주행도 위 새 기록기 누적에 포함됩니다. 다시 더하지 않습니다.</p>
      </details>
    </>}
  </div>;
}

export default function MissionRecords() {
  const [open, setOpen] = useState(false);
  return <section className="mission-records" aria-label="왕복 미션 · 자율/수동 기록">
    <div className="mission-records-heading"><div><h3>왕복 미션 · 자율/수동 기록</h3>
      <p>새 기록기 시작 이후의 상세 기록입니다. 기존 누적과 별도 상세기록이며 서로 더하지 않습니다.</p>
    </div><button type="button" aria-expanded={open} onClick={() => setOpen(value => !value)}>
      {open ? '왕복 미션 기록 닫기' : '왕복 미션 기록 보기'}
    </button></div>
    {/* HH_260915 - Lazy mounting preserves legacy KPI polling and read-only scope. */}
    {open && <MissionRecordsBody />}
  </section>;
}

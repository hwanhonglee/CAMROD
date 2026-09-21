import React, { useEffect, useMemo, useState } from 'react';
import './ServiceEvidence.css';
import MissionRecords from './MissionRecords';

const SUMMARY_ENDPOINT = '/api/service-metrics/summary';
const HISTORY_ENDPOINT = '/api/service-metrics?days=30';
const SEOUL_TIME_ZONE = 'Asia/Seoul';

const isRecord = value => Boolean(value) && typeof value === 'object' && !Array.isArray(value);

const finiteNumber = value => {
  if (value === null || value === undefined || value === '') return null;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : null;
};

const distanceOf = record => finiteNumber(
  record?.distance_m ?? record?.total_distance_m
);

const completedCountOf = record => finiteNumber(
  record?.completed_service_count ?? record?.service_count ?? record?.count
);

const siteOf = service => (
  service?.site || service?.destination_site || service?.destination || '-'
);

const formatDistance = (distanceM, digits = 3) => {
  const value = finiteNumber(distanceM);
  if (value === null || value < 0) return null;
  // Small real movements must not disappear behind two-decimal kilometre rounding.
  if (Math.abs(value) < 1000) return formatMeters(value);
  return `${(value / 1000).toLocaleString('ko-KR', {
    minimumFractionDigits: digits,
    maximumFractionDigits: digits,
  })} km`;
};

const formatCount = count => {
  const value = finiteNumber(count);
  if (value === null) return null;
  return `${Math.max(0, Math.trunc(value)).toLocaleString('ko-KR')}회`;
};

const formatDuration = durationS => {
  const value = finiteNumber(durationS);
  if (value === null) return '-';
  const totalSeconds = Math.max(0, Math.round(value));
  const hours = Math.floor(totalSeconds / 3600);
  const minutes = Math.floor((totalSeconds % 3600) / 60);
  const seconds = totalSeconds % 60;
  if (hours > 0) return `${hours}시간 ${minutes}분`;
  if (minutes > 0) return `${minutes}분 ${seconds}초`;
  return `${seconds}초`;
};

const formatMeters = distanceM => {
  const value = finiteNumber(distanceM);
  if (value === null) return '-';
  return `${value.toLocaleString('ko-KR', {
    minimumFractionDigits: value < 10 ? 1 : 0,
    maximumFractionDigits: 1,
  })} m`;
};

const DISTANCE_KINDS = [
  ['delivery', '배송(가는 길)'], ['recall', '호출(가는 길)'],
  ['return', '복귀'], ['unknown', '구간 미확인'],
];

// Never infer old trip categories from a site name, source string, or final state.
// Unallocated metres remain unknown; inconsistent breakdowns cannot invent distance.
export function serviceDistanceBreakdown(record) {
  const total = distanceOf(record);
  if (total === null || total < 0) return null;
  const fallback = { delivery: 0, recall: 0, return: 0, unknown: total };
  if (!isRecord(record?.distance_breakdown_m)) return fallback;
  const values = {};
  for (const [kind] of DISTANCE_KINDS) {
    const value = finiteNumber(record.distance_breakdown_m[kind]);
    if (value !== null && value < 0) return fallback;
    values[kind] = value === null ? 0 : value;
  }
  const known = values.delivery + values.recall + values.return;
  const arithmeticTolerance = Math.max(1, total) * Number.EPSILON * 8;
  if (known > total + arithmeticTolerance || known + values.unknown > total + 0.05) return fallback;
  return { ...values, unknown: Math.max(0, total - known) };
}

function DistanceBreakdown({ record }) {
  const values = serviceDistanceBreakdown(record);
  return (
    <span className="evidence-distance-breakdown" role="group" aria-label="구간별 서비스 이동 거리">
      {DISTANCE_KINDS.map(([kind, label]) => (
        <span className={`evidence-distance-kind ${kind}`} key={kind}>
          <span>{label}</span>
          <strong data-distance-kind={kind}>{values ? formatDistance(values[kind]) : '—'}</strong>
        </span>
      ))}
    </span>
  );
}

function HistoricalRecordsNotice({ history }) {
  const count = finiteNumber(history?.record_count);
  const distance = finiteNumber(history?.distance_m);
  // An unknown transition interval from a new trip is not a legacy record.
  // Show preservation only when explicitly confirmed by the backend; never
  // infer it from source/last state or add these metres to the total again.
  if (!(count > 0) || distance === null || distance < 0
      || history?.included_in_lifetime_total !== true) return null;
  return (
    <aside className="evidence-historical-records" aria-label="기존 운행 기록 보존 안내">
      <strong>기존 기록 {Math.trunc(count).toLocaleString('ko-KR')}건의 구간 미분류 거리 {formatDistance(distance)}가 전체 누적에 포함되어 있습니다.</strong>
      <p>삭제되거나 호출 거리로 바뀐 것이 아닙니다. 과거에 일반 이동·호출·복귀별 거리를 저장하지 않은 부분만 ‘구간 미확인’으로 유지합니다. 구간별 원시 기록이 있어야 정확히 나눌 수 있으며, 합계에 다시 더하지 않습니다.</p>
    </aside>
  );
}

const intentLabel = service => ({
  delivery: '배송', recall: '호출', return: '복귀', unknown: '유형 미확인',
}[service?.intent] || '유형 미확인');

const phaseLabel = service => {
  const phase = service?.phase || service?.state_name;
  return ({
    ACCEPTED: '요청 수락', MOVING_TO_SITE: '사이트로 이동',
    ROAD_HANDOFF_READY: '출차 완료', GUEST_LOADING_WAIT: '이용객 적재 대기',
    WAITING_FOR_RETURN_REQUEST: '이용 완료 대기', RETURN_WITH_CARGO: '짐과 함께 복귀',
    RETURNING_TO_DROP_ZONE: '대기 장소로 복귀', DROP_ZONE_PARKING: '주차 중',
    DROP_ZONE_WAIT: '대기 장소', CHARGING: '충전 중',
  }[phase] || phase || '단계 미수신');
};

const formatPercentage = value => {
  const parsed = finiteNumber(value);
  return parsed === null ? '-' : `${parsed.toFixed(1)}%`;
};

const scaledBarWidth = (value, maximum) => {
  const parsed = finiteNumber(value);
  if (parsed === null || !finiteNumber(maximum) || maximum <= 0) return '0%';
  return `${Math.max(0, Math.min(100, parsed * 100 / maximum)).toFixed(1)}%`;
};

function buildTrendSeries(sites, valueKey, maximum, geometry) {
  const { left, top, plotWidth, plotHeight } = geometry;
  const segments = [];
  const points = [];
  let currentSegment = [];
  sites.forEach((site, index) => {
    const value = finiteNumber(site[valueKey]);
    if (value === null || maximum <= 0) {
      if (currentSegment.length) segments.push(currentSegment);
      currentSegment = [];
      return;
    }
    const ratio = sites.length <= 1 ? 0.5 : index / (sites.length - 1);
    const point = {
      site: site.site,
      value,
      x: left + ratio * plotWidth,
      y: top + (1 - Math.max(0, Math.min(1, value / maximum))) * plotHeight,
    };
    points.push(point);
    currentSegment.push(point);
  });
  if (currentSegment.length) segments.push(currentSegment);
  return { segments, points };
}

// HH_260904 - Reuse the existing B1-B13 aggregates for an SVG trend layer.
// Missing-site samples break the line instead of implying measured values, and
// distance/time retain independent scales because their units are unrelated.
function SiteTrendChart({ sites, maximumDistance, maximumDuration }) {
  const width = 1040;
  const height = 230;
  const geometry = { left: 52, top: 20, plotWidth: 936, plotHeight: 168 };
  const distance = buildTrendSeries(
    sites, 'average_distance_m', maximumDistance, geometry,
  );
  const duration = buildTrendSeries(
    sites, 'average_duration_s', maximumDuration, geometry,
  );
  const xFor = index => geometry.left + (
    sites.length <= 1 ? 0.5 : index / (sites.length - 1)
  ) * geometry.plotWidth;

  return (
    <div className="evidence-site-trend-block">
      <div className="evidence-site-trend-scale">
        <span className="distance">거리 최대 <b>{formatMeters(maximumDistance)}</b></span>
        <span className="duration">시간 최대 <b>{formatDuration(maximumDuration)}</b></span>
      </div>
      <div className="evidence-site-trend-scroll">
        <svg
          className="evidence-site-trend"
          viewBox={`0 0 ${width} ${height}`}
          role="img"
          aria-label="B1부터 B13까지 완료 평균 거리와 시간 꺾은선 그래프"
        >
          <title>B1-B13 완료 평균 추세</title>
          <desc>거리와 시간은 각 항목의 최댓값을 기준으로 독립 정규화됩니다.</desc>
          {[0, 0.25, 0.5, 0.75, 1].map(ratio => {
            const y = geometry.top + ratio * geometry.plotHeight;
            return (
              <g key={ratio} className="evidence-site-trend-grid">
                <line x1={geometry.left} y1={y} x2={geometry.left + geometry.plotWidth} y2={y} />
                <text x={geometry.left - 9} y={y + 4} textAnchor="end">{Math.round((1 - ratio) * 100)}%</text>
              </g>
            );
          })}
          {sites.map((site, index) => (
            <text
              key={site.site}
              className="evidence-site-trend-site"
              x={xFor(index)}
              y={height - 12}
              textAnchor="middle"
            >
              {site.site}
            </text>
          ))}
          {distance.segments.map((segment, index) => (
            <polyline
              key={`distance-${index}`}
              className="evidence-site-trend-line distance"
              points={segment.map(point => `${point.x},${point.y}`).join(' ')}
            />
          ))}
          {duration.segments.map((segment, index) => (
            <polyline
              key={`duration-${index}`}
              className="evidence-site-trend-line duration"
              points={segment.map(point => `${point.x},${point.y}`).join(' ')}
            />
          ))}
          {distance.points.map(point => (
            <circle key={`distance-${point.site}`} className="evidence-site-trend-point distance" cx={point.x} cy={point.y} r="4">
              <title>{point.site} 평균 거리 {formatMeters(point.value)}</title>
            </circle>
          ))}
          {duration.points.map(point => (
            <circle key={`duration-${point.site}`} className="evidence-site-trend-point duration" cx={point.x} cy={point.y} r="4">
              <title>{point.site} 평균 시간 {formatDuration(point.value)}</title>
            </circle>
          ))}
        </svg>
      </div>
    </div>
  );
}

const formatDate = value => {
  if (!value) return '-';
  const text = String(value);
  const dateOnly = /^(\d{4})-(\d{2})-(\d{2})$/.exec(text);
  if (dateOnly) return `${dateOnly[1]}. ${dateOnly[2]}. ${dateOnly[3]}.`;
  const parsed = new Date(text);
  if (Number.isNaN(parsed.getTime())) return text;
  return new Intl.DateTimeFormat('ko-KR', {
    timeZone: SEOUL_TIME_ZONE,
    year: 'numeric', month: '2-digit', day: '2-digit',
  }).format(parsed);
};

const formatDateTime = value => {
  if (!value) return '-';
  const parsed = new Date(value);
  if (Number.isNaN(parsed.getTime())) return String(value);
  return new Intl.DateTimeFormat('ko-KR', {
    timeZone: SEOUL_TIME_ZONE,
    year: 'numeric', month: '2-digit', day: '2-digit',
    hour: '2-digit', minute: '2-digit',
  }).format(parsed);
};

const serviceStatusLabel = service => {
  const raw = String(service?.result || service?.status || '').toUpperCase();
  const labels = {
    ACTIVE: '운행 중', RUNNING: '운행 중', IN_PROGRESS: '운행 중',
    COMPLETED: '완료', SUCCESS: '완료', SUCCEEDED: '완료',
    CANCELLED: '취소', CANCELED: '취소', STOPPED: '중지',
    INTERRUPTED: '중단', SUPERSEDED: '교체 종료', FAILED: '실패',
  };
  return labels[raw] || service?.result || service?.status || '-';
};

const unavailableValue = (loading, error, emptyText = '기록 없음') => {
  if (loading) return '불러오는 중';
  if (error) return '확인 불가';
  return emptyText;
};

const lifetimeValue = lifetime => {
  const distance = formatDistance(distanceOf(lifetime));
  const count = formatCount(completedCountOf(lifetime));
  return [distance, count].filter(Boolean).join(' · ') || null;
};

const persistenceLabel = persistence => {
  if (typeof persistence === 'string' && persistence.trim()) return persistence;
  if (typeof persistence === 'boolean') {
    return persistence ? '운행 기록 저장 중' : '영구 저장 비활성';
  }
  if (!isRecord(persistence)) return '저장 상태 미수신';
  if (persistence.error) return '저장 장애 · 메모리 집계 중';
  if (persistence.label) return String(persistence.label);
  if (persistence.status) return String(persistence.status);
  if (persistence.enabled === false || persistence.durable === false) {
    return '영구 저장 비활성';
  }
  if (persistence.enabled === true || persistence.durable === true) {
    const backend = persistence.backend || persistence.kind || persistence.format;
    return backend ? `운행 기록 저장 중 · ${backend}` : '운행 기록 저장 중';
  }
  return '저장 상태 확인됨';
};

async function readJson(response) {
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  const body = await response.json();
  if (!isRecord(body)) throw new Error('invalid service metrics response');
  return body;
}

function useServiceMetricsPolling(endpoint, refreshMs, requestKey = 0) {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');

  useEffect(() => {
    let mounted = true;
    let timer = null;
    let controller = null;
    let requestTimeout = null;
    setLoading(true);

    const load = async () => {
      controller = new AbortController();
      let timedOut = false;
      requestTimeout = setTimeout(() => {
        timedOut = true;
        controller.abort();
      }, 8000);
      try {
        const response = await fetch(endpoint, {
          cache: 'no-store',
          signal: controller.signal,
        });
        const body = await readJson(response);
        if (!mounted) return;
        if (timedOut) throw new Error('service metrics timeout');
        setData(body);
        setError('');
      } catch (requestError) {
        if (!mounted) return;
        setError(timedOut ? '실증 운행 집계 응답 시간 초과'
          : (requestError.message || '실증 운행 집계를 불러오지 못했습니다.'));
      } finally {
        clearTimeout(requestTimeout);
        if (mounted) {
          setLoading(false);
          timer = setTimeout(load, refreshMs);
        }
      }
    };

    load();
    return () => {
      mounted = false;
      if (timer) clearTimeout(timer);
      if (requestTimeout) clearTimeout(requestTimeout);
      if (controller) controller.abort();
    };
  }, [endpoint, refreshMs, requestKey]);

  return { data, loading, error };
}

export function useServiceMetricsSummary(refreshMs = 3000) {
  return useServiceMetricsPolling(SUMMARY_ENDPOINT, refreshMs);
}

function EvidenceKpi({ label, value, tone = '', record, showTotalKm = false }) {
  const metres = distanceOf(record);
  const totalKm = metres !== null && metres >= 0
    ? `${(metres / 1000).toLocaleString('ko-KR', {
      minimumFractionDigits: 3, maximumFractionDigits: 3,
    })} km` : '—';
  return (
    <span className={`evidence-kpi ${tone ? `evidence-kpi-${tone}` : ''}`}>
      <span className="evidence-kpi-label">{label}</span>
      <strong className="evidence-kpi-value">{value}</strong>
      {showTotalKm && <small className="evidence-kpi-label">합산 {totalKm}</small>}
      {record !== undefined && <DistanceBreakdown record={record} />}
    </span>
  );
}

export function ServiceEvidenceSummary({ data, loading, error, onOpen }) {
  const current = isRecord(data?.current_service) ? data.current_service : null;
  const recent = current || (isRecord(data?.last_completed_service)
    ? data.last_completed_service : null);
  const tripDistance = formatDistance(distanceOf(recent));
  const todayDistance = formatDistance(distanceOf(data?.today));
  const todayCount = formatCount(completedCountOf(data?.today));
  const lifetime = lifetimeValue(data?.lifetime);
  const fallback = unavailableValue(loading, error);
  const stale = Boolean(error && data);

  return (
    <button
      type="button"
      className={`evidence-summary-strip ${stale ? 'evidence-summary-stale' : ''}`}
      onClick={onOpen}
      aria-label="실증 운행 현황 상세 보기"
    >
      <span className="evidence-summary-heading">
        <strong>실증 운행 현황</strong>
        <small>
          {stale
            ? '최근 집계 표시 중 · 갱신 지연'
            : (data?.generated_at ? `${formatDateTime(data.generated_at)} 기준` : fallback)}
        </small>
        <span className="evidence-summary-more">상세 보기 ›</span>
      </span>
      <EvidenceKpi
        label={current ? '이번 서비스' : '최근 서비스'}
        value={tripDistance || fallback}
        tone={current ? 'live' : ''}
        record={recent}
      />
      <EvidenceKpi label="오늘 누적" value={todayDistance || fallback} record={data?.today || null} showTotalKm />
      <EvidenceKpi label="오늘 완료" value={todayCount || fallback} />
      <EvidenceKpi label="전체 누적" value={lifetime || fallback} record={data?.lifetime || null} showTotalKm />
    </button>
  );
}

export function ServiceTripBadge({ serviceActive, currentService, loading, error, onOpen }) {
  if (!serviceActive) return null;
  const distance = formatDistance(distanceOf(currentService));
  let value = distance;
  if (!value) {
    if (loading) value = '거리 집계 확인 중';
    else if (error) value = '거리 집계 확인 불가';
    else value = '거리 집계 시작 대기';
  }
  return (
    <button
      type="button"
      className={`evidence-trip-badge ${error ? 'evidence-trip-badge-stale' : ''}`}
      onClick={onOpen}
      aria-label="실증 운행 현황 상세 보기"
    >
      <span className="evidence-trip-live-dot" />
      <span>이번 서비스</span>
      <strong>{value}</strong>
      {distance && error && <small>갱신 지연</small>}
      <span className="evidence-trip-more">상세 보기 ›</span>
    </button>
  );
}

function ServiceOverview({ service, active }) {
  if (!isRecord(service)) {
    return (
      <div className="evidence-empty evidence-service-empty">
        아직 표시할 서비스 운행 기록이 없습니다.
      </div>
    );
  }
  const start = service.started_at || service.start_time;
  const end = service.completed_at || service.ended_at || service.end_time;
  const distance = formatDistance(distanceOf(service), 3) || '집계 중';
  return (
    <div className="evidence-service-overview">
      <div className="evidence-service-identity">
        <span className={`evidence-status-pill ${active ? 'active' : ''}`}>
          {active ? '운행 중' : serviceStatusLabel(service)}
        </span>
        <strong>{siteOf(service)}</strong>
        <span className="evidence-service-phase">{intentLabel(service)} · {phaseLabel(service)}</span>
        <small>{service.id || service.service_id || '서비스 식별자 미수신'}</small>
      </div>
      <dl className="evidence-service-facts">
        <div><dt>이동 거리</dt><dd>{distance}</dd></div>
        <div><dt>서비스 경과시간(대기 포함)</dt><dd>{formatDuration(service.duration_s)}</dd></div>
        <div><dt>시작</dt><dd>{formatDateTime(start)}</dd></div>
        <div><dt>{active ? '현재 상태' : '완료'}</dt><dd>{active ? serviceStatusLabel(service) : formatDateTime(end)}</dd></div>
      </dl>
      <DistanceBreakdown record={service} />
    </div>
  );
}

function MetricsNotice({ loading, error, hasData, onRetry }) {
  if (loading && !hasData) {
    return <div className="evidence-notice evidence-notice-loading">실증 운행 기록을 불러오는 중입니다.</div>;
  }
  if (!error) return null;
  return (
    <div className="evidence-notice evidence-notice-error" role="alert">
      <span>{hasData ? '최근 저장된 집계를 표시합니다. 최신 기록 갱신에 실패했습니다.' : '실증 운행 기록을 불러오지 못했습니다.'}</span>
      <button type="button" onClick={onRetry}>다시 시도</button>
    </div>
  );
}

// HH_260904 - Compare every campsite in one bounded render using the backend's
// completed-run aggregates; active percentages are not route progress.
function SitePerformance({ sites, loading, error }) {
  const maximumDistance = Math.max(
    0,
    ...sites.map(site => finiteNumber(site.average_distance_m) || 0),
  );
  const maximumDuration = Math.max(
    0,
    ...sites.map(site => finiteNumber(site.average_duration_s) || 0),
  );

  if (loading && sites.length === 0) {
    return <div className="evidence-empty">사이트별 운행 지표를 불러오는 중입니다.</div>;
  }
  if (error && sites.length === 0) {
    return <div className="evidence-empty evidence-empty-error">사이트별 운행 지표를 확인할 수 없습니다.</div>;
  }

  return (
    <>
      <div className="evidence-site-chart" aria-label="B1부터 B13까지 평균 운행 거리와 시간 그래프">
        <div className="evidence-site-chart-legend">
          <span><i className="distance" />완료 평균 거리</span>
          <span><i className="duration" />완료 평균 시간</span>
          <em>항목별 독립 척도</em>
        </div>
        <SiteTrendChart
          sites={sites}
          maximumDistance={maximumDistance}
          maximumDuration={maximumDuration}
        />
        {sites.map(site => {
          const current = isRecord(site.current_service) ? site.current_service : null;
          const completedCount = finiteNumber(site.completed_service_count);
          return (
            <div className={`evidence-site-chart-row ${current ? 'active' : ''}`} key={site.site}>
              <strong>{site.site}</strong>
              <div className="evidence-site-bars">
                <div className="evidence-site-bar-line">
                  <span className="evidence-site-bar-track">
                    <i className="distance" style={{ width: scaledBarWidth(site.average_distance_m, maximumDistance) }} />
                  </span>
                  <b>{formatMeters(site.average_distance_m)}</b>
                </div>
                <div className="evidence-site-bar-line">
                  <span className="evidence-site-bar-track">
                    <i className="duration" style={{ width: scaledBarWidth(site.average_duration_s, maximumDuration) }} />
                  </span>
                  <b>{formatDuration(site.average_duration_s)}</b>
                </div>
              </div>
              <span className="evidence-site-run-state">
                {current
                  ? `운행 중 · ${formatMeters(current.distance_m)} · ${formatDuration(current.duration_s)}`
                  : (completedCount === null ? '기록 없음' : `${Math.trunc(completedCount)}회 완료`)}
              </span>
            </div>
          );
        })}
      </div>
      <div className="evidence-table-scroll evidence-site-table-scroll">
        <table className="evidence-table evidence-site-table">
          <caption className="sr-only">사이트별 평균, 최근 실행, 현재 실행 정량 지표</caption>
          <thead>
            <tr>
              <th>사이트</th><th>완료/시도</th><th>완료율</th>
              <th>평균 거리</th><th>평균 시간</th><th>최근 실행</th><th>현재 진행</th>
              <th>누적 구간거리(진행·중단 포함)</th>
            </tr>
          </thead>
          <tbody>
            {sites.map(site => {
              const latest = isRecord(site.latest_service) ? site.latest_service : null;
              const current = isRecord(site.current_service) ? site.current_service : null;
              const completedCount = finiteNumber(site.completed_service_count);
              const attemptCount = finiteNumber(site.service_attempt_count);
              return (
                <tr key={site.site} className={current ? 'evidence-site-active-row' : ''}>
                  <td><strong>{site.site}</strong></td>
                  <td>{completedCount === null || attemptCount === null
                    ? '-' : `${Math.trunc(completedCount)}/${Math.trunc(attemptCount)}`}</td>
                  <td>{formatPercentage(site.completion_rate_percentage)}</td>
                  <td>{formatMeters(site.average_distance_m)}</td>
                  <td>{formatDuration(site.average_duration_s)}</td>
                  <td>{latest
                    ? `${formatMeters(latest.distance_m)} · ${formatDuration(latest.duration_s)}`
                    : '-'}</td>
                  <td>{current
                    ? (
                      <span className="evidence-current-progress">
                        <b>{formatMeters(current.distance_m)} · {formatDuration(current.duration_s)}</b>
                        <small>
                          완료 평균 대비 거리 {formatPercentage(site.current_distance_progress_percentage)}
                          {' · '}시간 {formatPercentage(site.current_duration_progress_percentage)}
                        </small>
                      </span>
                    ) : '-'}</td>
                  <td><DistanceBreakdown record={site} /></td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </div>
    </>
  );
}

export function ServiceEvidenceDashboard({ summaryData, summaryLoading, summaryError }) {
  const [requestKey, setRequestKey] = useState(0);
  // Only mounted while the detail dialog is open. One in-flight request per
  // effect; refresh/unmount aborts it and invalidates any late response.
  const { data: detailData, loading: detailLoading, error: detailError } =
    useServiceMetricsPolling(HISTORY_ENDPOINT, 4000, requestKey);

  // HH_260819 - History is a bounded modal payload, while the always-mounted
  // summary hook keeps active cards live every 3 s; history refreshes every 4 s.
  // A null current_service in the summary is authoritative after completion,
  // so spread it rather than using nullish fallbacks.
  const data = useMemo(() => {
    if (!detailData) return summaryData;
    if (!summaryData) return detailData;
    return {
      ...detailData,
      ...summaryData,
      daily_history: detailData.daily_history,
      recent_services: detailData.recent_services,
    };
  }, [detailData, summaryData]);
  const current = isRecord(data?.current_service) ? data.current_service : null;
  const last = isRecord(data?.last_completed_service) ? data.last_completed_service : null;
  const todayDistance = formatDistance(distanceOf(data?.today));
  const todayCount = formatCount(completedCountOf(data?.today));
  const lifetimeDistance = formatDistance(distanceOf(data?.lifetime));
  const lifetimeCount = formatCount(completedCountOf(data?.lifetime));
  const combinedLoading = detailLoading && !data;
  // Keep stale live-summary failures visible even when a cached history
  // response is still available; cached evidence must not look freshly updated.
  const combinedError = detailError || summaryError;
  const fallback = unavailableValue(combinedLoading || summaryLoading, combinedError);
  const history = useMemo(() => (
    Array.isArray(detailData?.daily_history)
      ? [...detailData.daily_history].sort((a, b) => String(b.date || '').localeCompare(String(a.date || '')))
      : []
  ), [detailData]);
  const recentServices = useMemo(() => (
    Array.isArray(detailData?.recent_services) ? detailData.recent_services : []
  ), [detailData]);
  const siteSummaries = useMemo(() => (
    Array.isArray(data?.site_summaries) ? data.site_summaries : []
  ), [data]);

  return (
    <div className="service-evidence-dashboard">
      <div className="evidence-dashboard-head">
        <div>
          <span className="evidence-summary-eyebrow">CAMROD FIELD OPERATION</span>
          <h2>실증 운행 누적 현황</h2>
          <p>서비스 중 실측 이동거리입니다. 순수 자율주행 누적거리와는 다르며, 경과시간에는 대기가 포함됩니다.</p>
          <p>누적 거리는 진행·중단된 서비스도 포함합니다. 과거 기록의 알 수 없는 구간은 구간 미확인으로 표시합니다.</p>
        </div>
        <div className="evidence-data-state">
          <strong>{persistenceLabel(data?.persistence)}</strong>
          <span>{data?.generated_at ? `${formatDateTime(data.generated_at)} 요약 갱신` : '요약 갱신 시각 미수신'}</span>
          <span>{detailData?.generated_at ? `${formatDateTime(detailData.generated_at)} 목록 갱신` : '목록 갱신 시각 미수신'} · 4초 자동 갱신</span>
          <span>집계 기준 시간대 · Asia/Seoul</span>
          <button className="evidence-refresh-button" type="button" onClick={() => setRequestKey(key => key + 1)}>
            {detailLoading ? '다시 새로고침' : '새로고침'}
          </button>
        </div>
      </div>

      <MetricsNotice
        loading={combinedLoading}
        error={combinedError}
        hasData={Boolean(data)}
        onRetry={() => setRequestKey(key => key + 1)}
      />

      <HistoricalRecordsNotice history={data?.historical_unclassified} />

      <section className="evidence-dashboard-kpis" aria-label="실증 운행 핵심 지표">
        <EvidenceKpi label="오늘 이동 거리" value={todayDistance || fallback} record={data?.today || null} showTotalKm />
        <EvidenceKpi label="오늘 완료 서비스" value={todayCount || fallback} />
        <EvidenceKpi label="전체 이동 거리" value={lifetimeDistance || fallback} record={data?.lifetime || null} showTotalKm />
        <EvidenceKpi label="전체 완료 서비스" value={lifetimeCount || fallback} />
      </section>

      <MissionRecords />

      <section className="evidence-panel evidence-current-panel">
        <div className="evidence-panel-heading">
          <div>
            <span>{current ? 'LIVE SERVICE' : 'LATEST SERVICE'}</span>
            <h3>{current ? '현재 서비스 운행' : '최근 완료 서비스'}</h3>
          </div>
          {current && <em>거리 실시간 집계 중</em>}
        </div>
        <ServiceOverview service={current || last} active={Boolean(current)} />
      </section>

      <section className="evidence-panel evidence-site-performance-panel">
        <div className="evidence-panel-heading">
          <div><span>SITE PERFORMANCE</span><h3>B1-B13 서비스 비교</h3></div>
          <em>현재 진행률은 완료 평균 대비 값</em>
        </div>
        <SitePerformance
          sites={siteSummaries}
          loading={combinedLoading || summaryLoading}
          error={combinedError}
        />
      </section>

      <div className="evidence-history-layout">
        <section className="evidence-panel">
          <div className="evidence-panel-heading">
            <div><span>DAILY HISTORY</span><h3>날짜별 운행 실적</h3></div>
            <em>서비스 시작일 기준 · 최근 30일</em>
          </div>
          <div className="evidence-table-scroll">
            <table className="evidence-table">
              <caption className="sr-only">날짜별 운행 거리와 완료 서비스 수</caption>
              <thead><tr><th>날짜</th><th>완료 서비스</th><th>이동 거리</th><th>구간별 거리</th></tr></thead>
              <tbody>
                {history.map((day, index) => (
                  <tr key={day.date || index}>
                    <td>{formatDate(day.date)}</td>
                    <td>{formatCount(completedCountOf(day)) || '-'}</td>
                    <td>{formatDistance(distanceOf(day), 3) || '-'}</td>
                    <td><DistanceBreakdown record={day} /></td>
                  </tr>
                ))}
              </tbody>
            </table>
            {!detailLoading && !detailError && history.length === 0 && (
              <div className="evidence-empty">최근 30일간 완료된 서비스 기록이 없습니다.</div>
            )}
            {detailLoading && <div className="evidence-empty">날짜별 기록을 불러오는 중입니다.</div>}
            {detailError && <div className="evidence-empty evidence-empty-error">날짜별 기록을 확인할 수 없습니다.</div>}
          </div>
        </section>

        <section className="evidence-panel">
          <div className="evidence-panel-heading">
            <div><span>RECENT SERVICES</span><h3>최근 서비스 상세</h3></div>
          </div>
          <div className="evidence-table-scroll">
            <table className="evidence-table evidence-recent-table">
              <caption className="sr-only">최근 서비스별 목적지와 이동 거리</caption>
              <thead><tr><th>완료 시각</th><th>목적지</th><th>유형·단계</th><th>거리</th><th>경과(대기 포함)</th><th>결과</th><th>구간별 거리</th></tr></thead>
              <tbody>
                {recentServices.map((service, index) => (
                  <tr key={service.id || service.service_id || index}>
                    <td>{formatDateTime(service.completed_at || service.ended_at || service.end_time)}</td>
                    <td><strong>{siteOf(service)}</strong></td>
                    <td>{intentLabel(service)} · {phaseLabel(service)}</td>
                    <td>{formatDistance(distanceOf(service), 3) || '-'}</td>
                    <td>{formatDuration(service.duration_s)}</td>
                    <td><span className="evidence-result">{serviceStatusLabel(service)}</span></td>
                    <td><DistanceBreakdown record={service} /></td>
                  </tr>
                ))}
              </tbody>
            </table>
            {!detailLoading && !detailError && recentServices.length === 0 && (
              <div className="evidence-empty">최근 서비스 상세 기록이 없습니다.</div>
            )}
            {detailLoading && <div className="evidence-empty">최근 서비스 기록을 불러오는 중입니다.</div>}
            {detailError && <div className="evidence-empty evidence-empty-error">최근 서비스 기록을 확인할 수 없습니다.</div>}
          </div>
        </section>
      </div>
    </div>
  );
}

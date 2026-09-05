import React, { useEffect, useMemo, useState } from 'react';

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

const formatDistance = (distanceM, digits = 2) => {
  const value = finiteNumber(distanceM);
  if (value === null) return null;
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

export function useServiceMetricsSummary(refreshMs = 3000) {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');

  useEffect(() => {
    let mounted = true;
    let timer = null;
    let controller = null;

    const load = async () => {
      controller = new AbortController();
      try {
        const response = await fetch(SUMMARY_ENDPOINT, {
          cache: 'no-store',
          signal: controller.signal,
        });
        const body = await readJson(response);
        if (!mounted) return;
        setData(body);
        setError('');
      } catch (requestError) {
        if (!mounted || requestError.name === 'AbortError') return;
        setError(requestError.message || '실증 운행 집계를 불러오지 못했습니다.');
      } finally {
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
      if (controller) controller.abort();
    };
  }, [refreshMs]);

  return { data, loading, error };
}

function EvidenceKpi({ label, value, tone = '' }) {
  return (
    <span className={`evidence-kpi ${tone ? `evidence-kpi-${tone}` : ''}`}>
      <span className="evidence-kpi-label">{label}</span>
      <strong className="evidence-kpi-value">{value}</strong>
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
        <span className="evidence-summary-eyebrow">CAMROD FIELD OPERATION</span>
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
      />
      <EvidenceKpi label="오늘 누적" value={todayDistance || fallback} />
      <EvidenceKpi label="오늘 완료" value={todayCount || fallback} />
      <EvidenceKpi label="전체 누적" value={lifetime || fallback} />
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
        <small>{service.id || service.service_id || '서비스 식별자 미수신'}</small>
      </div>
      <dl className="evidence-service-facts">
        <div><dt>이동 거리</dt><dd>{distance}</dd></div>
        <div><dt>소요 시간</dt><dd>{formatDuration(service.duration_s)}</dd></div>
        <div><dt>시작</dt><dd>{formatDateTime(start)}</dd></div>
        <div><dt>{active ? '현재 상태' : '완료'}</dt><dd>{active ? serviceStatusLabel(service) : formatDateTime(end)}</dd></div>
      </dl>
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
  const [detailData, setDetailData] = useState(null);
  const [detailLoading, setDetailLoading] = useState(true);
  const [detailError, setDetailError] = useState('');
  const [requestKey, setRequestKey] = useState(0);

  useEffect(() => {
    let mounted = true;
    const controller = new AbortController();
    setDetailLoading(true);
    setDetailError('');
    fetch(HISTORY_ENDPOINT, { cache: 'no-store', signal: controller.signal })
      .then(readJson)
      .then(body => {
        if (mounted) setDetailData(body);
      })
      .catch(requestError => {
        if (mounted && requestError.name !== 'AbortError') {
          setDetailError(requestError.message || '실증 운행 기록을 불러오지 못했습니다.');
        }
      })
      .finally(() => { if (mounted) setDetailLoading(false); });
    return () => { mounted = false; controller.abort(); };
  }, [requestKey]);

  // HH_260819 - History is a bounded modal payload, while the always-mounted
  // summary hook keeps the active trip and aggregate cards live every 3 s.
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
  // Keep stale live-summary failures visible even when the one-time history
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
          <p>서비스 한 건의 실제 이동 거리와 날짜별 누적 결과를 확인합니다.</p>
        </div>
        <div className="evidence-data-state">
          <strong>{persistenceLabel(data?.persistence)}</strong>
          <span>{data?.generated_at ? `${formatDateTime(data.generated_at)} 갱신` : '갱신 시각 미수신'}</span>
          <span>집계 기준 시간대 · Asia/Seoul</span>
        </div>
      </div>

      <MetricsNotice
        loading={combinedLoading}
        error={combinedError}
        hasData={Boolean(data)}
        onRetry={() => setRequestKey(key => key + 1)}
      />

      <section className="evidence-dashboard-kpis" aria-label="실증 운행 핵심 지표">
        <EvidenceKpi label="오늘 이동 거리" value={todayDistance || fallback} />
        <EvidenceKpi label="오늘 완료 서비스" value={todayCount || fallback} />
        <EvidenceKpi label="전체 이동 거리" value={lifetimeDistance || fallback} />
        <EvidenceKpi label="전체 완료 서비스" value={lifetimeCount || fallback} />
      </section>

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
            <em>최근 30일</em>
          </div>
          <div className="evidence-table-scroll">
            <table className="evidence-table">
              <caption className="sr-only">날짜별 운행 거리와 완료 서비스 수</caption>
              <thead><tr><th>날짜</th><th>완료 서비스</th><th>이동 거리</th></tr></thead>
              <tbody>
                {history.map((day, index) => (
                  <tr key={day.date || index}>
                    <td>{formatDate(day.date)}</td>
                    <td>{formatCount(completedCountOf(day)) || '-'}</td>
                    <td>{formatDistance(distanceOf(day), 3) || '-'}</td>
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
              <thead><tr><th>완료 시각</th><th>목적지</th><th>거리</th><th>소요</th><th>결과</th></tr></thead>
              <tbody>
                {recentServices.map((service, index) => (
                  <tr key={service.id || service.service_id || index}>
                    <td>{formatDateTime(service.completed_at || service.ended_at || service.end_time)}</td>
                    <td><strong>{siteOf(service)}</strong></td>
                    <td>{formatDistance(distanceOf(service), 3) || '-'}</td>
                    <td>{formatDuration(service.duration_s)}</td>
                    <td><span className="evidence-result">{serviceStatusLabel(service)}</span></td>
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

import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';

const formatTime = value => {
  const seconds = Number(value);
  if (!Number.isFinite(seconds) || seconds <= 0) return '';
  return new Date(seconds * 1000).toLocaleString('ko-KR');
};

const formatBytes = value => {
  const bytes = Number(value);
  if (!Number.isFinite(bytes) || bytes < 0) return '—';
  if (bytes >= 1_000_000_000) return `${(bytes / 1_000_000_000).toFixed(1)}GB`;
  if (bytes >= 1_000_000) return `${Math.round(bytes / 1_000_000)}MB`;
  if (bytes >= 1_000) return `${Math.round(bytes / 1_000)}KB`;
  return `${Math.round(bytes)}B`;
};

async function requestJson(url, options) {
  const response = await fetch(url, options);
  let body = {};
  try {
    body = await response.json();
  } catch (_error) {
    body = { message: '서버 응답을 해석할 수 없습니다.' };
  }
  if (!response.ok || body.success === false) {
    const error = new Error(body.message || `요청 실패 (${response.status})`);
    error.body = body;
    throw error;
  }
  return body;
}

export default function SnapshotControl() {
  const [status, setStatus] = useState({
    available: false,
    recording: false,
    writing: false,
    pending: false,
    active_topics: [],
    dynamic_topics: [],
    available_topics: [],
    last_result: {},
  });
  const [selectedTopics, setSelectedTopics] = useState([]);
  const [topicFilter, setTopicFilter] = useState('');
  const [label, setLabel] = useState('');
  const [outputDirectory, setOutputDirectory] = useState(null);
  const [lookbackSeconds, setLookbackSeconds] = useState('300');
  const [autoFit, setAutoFit] = useState(true);
  const [estimate, setEstimate] = useState(null);
  const [estimatePending, setEstimatePending] = useState(false);
  const [message, setMessage] = useState('Snapshot 서비스 연결 확인 중…');
  const [topicPending, setTopicPending] = useState('');
  const [writePending, setWritePending] = useState(false);
  const [confirmWrite, setConfirmWrite] = useState(false);
  const selectionInitialized = useRef(false);
  const previousActiveTopics = useRef(new Set());

  const refresh = useCallback(async (quiet = false) => {
    try {
      const body = await requestJson('/api/admin/snapshot/status');
      setStatus(body);
      const activeNames = (body.active_topics || []).map(item => item.name);
      const activeSet = new Set(activeNames);
      setSelectedTopics(current => {
        if (!selectionInitialized.current) {
          selectionInitialized.current = true;
          previousActiveTopics.current = activeSet;
          return activeNames.sort();
        }
        const next = new Set(current.filter(name => activeSet.has(name)));
        activeNames.forEach(name => {
          if (!previousActiveTopics.current.has(name)) next.add(name);
        });
        previousActiveTopics.current = activeSet;
        return Array.from(next).sort();
      });
      if (!quiet) {
        setMessage(body.available
          ? (body.recording
            ? ''
            : '메시지 버퍼링이 일시 정지되어 있습니다.')
          : 'Snapshot 서비스에 연결할 수 없습니다.');
      }
    } catch (error) {
      setStatus(current => ({ ...current, available: false }));
      if (!quiet) setMessage(error.message);
    }
  }, []);

  useEffect(() => {
    let mounted = true;
    const run = async quiet => {
      if (mounted) await refresh(quiet);
    };
    run(false);
    const timer = window.setInterval(() => run(true), 3000);
    return () => {
      mounted = false;
      window.clearInterval(timer);
    };
  }, [refresh]);

  const activeTopicNames = useMemo(
    () => new Set((status.active_topics || []).map(item => item.name)),
    [status.active_topics]
  );
  const selectedTopicNames = useMemo(() => new Set(selectedTopics), [selectedTopics]);
  const allTopics = useMemo(() => {
    const topics = new Map();
    (status.available_topics || []).forEach(topic => topics.set(topic.name, topic));
    (status.active_topics || []).forEach(topic => topics.set(topic.name, {
      ...topics.get(topic.name),
      ...topic,
      selectable: true,
    }));
    return Array.from(topics.values()).sort((left, right) => left.name.localeCompare(right.name));
  }, [status.active_topics, status.available_topics]);
  const visibleTopics = useMemo(() => {
    const query = topicFilter.trim().toLocaleLowerCase();
    if (!query) return allTopics;
    return allTopics.filter(topic =>
      `${topic.name} ${topic.type || ''}`.toLocaleLowerCase().includes(query)
    );
  }, [allTopics, topicFilter]);
  const busy = Boolean(writePending || status.pending || status.writing);
  const numericLookback = Number(lookbackSeconds);
  const lookbackValid = Number.isFinite(numericLookback)
    && numericLookback >= 1
    && numericLookback <= 300;

  useEffect(() => {
    if (!status.available || !selectedTopics.length || !lookbackValid || busy) {
      setEstimate(null);
      setEstimatePending(false);
      return undefined;
    }
    let cancelled = false;
    const timer = window.setTimeout(async () => {
      setEstimatePending(true);
      try {
        const body = await requestJson('/api/admin/snapshot/estimate', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            topics: selectedTopics,
            output_directory: outputDirectory ?? status.output_directory ?? '',
            lookback_seconds: numericLookback,
            auto_fit: autoFit,
          }),
        });
        if (!cancelled) setEstimate(body);
      } catch (error) {
        if (!cancelled) setEstimate({
          success: false,
          message: error.message,
          ...(error.body || {}),
        });
      } finally {
        if (!cancelled) setEstimatePending(false);
      }
    }, 350);
    return () => {
      cancelled = true;
      window.clearTimeout(timer);
    };
  }, [
    autoFit,
    busy,
    lookbackValid,
    numericLookback,
    outputDirectory,
    selectedTopics,
    status.available,
    status.output_directory,
  ]);

  const toggleTopic = async topic => {
    if (topicPending || busy || topic.selectable === false) return;
    setConfirmWrite(false);
    if (selectedTopicNames.has(topic.name)) {
      setSelectedTopics(current => current.filter(name => name !== topic.name));
      return;
    }
    if (activeTopicNames.has(topic.name)) {
      setSelectedTopics(current => [...current, topic.name].sort());
      return;
    }

    setTopicPending(topic.name);
    setMessage(`${topic.name} 토픽의 버퍼링을 시작하는 중…`);
    try {
      const body = await requestJson('/api/admin/snapshot/topics', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ topics: [topic.name] }),
      });
      setSelectedTopics(current => Array.from(new Set([...current, topic.name])).sort());
      setMessage(body.message || `${topic.name} 토픽을 버퍼링하고 저장 대상으로 선택했습니다.`);
      await refresh(true);
    } catch (error) {
      const rejected = error.body?.rejected_topics || [];
      setMessage(rejected.length
        ? `버퍼링할 수 없는 토픽: ${rejected.join(', ')}`
        : error.message);
    } finally {
      setTopicPending('');
    }
  };

  const selectAllBufferedTopics = () => {
    setSelectedTopics(Array.from(activeTopicNames).sort());
    setConfirmWrite(false);
  };

  const clearSelectedTopics = () => {
    setSelectedTopics([]);
    setConfirmWrite(false);
  };

  const writeSnapshot = async () => {
    if (!lookbackValid) {
      setMessage('저장 범위는 1초에서 300초 사이로 입력해 주세요.');
      return;
    }
    if (!selectedTopics.length) {
      setMessage('저장할 토픽을 하나 이상 선택해 주세요.');
      return;
    }
    if (!estimate?.success) {
      setMessage(estimate?.message || '저장 예상 용량을 확인하는 중입니다.');
      return;
    }
    if (!autoFit && !estimate.fits_without_truncation) {
      setMessage('현재 저장 범위가 안전 용량을 초과합니다. 용량 자동 맞춤을 켜거나 저장 범위를 줄여 주세요.');
      return;
    }
    if (!confirmWrite) {
      setConfirmWrite(true);
      const range = estimate.truncated
        ? `최신 약 ${Math.max(1, Math.round(estimate.actual_lookback_seconds))}초로 자동 조정하여`
        : `최근 ${numericLookback}초를`;
      setMessage(`선택한 ${selectedTopics.length}개 토픽의 ${range} rosbag으로 저장합니다. 계속하려면 저장 버튼을 한 번 더 눌러 주세요.`);
      return;
    }
    if (busy || !status.available) return;
    setConfirmWrite(false);
    setWritePending(true);
    setMessage('rosbag 스냅샷을 저장하는 중…');
    try {
      const body = await requestJson('/api/admin/snapshot', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          label,
          topics: selectedTopics,
          output_directory: outputDirectory ?? status.output_directory ?? '',
          lookback_seconds: numericLookback,
          auto_fit: autoFit,
        }),
      });
      setLabel('');
      setMessage(body.message || `저장 완료: ${body.path}`);
      setStatus(current => ({ ...current, last_result: body }));
    } catch (error) {
      setMessage(error.message);
    } finally {
      setWritePending(false);
      await refresh(true);
    }
  };

  const lastResult = status.last_result || {};
  return (
    <section className="snapshot-control-card" aria-label="ROS Snapshot">
      <div className="snapshot-control-heading">
        <div>
          <div className="snapshot-control-title">ROSBAG Snapshot</div>
          <div className="snapshot-control-help">
            버퍼링 중인 ROS 토픽의 최근 데이터를 하나의 rosbag으로 저장합니다.
          </div>
        </div>
        <span className={`snapshot-state ${status.available ? (busy ? 'busy' : 'ready') : 'offline'}`}>
          {status.available
            ? (busy ? '저장 중' : (status.recording ? '버퍼링 중' : '일시 정지'))
            : '연결 안 됨'}
        </span>
      </div>

      <div className="snapshot-summary-row">
        <span>버퍼링 토픽 <strong>{(status.active_topics || []).length}</strong>개</span>
        <span>저장 선택 <strong>{selectedTopics.length}</strong>개</span>
        <span>발견 토픽 <strong>{allTopics.length}</strong>개</span>
        {Number.isFinite(status.free_space_mb) && (
          <span>기본 경로 여유 <strong>{Math.round(status.free_space_mb / 1024)}</strong>GB</span>
        )}
      </div>

      <div className="snapshot-topic-picker">
        <div className="snapshot-topic-picker-heading">
          <div>
            <div className="snapshot-field-label">저장할 ROS 토픽</div>
            <small>기존 버퍼링 토픽은 처음부터 선택되어 있습니다. 미버퍼링 토픽을 켜면 그 시점부터 버퍼링합니다.</small>
          </div>
          <div className="snapshot-topic-picker-actions">
            <button type="button" onClick={selectAllBufferedTopics} disabled={busy || Boolean(topicPending)}>
              버퍼링 토픽 전체 선택
            </button>
            <button type="button" onClick={clearSelectedTopics} disabled={busy || Boolean(topicPending) || !selectedTopics.length}>
              모두 해제
            </button>
          </div>
        </div>
        <input
          className="snapshot-topic-filter"
          type="search"
          value={topicFilter}
          onChange={event => setTopicFilter(event.target.value)}
          placeholder="토픽 이름 또는 메시지 타입 검색"
          aria-label="토픽 검색"
          disabled={busy}
        />
        <div className="snapshot-topic-list" aria-label="ROS 토픽 선택 목록">
          {visibleTopics.map(topic => {
            const active = activeTopicNames.has(topic.name);
            const selected = selectedTopicNames.has(topic.name);
            const pending = topicPending === topic.name;
            return (
              <div className={`snapshot-topic-row${selected ? ' selected' : ''}`} key={topic.name}>
                <button
                  type="button"
                  className="snapshot-topic-switch"
                  role="switch"
                  aria-checked={selected}
                  aria-label={`${topic.name} 저장 ${selected ? '해제' : '선택'}`}
                  onClick={() => toggleTopic(topic)}
                  disabled={!status.available || busy || Boolean(topicPending) || topic.selectable === false}
                >
                  <span className="snapshot-topic-switch-knob" />
                </button>
                <div className="snapshot-topic-identity">
                  <code>{topic.name}</code>
                  <small>{topic.type || '메시지 타입 확인 중'}</small>
                </div>
                <span className={`snapshot-topic-buffer-state ${active ? 'active' : 'inactive'}`}>
                  {pending ? '적용 중' : (active ? '버퍼링' : (topic.selectable === false ? '타입 충돌' : '미버퍼링'))}
                </span>
              </div>
            );
          })}
          {!visibleTopics.length && (
            <div className="snapshot-topic-empty">검색 조건에 맞는 토픽이 없습니다.</div>
          )}
        </div>
      </div>

      <div className="snapshot-directory-row">
        <label htmlFor="snapshot-directory-input">저장 폴더</label>
        <input
          id="snapshot-directory-input"
          type="text"
          value={outputDirectory ?? status.output_directory ?? ''}
          maxLength={1024}
          onChange={event => setOutputDirectory(event.target.value)}
          placeholder="예: /home/avg/Data/snapshots"
          disabled={busy}
          spellCheck={false}
        />
        <button
          type="button"
          className="reset-directory"
          onClick={() => setOutputDirectory(null)}
          disabled={busy || outputDirectory === null}
          title="설정된 기본 저장 폴더로 되돌리기"
        >
          기본값 복원
        </button>
        <small>로봇(서버) 기준 경로입니다. 절대 경로 또는 ~로 시작하는 경로를 입력하세요.</small>
      </div>

      <div className="snapshot-lookback-row">
        <label htmlFor="snapshot-lookback-input">저장 범위</label>
        <span className="snapshot-lookback-input">
          최근
          <input
            id="snapshot-lookback-input"
            type="number"
            min="1"
            max="300"
            step="1"
            value={lookbackSeconds}
            onChange={event => {
              setLookbackSeconds(event.target.value);
              setConfirmWrite(false);
            }}
            disabled={busy}
          />
          초
        </span>
        <small>최대 300초이며, 토픽별로 현재 버퍼에 남아 있는 범위 안에서 저장됩니다.</small>
      </div>

      <div className={`snapshot-capacity-card${estimate?.truncated ? ' adjusted' : ''}`}>
        <div className="snapshot-capacity-heading">
          <div>
            <strong>SSD 용량 보호</strong>
            <small>여유 공간의 10% 또는 5GB 이상을 남기고 예상 크기에 30% 안전 여유를 적용합니다.</small>
          </div>
          <label className="snapshot-auto-fit-toggle">
            <input
              type="checkbox"
              checked={autoFit}
              onChange={event => {
                setAutoFit(event.target.checked);
                setConfirmWrite(false);
              }}
              disabled={busy}
            />
            용량 자동 맞춤
          </label>
        </div>
        {estimatePending ? (
          <div className="snapshot-capacity-loading">저장 예상 용량 계산 중…</div>
        ) : estimate?.success ? (
          <>
            <div className="snapshot-capacity-metrics">
              <span>현재 여유 <strong>{formatBytes(estimate.free_bytes)}</strong></span>
              <span>안전 보존 <strong>{formatBytes(estimate.reserve_bytes)}</strong></span>
              <span>예상 저장 <strong>{formatBytes(estimate.selected_disk_bytes)}</strong></span>
              <span>저장 후 예상 <strong>{formatBytes(estimate.projected_free_bytes)}</strong></span>
            </div>
            {estimate.truncated && (
              <div className="snapshot-capacity-warning">
                요청한 최근 {numericLookback}초 중 최신 약 {Math.max(1, Math.round(estimate.actual_lookback_seconds))}초만 저장하도록 자동 조정됩니다.
              </div>
            )}
            {!autoFit && !estimate.fits_without_truncation && (
              <div className="snapshot-capacity-warning danger">
                예상 저장량이 안전한 쓰기 가능 공간을 초과합니다. 자동 맞춤을 켜거나 저장 범위를 줄여 주세요.
              </div>
            )}
          </>
        ) : (
          <div className="snapshot-capacity-warning danger">
            {estimate?.message || '저장 예상 용량을 계산할 수 없습니다.'}
          </div>
        )}
      </div>

      <div className="snapshot-save-row">
        <label htmlFor="snapshot-label-input">파일 이름 태그 (선택)</label>
        <input
          id="snapshot-label-input"
          type="text"
          value={label}
          maxLength={48}
          onChange={event => setLabel(event.target.value)}
          placeholder="예: 우측 레이더 점검"
          disabled={busy}
        />
        <button
          type="button"
          className={confirmWrite ? 'confirm' : ''}
          onClick={writeSnapshot}
          disabled={
            !status.available
            || busy
            || estimatePending
            || !estimate?.success
            || (!autoFit && !estimate?.fits_without_truncation)
            || !lookbackValid
            || !selectedTopics.length
          }
        >
          {busy ? '저장 중…' : (confirmWrite ? '확인 후 저장' : '스냅샷 저장')}
        </button>
        {confirmWrite && (
          <button type="button" className="cancel" onClick={() => setConfirmWrite(false)}>
            취소
          </button>
        )}
        <small>bag 내부에 TXT가 생성되지 않으며, 저장 폴더 이름 뒤에 붙습니다.</small>
      </div>

      <div className="snapshot-status-message" role="status">{message}</div>
      {lastResult.path && (
        <div className={`snapshot-last-result ${lastResult.success ? 'success' : 'failure'}`}>
          <span>{lastResult.success ? '최근 저장' : '최근 저장 시도'}</span>
          <code>{lastResult.path}</code>
          {lastResult.completed_at && <time>{formatTime(lastResult.completed_at)}</time>}
        </div>
      )}
    </section>
  );
}

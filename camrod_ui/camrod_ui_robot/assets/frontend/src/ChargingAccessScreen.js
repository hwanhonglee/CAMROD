import React, { useCallback, useEffect, useRef, useState } from 'react';
import './ChargingAccessScreen.css';

const PHASES = ['inactive', 'departing', 'ready', 'error'];
// HH_261102 - The requested move ends at standstill inside drop_zone; ready is not another locked workflow.
export const chargingAccessBusy = phase => phase === 'departing';

// HH_261102 - Only backend observations can grant cargo access; elapsed UI time cannot.
export function useChargingAccess(connected) {
  const [snapshot, setSnapshot] = useState(null);
  const [available, setAvailable] = useState(false);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState('');
  const pendingRef = useRef(false);
  const revision = useRef(0);
  useEffect(() => {
    setAvailable(false);
    if (!connected) return undefined;
    let disposed = false;
    let timer;
    let timeout;
    let controller;
    const poll = async () => {
      if (pendingRef.current) { timer = setTimeout(poll, 1000); return; }
      const requestedRevision = revision.current;
      controller = new AbortController();
      timeout = setTimeout(() => controller.abort(), 3000);
      try {
        const response = await fetch('/api/charging-access', {
          method: 'GET', cache: 'no-store', signal: controller.signal,
        });
        const body = await response.json();
        if (!disposed && requestedRevision === revision.current && !pendingRef.current) {
          if (!response.ok || !PHASES.includes(body?.phase)) throw new Error('invalid_status');
          setSnapshot(body);
          setAvailable(true);
        }
      } catch (_) {
        if (!disposed && requestedRevision === revision.current) setAvailable(false);
      } finally {
        clearTimeout(timeout);
        if (!disposed) timer = setTimeout(poll, 1000);
      }
    };
    poll();
    return () => {
      disposed = true;
      clearTimeout(timer);
      clearTimeout(timeout);
      controller?.abort();
    };
  }, [connected]);

  const request = useCallback(async () => {
    if (!connected || !available || pendingRef.current || snapshot?.can_leave !== true) return;
    pendingRef.current = true;
    revision.current += 1;
    setPending(true);
    setError('');
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 10000);
    try {
      // HH_261102 - This single confirmed move only clears the charging contact inside drop_zone.
      const response = await fetch('/ui/charging_access?action=leave&confirm=true', {
        method: 'POST', signal: controller.signal,
      });
      const body = await response.json();
      if (PHASES.includes(body?.phase)) {
        setSnapshot(body);
        setAvailable(true);
      }
      if (!response.ok || body?.success !== true || !PHASES.includes(body?.phase)) {
        throw new Error(body?.message || '이동 요청을 확인할 수 없습니다.');
      }
    } catch (failure) {
      setError(failure.name === 'AbortError'
        ? '응답을 기다리는 중 연결이 지연되었습니다. 현재 상태를 확인해 주세요.'
        : failure.message || '이동 요청을 확인할 수 없습니다.');
      setAvailable(false);
    } finally {
      clearTimeout(timeout);
      pendingRef.current = false;
      setPending(false);
    }
  }, [connected, available, snapshot]);
  return { snapshot, available: connected && available, pending, error, request };
}

// HH_261102 - Show charging status and one confirmed staging move, without adding a cargo-complete workflow.
export default function ChargingAccessScreen({ access, batteryChargeComplete, onServices, onStop, adminEntry }) {
  const { snapshot, available, pending, error, request } = access;
  const [confirmation, setConfirmation] = useState(null);
  const phase = snapshot?.phase || 'inactive';
  const battery = Number.isFinite(snapshot?.battery_percentage) && snapshot.battery_percentage >= 0
    ? snapshot.battery_percentage : null;
  const minimum = snapshot?.minimum_battery_percentage ?? 35;
  const enoughBattery = battery !== null && battery >= minimum;
  // HH_261102 - A fresh HTTP reply does not make cached vehicle/CAN feedback fresh.
  const observed = available && snapshot?.platform_feedback_fresh === true;
  const busy = chargingAccessBusy(phase) || pending;
  const canLeave = observed && enoughBattery && !pending && snapshot?.can_leave === true;
  const ready = observed && phase === 'ready' && snapshot?.cargo_access_ready === true;
  // HH_261102 - Public copy gives one state and one next action; technical evidence stays in a closed disclosure.
  const title = !observed ? '상태 확인 중'
    : phase === 'departing' ? '앞으로 이동 중'
      : phase === 'ready' ? (ready ? '정차 완료' : '정차 상태 확인 중')
        : phase === 'error' ? '이동할 수 없어요' : '짐을 꺼내시겠어요?';
  const instruction = !observed ? '잠시 기다려 주세요. 로봇에 접근하지 마세요.'
    : phase === 'departing' ? '멈출 때까지 로봇 주변을 비워 주세요.'
      : phase === 'ready' ? (ready ? '로봇에 짐을 실어주세요.' : '정차 안내가 나올 때까지 기다려 주세요.')
        : phase === 'error' ? '다시 시도하거나 관리자에게 문의해 주세요.'
          // HH_261102 - Missing SOC means unknown, not a measured low battery.
          : battery === null ? '배터리 확인 중입니다. 잠시 기다려 주세요.'
            : !enoughBattery ? '충전이 더 필요해요.'
            : !canLeave && !pending ? '지금은 이동할 수 없어요. 자세한 안내를 확인해 주세요.'
              : '짐을 꺼내려면 먼저 로봇을 앞으로 이동해 주세요.';
  const charging = !observed ? '충전 상태 확인 중'
    : snapshot?.platform_is_charging === true ? (batteryChargeComplete ? '충전 완료' : '충전 중')
      : snapshot?.platform_is_charging === false ? '충전 신호 없음' : '충전 신호 확인 중';

  useEffect(() => {
    // HH_261102 - A changed backend phase invalidates a previously opened motion confirmation.
    setConfirmation(current => current === 'stop' ? current : null);
  }, [phase, available, snapshot?.platform_feedback_fresh]);

  return <div className="charging-access-screen" data-ui="charging-screen" data-phase={phase}>
    {adminEntry}
    <header className="charging-header">
      <div className="charging-brand"><img src="/월악산_국립공원_로고.jpg" alt="월악산 국립공원 로고" />
        <div><span>월악산 국립공원 배송 로봇</span><strong>충전 안내</strong></div></div>
      <div className="charging-header-actions">
        <span className={`charging-connection${observed ? '' : ' unavailable'}`}>{observed ? '연결됨' : '연결 확인 중'}</span>
        <button type="button" className="charging-stop" data-ui="charging-safety-stop" onClick={() => setConfirmation('stop')}>운행 정지</button>
      </div>
    </header>
    <main className="charging-content">
      <section className="charging-status" aria-label="충전 상태">
        <div className="charging-soc"><span>{observed ? '현재 배터리' : '최근 수신 배터리'}</span><strong>{battery === null ? '—' : battery}<small>%</small></strong></div>
        <div className="charging-facts">
          <strong data-ui="charging-contact-state">{charging}</strong>
          <p className={observed && enoughBattery ? 'charging-eligible' : 'charging-ineligible'} data-ui="charging-eligibility">
            {!observed || battery === null ? '배터리 확인 중 · 출발할 수 없어요'
              : enoughBattery ? '배터리 준비 완료' : `${minimum}%부터 이용할 수 있어요`}
          </p>
        </div>
      </section>
      <section className={`charging-task charging-task-${phase}`} aria-live="polite">
        <h1>{title}</h1>
        <p data-ui="charging-access-message">{instruction}</p>
        <p className="charging-safety-note">충전 전원을 끄는 기능은 아닙니다. 충전 접점은 만지지 마세요.</p>
        {error && <p role="alert" className="charging-error">요청을 확인하지 못했어요. 자세한 안내를 확인해 주세요.</p>}
        <div className="charging-actions">
          {(phase === 'inactive' || phase === 'error') && <button type="button" className="charging-primary" data-ui="charging-access-leave"
            disabled={!canLeave} onClick={() => setConfirmation('leave')}>앞으로 이동</button>}
          {(phase === 'inactive' || phase === 'ready') && <button type="button" className="charging-secondary" data-ui="charging-open-services"
            disabled={!observed || busy || !enoughBattery || (phase === 'ready' && !ready)} onClick={onServices}>서비스 선택</button>}
        </div>
        {pending && <p role="status">요청 중…</p>}
        <details className="charging-details" data-ui="charging-details">
          <summary>자세한 안내</summary>
          {snapshot?.message && <p>{snapshot.message}</p>}
          {snapshot?.blocked_reason && snapshot.blocked_reason !== snapshot.message && <p>{snapshot.blocked_reason}</p>}
          {error && <p>{error}</p>}
          <p>정차 위치: {snapshot?.target_label || 'drop_zone 내 대기 위치'}</p>
          <p>배터리 {minimum}% 이상에서 새 서비스를 이용할 수 있습니다. 충전 신호와 완충 여부는 서로 다릅니다.</p>
          <p>이동 후 정차하며, 다음 이동은 별도로 서비스를 선택해야 시작됩니다.</p>
        </details>
      </section>
    </main>
    {confirmation && <div className="move-confirm-overlay" role="dialog" aria-modal="true" aria-labelledby="charging-confirm-title">
      <div className="move-confirm-box charging-confirm-box">
        <h2 id="charging-confirm-title">{confirmation === 'stop' ? '운행을 정지할까요?'
          : '앞으로 이동할까요?'}</h2>
        <p>{confirmation === 'stop' ? '진행 중인 미션이 종료됩니다. 긴급 시에는 로봇의 비상정지 버튼을 누르세요.'
          : '짐과 충전 접점에서 손을 떼고, 로봇 주변을 비워 주세요.'}</p>
        <div className="move-confirm-btns">
          <button type="button" className="move-confirm-no" data-ui="charging-confirm-cancel" onClick={() => setConfirmation(null)}>{confirmation === 'stop' ? '아니요' : '취소'}</button>
          {/* HH_261102 - Mission-ending Stop keeps its existing confirmation, independent of movement readiness. */}
          <button type="button" className="move-confirm-yes" data-ui={confirmation === 'stop' ? 'charging-confirm-stop' : 'charging-confirm-move'}
            disabled={confirmation !== 'stop' && !canLeave}
            onClick={() => { setConfirmation(null); if (confirmation === 'stop') onStop(); else request(); }}>
            {confirmation === 'stop' ? '예 · 운행 정지' : '확인 · 이동'}</button>
        </div>
      </div>
    </div>}
  </div>;
}

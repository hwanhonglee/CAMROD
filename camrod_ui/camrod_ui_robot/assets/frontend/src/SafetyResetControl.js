import React, { useCallback, useEffect, useRef, useState } from 'react';
import './SafetyResetControl.css';

const ENDPOINT = '/api/admin/safety/reset';
const blockerText = value => typeof value === 'string'
  ? value : value?.message || value?.reason || value?.code || '현재 안전 조건을 확인해주세요.';
const requestHeaders = (username, password) => ({
  Authorization: `Basic ${window.btoa(`${username}:${password}`)}`,
  'Content-Type': 'application/json',
});

// HH_261024 - Credentials are forwarded only for the protected reset endpoint.
// Opening this card reads status; only a second explicit confirmation can POST.
export default function SafetyResetControl({ username = '', password = '' }) {
  const [available, setAvailable] = useState(false);
  const [checking, setChecking] = useState(false);
  const [confirming, setConfirming] = useState(false);
  const [pending, setPending] = useState(false);
  const [message, setMessage] = useState('초기화 서비스 상태를 확인해주세요.');
  const [blockers, setBlockers] = useState([]);
  const [failed, setFailed] = useState(false);
  const inFlight = useRef(false);
  const epoch = useRef(0);
  const credentialsPresent = Boolean(username && password);
  const checkStatus = useCallback(async () => {
    if (!credentialsPresent || inFlight.current) return;
    const requestEpoch = epoch.current;
    inFlight.current = true;
    setChecking(true);
    setAvailable(false);
    setConfirming(false);
    setFailed(false);
    try {
      const response = await fetch(ENDPOINT, { method: 'GET', headers: requestHeaders(username, password) });
      const body = await response.json();
      if (requestEpoch !== epoch.current) return;
      if (!response.ok) throw new Error(body.message || body.detail || '초기화 상태 확인 실패');
      setAvailable(body.available === true);
      setMessage(body.message || (body.available === true
        ? '정지 원인을 확인한 뒤 초기화를 요청할 수 있습니다.' : '지금은 초기화할 수 없습니다.'));
      setBlockers(Array.isArray(body.blockers) ? body.blockers : []);
    } catch (error) {
      if (requestEpoch !== epoch.current) return;
      setFailed(true);
      setMessage(error.message || '초기화 서비스에 연결할 수 없습니다.');
    } finally {
      if (requestEpoch === epoch.current) {
        inFlight.current = false;
        setChecking(false);
      }
    }
  }, [credentialsPresent, username, password]);

  useEffect(() => {
    epoch.current += 1;
    inFlight.current = false;
    setAvailable(false);
    setChecking(false);
    setPending(false);
    setConfirming(false);
    setBlockers([]);
    if (credentialsPresent) checkStatus();
    return () => { epoch.current += 1; };
    // HH_261024 - No polling, automatic retry or reset is allowed on telemetry
    // updates. A credential change invalidates any stale in-flight response.
  }, [credentialsPresent, checkStatus]);

  const reset = async () => {
    if (!confirming || !available || !credentialsPresent || inFlight.current) return;
    const requestEpoch = epoch.current;
    inFlight.current = true;
    setPending(true);
    setConfirming(false);
    setFailed(false);
    try {
      const response = await fetch(ENDPOINT, {
        method: 'POST', headers: requestHeaders(username, password), body: JSON.stringify({ confirm: true }),
      });
      const body = await response.json();
      if (requestEpoch !== epoch.current) return;
      setBlockers(Array.isArray(body.blockers) ? body.blockers : []);
      setAvailable(body.available === true);
      // HH_261024 - A refusal is visible and must never be presented as a
      // successful safety bypass. The operator must explicitly request again.
      if (!response.ok || body.success !== true) {
        throw new Error(body.message || body.detail || '안전 조건 때문에 초기화가 거부되었습니다.');
      }
      setMessage(body.message || '복구 상태를 초기화했습니다. 주변을 확인한 뒤 별도로 주행을 시작해주세요.');
    } catch (error) {
      if (requestEpoch !== epoch.current) return;
      setFailed(true);
      setMessage(error.message || '초기화 요청 실패');
    } finally {
      if (requestEpoch === epoch.current) {
        inFlight.current = false;
        setPending(false);
      }
    }
  };

  return (
    <section className="safety-reset-card" data-ui="admin-safety-reset">
      <div className="safety-reset-heading">
        <div><span>관리자 복구</span><h3>안전정지 · 복구 초기화</h3></div>
        <button type="button" className="safety-reset-refresh" onClick={checkStatus}
          disabled={!credentialsPresent || checking || pending}>상태 새로 확인</button>
      </div>
      <p className="safety-reset-explanation">
        장애물 제거와 정지 원인 확인 후, 현재 미션을 종료하고 복구 대기·재시도 상태를 초기화합니다.
        비상정지·장애물 감지 등 현재 안전 조건은 해제하지 않습니다.
      </p>
      <p className="safety-reset-explicit-start">로봇은 정지 상태를 유지하며 현재 미션은 취소됩니다.
        이후 수동주행 ARM 또는 새 미션 시작을 별도로 요청해야 합니다.</p>
      <div className={`safety-reset-message${failed ? ' safety-reset-error' : ''}`}
        role={failed ? 'alert' : 'status'}>
        {pending ? '초기화 요청 중…' : checking ? '초기화 서비스 상태 확인 중…' : message}
      </div>
      {blockers.length > 0 && <ul className="safety-reset-blockers">
        {blockers.map((item, index) => <li key={index}>{blockerText(item)}</li>)}
      </ul>}
      {!confirming ? (
        <button type="button" className="safety-reset-request"
          data-ui="admin-safety-reset-request"
          disabled={!available || !credentialsPresent || checking || pending}
          onClick={() => setConfirming(true)}>
          {pending ? '초기화 요청 중…' : '정지 원인 확인 후 초기화'}
        </button>
      ) : (
        <div className="safety-reset-confirm" role="alertdialog" aria-modal="false"
          aria-labelledby="safety-reset-confirm-title">
          <strong id="safety-reset-confirm-title">주변 안전과 정지 원인을 확인하셨습니까?</strong>
          <p>현재 미션을 취소하고 정지 상태를 유지합니다. 비상정지와 현재 장애물 등 안전 조건은 그대로 유지됩니다.</p>
          <div><button type="button" onClick={() => setConfirming(false)}>아니요</button>
            <button type="button" className="safety-reset-request"
              data-ui="admin-safety-reset-confirm" onClick={reset}>예, 초기화 실행</button></div>
        </div>
      )}
    </section>
  );
}

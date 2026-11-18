import React, { useCallback, useEffect, useState } from 'react';
import './BringupShutdownControl.css';

// HJ_261118 - Admin-only bringup shutdown. The backend re-checks credentials,
// an explicit confirm, active missions and measured motion before it sends
// SIGINT to ros2 launch; this card only mirrors that status.
const ENDPOINT = '/api/admin/bringup/shutdown';
const POLL_MS = 2000;

const requestHeaders = (username, password) => ({
  Authorization: `Basic ${window.btoa(`${username}:${password}`)}`,
  'Content-Type': 'application/json',
});

export default function BringupShutdownControl({ username = '', password = '' }) {
  const [status, setStatus] = useState(null);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [pending, setPending] = useState(false);
  const [message, setMessage] = useState('');
  const [requested, setRequested] = useState(false);
  const credentialsPresent = Boolean(username && password);

  const refresh = useCallback(async () => {
    if (!credentialsPresent) return;
    try {
      const response = await fetch(ENDPOINT, { method: 'GET', headers: requestHeaders(username, password) });
      const body = await response.json();
      if (!response.ok) throw new Error(body.message || '상태를 확인할 수 없어요.');
      setStatus(body);
      if (body.requested) setRequested(true);
    } catch (error) {
      setStatus({ available: false, blockers: [], messages: [error.message || '상태를 확인할 수 없어요.'] });
    }
  }, [credentialsPresent, username, password]);

  useEffect(() => {
    refresh();
    const timer = setInterval(refresh, POLL_MS);
    return () => clearInterval(timer);
  }, [refresh]);

  const blockers = status?.blockers || [];
  const unavailable = status !== null && status.available === false;
  const blocked = !credentialsPresent || status === null || unavailable || blockers.length > 0;

  const submit = async () => {
    setPending(true);
    setMessage('');
    try {
      const response = await fetch(ENDPOINT, {
        method: 'POST', headers: requestHeaders(username, password), body: JSON.stringify({ confirm: true }),
      });
      const body = await response.json();
      if (!response.ok || !body.success) throw new Error(body.message || '종료 요청이 거절되었어요.');
      setRequested(true);
      setMessage(body.message || '소프트웨어를 종료합니다.');
    } catch (error) {
      setMessage(error.message || '종료 요청이 거절되었어요.');
      refresh();
    } finally {
      setPending(false);
      setConfirmOpen(false);
    }
  };

  let note = '로봇 소프트웨어를 모두 끕니다. 다시 켜려면 바탕화면 아이콘을 눌러 주세요.';
  if (!credentialsPresent) note = '관리자 인증 정보가 없어 종료할 수 없어요. 다시 로그인해 주세요.';
  else if (status === null) note = '종료 가능 여부를 확인하는 중이에요.';
  else if (unavailable) note = (status.messages && status.messages[0]) || '바탕화면 아이콘으로 실행된 상태가 아니라 종료할 수 없어요.';
  else if (blockers.length > 0) note = status.messages.join(' ');

  return (
    <section className="bringup-shutdown-card" data-ui="operator-bringup-shutdown-card"
      aria-labelledby="bringup-shutdown-title">
      <div className="bringup-shutdown-copy">
        <span className="bringup-shutdown-kicker">System Shutdown</span>
        <h3 id="bringup-shutdown-title">시스템 종료</h3>
        <p className={blocked && status !== null ? 'bringup-shutdown-blocked' : ''} role="status">
          {requested ? (message || '소프트웨어를 종료하는 중이에요. 잠시 후 화면이 닫혀요.') : (message || note)}
        </p>
      </div>
      <button type="button" className="bringup-shutdown-btn" data-ui="operator-bringup-shutdown"
        disabled={blocked || pending || requested} onClick={() => setConfirmOpen(true)}>
        {requested ? '종료 중…' : '소프트웨어 종료'}
      </button>

      {confirmOpen && (
        <div className="bringup-shutdown-overlay" role="dialog" aria-modal="true"
          aria-labelledby="bringup-shutdown-confirm-title">
          <div className="bringup-shutdown-dialog">
            <h4 id="bringup-shutdown-confirm-title">소프트웨어를 종료할까요?</h4>
            <div className="bringup-shutdown-actions">
              <button type="button" className="bringup-shutdown-cancel" data-ui="operator-bringup-shutdown-cancel"
                onClick={() => setConfirmOpen(false)} disabled={pending}>취소</button>
              <button type="button" className="bringup-shutdown-confirm" data-ui="operator-bringup-shutdown-confirm"
                onClick={submit} disabled={pending}>{pending ? '요청 중…' : '종료'}</button>
            </div>
          </div>
        </div>
      )}
    </section>
  );
}

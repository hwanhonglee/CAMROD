import React, { useEffect, useRef, useState } from 'react';
import './ManualMissionResume.css';

// HH_261002 - The CARLA opt-in backend owns the suspended mission token and
// admission checks. A missing/cleared contract must never create UI authority.
export function manualResumeFromSnapshot(current, snapshot) {
  if (!snapshot || typeof snapshot !== 'object') return current;
  if (snapshot.mission_dispatch_active === true) return null;
  if (Object.prototype.hasOwnProperty.call(snapshot, 'manual_resume')) {
    const value = snapshot.manual_resume;
    if (!value || value.pending !== true || typeof value.token !== 'string'
        || !value.token.trim()) return null;
    return { ...value, can_resume: value.can_resume === true };
  }
  // HH_261002 - An admitted replacement mission invalidates an older prompt,
  // even when a compatibility broadcast omits the optional resume contract.
  return current;
}

export default function ManualMissionResume({ resume, connected }) {
  const token = resume?.token || '';
  const tokenRef = useRef(token);
  tokenRef.current = token;
  const requestRef = useRef(null);
  const [request, setRequest] = useState(null);
  const [error, setError] = useState('');
  useEffect(() => {
    tokenRef.current = token;
    requestRef.current = null;
    setRequest(null);
    setError('');
    return () => { tokenRef.current = ''; };
  }, [token]);
  if (!resume?.pending || !token) return null;

  const busy = request?.token === token;
  const allowed = connected && resume.can_resume === true && !busy;
  const destination = resume.stage === 'return'
    ? `${resume.site || '사이트'}에서 대기·충전 장소로 복귀`
    : `${resume.site || '선택 사이트'} ${resume.intent === 'recall' ? '호출' : '배송'}`;

  const requestResume = async () => {
    // HH_261002 - Only this explicit click sends a command. Disarm, reconnect,
    // status updates and mounting are read-only; latch before React re-renders.
    if (!allowed || requestRef.current || tokenRef.current !== token) return;
    const capturedToken = token;
    const requestIdentity = {};
    requestRef.current = requestIdentity;
    setRequest({ token: capturedToken, accepted: false });
    setError('');
    try {
      const response = await fetch('/ui/manual_resume', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ token: capturedToken }),
      });
      const result = await response.json();
      if (tokenRef.current !== capturedToken || requestRef.current !== requestIdentity) return;
      if (!response.ok || result.success !== true || result.accepted !== true) {
        throw new Error(result.message || result.error || '재개 조건을 확인하고 다시 눌러주세요.');
      }
      // HH_261002 - Keep a consumed token disabled until authoritative state
      // clears/replaces it; an HTTP acknowledgement alone is not proof of motion.
      setRequest({ token: capturedToken, accepted: true });
    } catch (reason) {
      if (tokenRef.current !== capturedToken || requestRef.current !== requestIdentity) return;
      requestRef.current = null;
      setRequest(null);
      setError(reason.message || '재개 요청을 보내지 못했습니다. 연결을 확인해주세요.');
    }
  };

  return (
    <section className="manual-mission-resume" data-ui="manual-mission-resume"
      aria-label="일시정지한 자율주행 재개">
      <div className="manual-mission-resume-copy">
        <strong>수동 개입으로 자율주행 일시정지</strong>
        <span>{destination}</span>
        <p role="status" aria-live="polite">
          {!connected ? '로봇 연결을 확인해주세요. 연결되어도 자동으로 재개하지 않습니다.'
            : request?.accepted ? '재개 요청이 수락됐습니다. 로봇 상태를 확인하고 있습니다.'
              : resume.message || (resume.can_resume
                ? '주변 안전을 확인한 뒤 재개 버튼을 눌러주세요.' : '재개 조건을 확인하고 있습니다.')}
        </p>
        {!resume.can_resume && resume.reason && (
          <small data-ui="manual-resume-reason">대기 사유: {resume.reason}</small>
        )}
        {error && <p className="manual-mission-resume-error" role="alert">{error}</p>}
      </div>
      <button type="button" data-ui="manual-mission-resume-confirm"
        disabled={!allowed} onClick={requestResume}>
        {busy ? (request.accepted ? '재개 상태 확인 중…' : '재개 요청 중…') : '자율주행 재개'}
      </button>
    </section>
  );
}

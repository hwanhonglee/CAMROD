import React, { useCallback, useEffect, useRef, useState } from 'react';

const EMPTY_STATE = Object.freeze({
  available: false,
  connected: false,
  armed: false,
  holding: false,
  reason: '수동 제어 연결 대기',
  limits: {
    linear_x_mps: 0.2,
    lateral_y_mps: 0.2,
    angular_z_radps: 0.2,
  },
  command: { linear_x: 0, linear_y: 0, angular_z: 0 },
});

const MOTION_KEYS = new Set([
  'KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyZ', 'KeyC',
  'ArrowUp', 'ArrowLeft', 'ArrowDown', 'ArrowRight',
]);

const isTextEntry = target => {
  const tag = String(target?.tagName || '').toLowerCase();
  return target?.isContentEditable || ['input', 'textarea', 'select'].includes(tag);
};

const normalizedCommand = pressed => {
  const forward = Number(pressed.has('KeyW') || pressed.has('ArrowUp'))
    - Number(pressed.has('KeyS') || pressed.has('ArrowDown'));
  const turn = Number(pressed.has('KeyA') || pressed.has('ArrowLeft'))
    - Number(pressed.has('KeyD') || pressed.has('ArrowRight'));
  const crab = Number(pressed.has('KeyZ')) - Number(pressed.has('KeyC'));

  // Crab is an explicit 4WS mode. Ambiguous mixed-mode input is a zero
  // command instead of relying on an implicit priority.
  if (crab !== 0 && (forward !== 0 || turn !== 0)) {
    return { forward: 0, turn: 0, crab: 0, conflict: true };
  }
  return { forward, turn, crab, conflict: false };
};

export default function ManualDrivePanel() {
  const [manual, setManual] = useState(EMPTY_STATE);
  const [pressed, setPressed] = useState(() => new Set());
  const [scale, setScale] = useState(1.0);
  const [transportError, setTransportError] = useState('');
  const socketRef = useRef(null);
  const sequenceRef = useRef(0);
  const pressedRef = useRef(new Set());
  const manualRef = useRef(EMPTY_STATE);
  const scaleRef = useRef(1.0);

  const applyState = useCallback(next => {
    const merged = {
      ...EMPTY_STATE,
      ...(next || {}),
      limits: { ...EMPTY_STATE.limits, ...(next?.limits || {}) },
      command: { ...EMPTY_STATE.command, ...(next?.command || {}) },
    };
    manualRef.current = merged;
    setManual(merged);
  }, []);

  const send = useCallback((type, payload = {}) => {
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    sequenceRef.current += 1;
    socket.send(JSON.stringify({ type, seq: sequenceRef.current, ...payload }));
    return true;
  }, []);

  const sendDrive = useCallback(() => {
    if (!manualRef.current.armed) return;
    const command = normalizedCommand(pressedRef.current);
    send('drive', {
      forward: command.forward,
      turn: command.turn,
      crab: command.crab,
      scale: scaleRef.current,
    });
    if (command.conflict) {
      setTransportError('Crab과 전진/회전은 동시에 사용할 수 없습니다.');
    }
  }, [send]);

  const replacePressed = useCallback(next => {
    pressedRef.current = next;
    setPressed(new Set(next));
  }, []);

  const clearMotion = useCallback((disarm = false) => {
    replacePressed(new Set());
    if (disarm) {
      send('disarm');
    } else if (manualRef.current.armed) {
      send('drive', { forward: 0, turn: 0, crab: 0, scale: scaleRef.current });
    }
  }, [replacePressed, send]);

  useEffect(() => {
    let mounted = true;
    let reconnectTimer = null;

    const connect = () => {
      if (!mounted) return;
      const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
      const host = window.location.host || '127.0.0.1:8010';
      let socket;
      try {
        socket = new WebSocket(`${protocol}://${host}/ws/manual-drive`);
      } catch (error) {
        setTransportError(error.message || '수동 제어 연결 실패');
        reconnectTimer = setTimeout(connect, 1500);
        return;
      }
      socketRef.current = socket;
      socket.onopen = () => {
        if (!mounted) return;
        setTransportError('');
      };
      socket.onmessage = event => {
        try {
          const payload = JSON.parse(event.data);
          if (payload.manual_drive) applyState(payload.manual_drive);
          if (payload.type === 'error') {
            setTransportError(payload.message || payload.error || '수동 명령 거부');
          } else if (payload.type === 'state') {
            setTransportError('');
          }
        } catch (error) {
          setTransportError(error.message || '잘못된 수동 제어 응답');
        }
      };
      socket.onerror = () => socket.close();
      socket.onclose = event => {
        if (socketRef.current === socket) socketRef.current = null;
        if (!mounted) return;
        replacePressed(new Set());
        const disabled = event.code === 4403;
        const busy = event.code === 4409;
        applyState({
          ...manualRef.current,
          connected: false,
          armed: false,
          holding: false,
          reason: disabled
            ? '이 실행 모드에서는 수동 제어가 비활성화되어 있습니다.'
            : (busy ? '다른 UI가 수동 제어를 사용 중입니다.' : '수동 제어 연결 끊김'),
          command: { linear_x: 0, linear_y: 0, angular_z: 0 },
        });
        // Disabled/default CAMROD and a rejected second operator should not
        // create a permanent reconnect loop. Remounting the Camera tab is an
        // explicit retry; unexpected transport loss still reconnects safely.
        if (!disabled && !busy) reconnectTimer = setTimeout(connect, 1500);
      };
    };

    connect();
    return () => {
      mounted = false;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      const socket = socketRef.current;
      if (socket && socket.readyState === WebSocket.OPEN) {
        sequenceRef.current += 1;
        socket.send(JSON.stringify({ type: 'disarm', seq: sequenceRef.current }));
      }
      if (socket) socket.close();
      socketRef.current = null;
      pressedRef.current = new Set();
    };
  }, [applyState, replacePressed]);

  useEffect(() => {
    const timer = setInterval(sendDrive, 100);
    return () => clearInterval(timer);
  }, [sendDrive]);

  useEffect(() => {
    const keyDown = event => {
      if (isTextEntry(event.target)) return;
      if (event.code === 'Space') {
        event.preventDefault();
        clearMotion(false);
        return;
      }
      if (event.code === 'Escape') {
        event.preventDefault();
        clearMotion(true);
        return;
      }
      if (!MOTION_KEYS.has(event.code) || !manualRef.current.armed) return;
      event.preventDefault();
      if (event.repeat || pressedRef.current.has(event.code)) return;
      const next = new Set(pressedRef.current);
      next.add(event.code);
      replacePressed(next);
      sendDrive();
    };
    const keyUp = event => {
      if (!MOTION_KEYS.has(event.code)) return;
      event.preventDefault();
      const next = new Set(pressedRef.current);
      next.delete(event.code);
      replacePressed(next);
      sendDrive();
    };
    const loseFocus = () => clearMotion(true);
    const visibility = () => {
      if (document.hidden) loseFocus();
    };
    window.addEventListener('keydown', keyDown);
    window.addEventListener('keyup', keyUp);
    window.addEventListener('blur', loseFocus);
    window.addEventListener('pagehide', loseFocus);
    document.addEventListener('visibilitychange', visibility);
    return () => {
      window.removeEventListener('keydown', keyDown);
      window.removeEventListener('keyup', keyUp);
      window.removeEventListener('blur', loseFocus);
      window.removeEventListener('pagehide', loseFocus);
      document.removeEventListener('visibilitychange', visibility);
    };
  }, [clearMotion, replacePressed, sendDrive]);

  const changeButton = (code, active) => event => {
    event.preventDefault();
    if (!manualRef.current.armed) return;
    const next = new Set(pressedRef.current);
    if (active) next.add(code);
    else next.delete(code);
    replacePressed(next);
    sendDrive();
  };

  const buttonProps = code => ({
    className: `manual-drive-key ${pressed.has(code) ? 'pressed' : ''}`,
    onPointerDown: changeButton(code, true),
    onPointerUp: changeButton(code, false),
    onPointerCancel: changeButton(code, false),
    onPointerLeave: changeButton(code, false),
    disabled: !manual.armed,
  });

  const toggleArm = () => {
    clearMotion(false);
    send(manual.armed ? 'disarm' : 'arm');
  };

  const updateScale = event => {
    const value = Math.max(0.1, Math.min(1.0, Number(event.target.value)));
    scaleRef.current = value;
    setScale(value);
  };

  if (!manual.available && !manual.connected) return null;

  const command = manual.command || EMPTY_STATE.command;
  return (
    <section className="telemetry-section manual-drive-panel" aria-label="CARLA manual drive controls">
      <div className="manual-drive-header">
        <div>
          <h3>CARLA 수동주행</h3>
          <p>수동 시작 한 번으로 기존 목표를 취소하고 CAMROD safety gate를 통해 조작합니다.</p>
        </div>
        <div className={`manual-drive-state ${manual.armed ? 'armed' : ''}`}>
          <span />{manual.armed ? 'ARMED' : (manual.connected ? 'STANDBY' : 'OFFLINE')}
        </div>
      </div>

      <div className="manual-drive-layout">
        <div className="manual-drive-pad">
          <button {...buttonProps('KeyW')}>W<small>전진</small></button>
          <div className="manual-drive-pad-row">
            <button {...buttonProps('KeyA')}>A<small>좌회전</small></button>
            <button
              className="manual-drive-key stop"
              onClick={() => clearMotion(false)}
              disabled={!manual.armed}
            >SPACE<small>즉시 정지</small></button>
            <button {...buttonProps('KeyD')}>D<small>우회전</small></button>
          </div>
          <button {...buttonProps('KeyS')}>S<small>후진</small></button>
          <div className="manual-drive-pad-row crab">
            <button {...buttonProps('KeyZ')}>Z<small>좌 Crab</small></button>
            <button {...buttonProps('KeyC')}>C<small>우 Crab</small></button>
          </div>
        </div>

        <div className="manual-drive-config">
          <button
            type="button"
            className={`manual-drive-arm ${manual.armed ? 'active' : ''}`}
            onClick={toggleArm}
            disabled={!manual.available || !manual.connected}
          >
            {manual.armed ? '수동주행 종료 · ZERO' : '수동주행 시작 · STOP 후 ENGAGE'}
          </button>
          <label>
            속도 {Math.round(scale * 100)}%
            <input
              type="range"
              min="0.1"
              max="1.0"
              step="0.1"
              value={scale}
              onChange={updateScale}
            />
          </label>
          <div className="manual-drive-readout">
            <span>X <b>{Number(command.linear_x || 0).toFixed(2)}</b> m/s</span>
            <span>Y <b>{Number(command.linear_y || 0).toFixed(2)}</b> m/s</span>
            <span>Yaw <b>{Number(command.angular_z || 0).toFixed(2)}</b> rad/s</span>
          </div>
          <p className="manual-drive-hint">
            W/S·방향키: 전후진 · A/D: 조향/제자리회전 · Z/C: Crab · Space: 정지 · Esc: 해제
          </p>
          <p className={`manual-drive-message ${transportError ? 'error' : ''}`}>
            {transportError || manual.reason || '명령 대기'}
          </p>
        </div>
      </div>
    </section>
  );
}

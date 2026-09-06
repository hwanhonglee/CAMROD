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

const DRIVE_MODES = Object.freeze([
  {
    id: 'ackermann',
    label: '직진·조향',
    help: 'W/S + A/D',
  },
  {
    id: 'zero_turn',
    label: '제자리회전',
    help: 'A/D 단독',
  },
  {
    id: 'crab',
    label: 'Crab',
    help: 'Z/C 단독',
  },
]);

const DEFAULT_DRIVE_MODE = 'ackermann';

const isTextEntry = target => {
  const tag = String(target?.tagName || '').toLowerCase();
  return target?.isContentEditable || ['input', 'textarea', 'select'].includes(tag);
};

const classifiedCommand = (pressed, currentMode) => {
  const hasLongitudinal = pressed.has('KeyW') || pressed.has('ArrowUp')
    || pressed.has('KeyS') || pressed.has('ArrowDown');
  const hasTurn = pressed.has('KeyA') || pressed.has('ArrowLeft')
    || pressed.has('KeyD') || pressed.has('ArrowRight');
  const hasCrab = pressed.has('KeyZ') || pressed.has('KeyC');
  const forward = Number(pressed.has('KeyW') || pressed.has('ArrowUp'))
    - Number(pressed.has('KeyS') || pressed.has('ArrowDown'));
  const turn = Number(pressed.has('KeyA') || pressed.has('ArrowLeft'))
    - Number(pressed.has('KeyD') || pressed.has('ArrowRight'));
  const crab = Number(pressed.has('KeyZ')) - Number(pressed.has('KeyC'));

  // Never resolve a mixed crab/conventional key chord by implicit priority.
  // The caller sends an exact zero in the current mode until the conflict is
  // released, so no frame can carry axes from two 4WS kinematic modes.
  if (hasCrab && (hasLongitudinal || hasTurn)) {
    return {
      mode: currentMode, forward: 0, turn: 0, crab: 0, conflict: true,
    };
  }
  if (hasCrab) {
    return {
      mode: 'crab', forward: 0, turn: 0, crab, conflict: false,
    };
  }
  if (hasLongitudinal) {
    return {
      mode: 'ackermann',
      forward,
      turn: forward === 0 ? 0 : turn,
      crab: 0,
      conflict: false,
    };
  }
  if (hasTurn) {
    return {
      mode: 'zero_turn', forward: 0, turn, crab: 0, conflict: false,
    };
  }
  return {
    mode: currentMode, forward: 0, turn: 0, crab: 0, conflict: false,
  };
};

export default function ManualDrivePanel() {
  const [manual, setManual] = useState(EMPTY_STATE);
  const [pressed, setPressed] = useState(() => new Set());
  const [driveMode, setDriveMode] = useState(DEFAULT_DRIVE_MODE);
  const [armPending, setArmPending] = useState(false);
  const [collapsed, setCollapsed] = useState(true);
  const [scale, setScale] = useState(1.0);
  const [transportError, setTransportError] = useState('');
  const [inputError, setInputError] = useState('');
  const socketRef = useRef(null);
  const sequenceRef = useRef(0);
  const pressedRef = useRef(new Set());
  const driveModeRef = useRef(DEFAULT_DRIVE_MODE);
  const armPendingRef = useRef(false);
  const disarmPendingRef = useRef(false);
  const manualRef = useRef(EMPTY_STATE);
  const scaleRef = useRef(1.0);
  const manualEnabled = manual.available || manual.connected;

  const applyState = useCallback(next => {
    const merged = {
      ...EMPTY_STATE,
      ...(next || {}),
      limits: { ...EMPTY_STATE.limits, ...(next?.limits || {}) },
      command: { ...EMPTY_STATE.command, ...(next?.command || {}) },
    };
    manualRef.current = merged;
    setManual(merged);
    return merged;
  }, []);

  const send = useCallback((type, payload = {}) => {
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    sequenceRef.current += 1;
    socket.send(JSON.stringify({ type, seq: sequenceRef.current, ...payload }));
    return true;
  }, []);

  const replacePressed = useCallback(next => {
    pressedRef.current = next;
    setPressed(new Set(next));
  }, []);

  const sendZero = useCallback((mode = driveModeRef.current) => {
    if (!manualRef.current.armed || disarmPendingRef.current) return false;
    return send('drive', {
      mode,
      forward: 0,
      turn: 0,
      crab: 0,
      scale: scaleRef.current,
    });
  }, [send]);

  const sendDrive = useCallback(() => {
    if (!manualRef.current.armed || disarmPendingRef.current) return;
    const command = classifiedCommand(pressedRef.current, driveModeRef.current);
    if (command.conflict) {
      setInputError('Crab(Z/C)은 전후진·회전 키와 동시에 사용할 수 없습니다.');
      sendZero();
      return;
    }
    setInputError('');

    if (command.mode !== driveModeRef.current) {
      const previousMode = driveModeRef.current;
      // Cut the old kinematic mode with an exact zero. Keep the held key set,
      // then let a later heartbeat start the newly classified mode. This
      // prevents a single event from mixing an old-mode stop and new motion.
      if (!sendZero(previousMode)) return;
      driveModeRef.current = command.mode;
      setDriveMode(command.mode);
      return;
    }

    send('drive', {
      mode: command.mode,
      forward: command.forward,
      turn: command.turn,
      crab: command.crab,
      scale: scaleRef.current,
    });
  }, [send, sendZero]);

  const clearMotion = useCallback((disarm = false) => {
    replacePressed(new Set());
    setInputError('');
    if (disarm) {
      if (disarmPendingRef.current) return;
      // When currently armed, place an explicit zero immediately before the
      // DISARM frame. The pending guard then blocks interval/key heartbeats
      // until the backend acknowledges armed=false.
      sendZero();
      disarmPendingRef.current = true;
      armPendingRef.current = false;
      setArmPending(false);
      send('disarm');
    } else {
      sendZero();
    }
  }, [replacePressed, send, sendZero]);

  const toggleCollapsed = useCallback(() => {
    // Folding the visible controls is an operator context change. Cut any
    // held motion first, but deliberately retain ARM and the WebSocket so the
    // compact dock can be reopened without reacquiring manual ownership.
    if (!collapsed) clearMotion(false);
    setCollapsed(value => !value);
  }, [clearMotion, collapsed]);

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
          const nextManual = payload.manual_drive
            ? applyState(payload.manual_drive)
            : null;
          if (nextManual && !nextManual.armed) {
            disarmPendingRef.current = false;
          }
          if (nextManual?.armed && armPendingRef.current) {
            // The interval heartbeat may use keys collected while ARM was in
            // flight only after the server has acknowledged armed=true.
            armPendingRef.current = false;
            setArmPending(false);
          }
          if (payload.type === 'error') {
            armPendingRef.current = false;
            setArmPending(false);
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
        armPendingRef.current = false;
        disarmPendingRef.current = false;
        setArmPending(false);
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
      const manualTransitionActive = manualRef.current.armed || armPendingRef.current;
      if (socket && socket.readyState === WebSocket.OPEN && manualTransitionActive) {
        sequenceRef.current += 1;
        socket.send(JSON.stringify({ type: 'disarm', seq: sequenceRef.current }));
      }
      if (socket) socket.close();
      socketRef.current = null;
      pressedRef.current = new Set();
      armPendingRef.current = false;
      disarmPendingRef.current = true;
    };
  }, [applyState, replacePressed]);

  useEffect(() => {
    if (!manualEnabled) return undefined;
    const timer = setInterval(sendDrive, 100);
    return () => clearInterval(timer);
  }, [manualEnabled, sendDrive]);

  useEffect(() => {
    // Keep the ordinary develop UI completely inert when the CARLA-only
    // backend topic is absent. In particular, do not consume Space/Escape or
    // install a 10 Hz heartbeat timer for a panel that is not rendered.
    if (!manualEnabled) return undefined;
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
      if (!MOTION_KEYS.has(event.code)) return;
      const canTrackMotion = manualRef.current.armed || armPendingRef.current;
      if (!canTrackMotion) return;
      event.preventDefault();
      // Capture only a fresh key-down that occurs after ARM was requested.
      // A key held before the operator clicked ARM produces repeat events;
      // ignoring those prevents an unexpected departure at acknowledgement.
      if (event.repeat || pressedRef.current.has(event.code)) return;
      const next = new Set(pressedRef.current);
      next.add(event.code);
      replacePressed(next);
      if (manualRef.current.armed) sendDrive();
    };
    const keyUp = event => {
      if (!MOTION_KEYS.has(event.code)) return;
      event.preventDefault();
      const next = new Set(pressedRef.current);
      next.delete(event.code);
      replacePressed(next);
      if (manualRef.current.armed) sendDrive();
    };
    const stopForBlur = () => clearMotion(false);
    const disarmForPageLifecycle = () => clearMotion(true);
    const visibility = () => {
      if (document.hidden) disarmForPageLifecycle();
    };
    window.addEventListener('keydown', keyDown);
    window.addEventListener('keyup', keyUp);
    window.addEventListener('blur', stopForBlur);
    window.addEventListener('pagehide', disarmForPageLifecycle);
    document.addEventListener('visibilitychange', visibility);
    return () => {
      window.removeEventListener('keydown', keyDown);
      window.removeEventListener('keyup', keyUp);
      window.removeEventListener('blur', stopForBlur);
      window.removeEventListener('pagehide', disarmForPageLifecycle);
      document.removeEventListener('visibilitychange', visibility);
    };
  }, [clearMotion, manualEnabled, replacePressed, sendDrive]);

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
    if (manualRef.current.armed) {
      clearMotion(true);
      return;
    }
    if (armPendingRef.current || disarmPendingRef.current) return;

    clearMotion(false);
    disarmPendingRef.current = false;
    if (send('arm')) {
      armPendingRef.current = true;
      setArmPending(true);
    }
  };

  const updateScale = event => {
    const value = Math.max(0.1, Math.min(1.0, Number(event.target.value)));
    scaleRef.current = value;
    setScale(value);
  };

  if (!manualEnabled) return null;

  const command = manual.command || EMPTY_STATE.command;
  const requestedLinearMps = Number(manual.limits?.linear_x_mps || 0) * scale;
  const requestedLateralMps = Number(manual.limits?.lateral_y_mps || 0) * scale;
  const requestedAngularRadps = Number(manual.limits?.angular_z_radps || 0) * scale;
  const activeSpeed = driveMode === 'zero_turn'
    ? { axis: 'Yaw', value: requestedAngularRadps, unit: 'rad/s' }
    : (driveMode === 'crab'
      ? { axis: 'Y', value: requestedLateralMps, unit: 'm/s' }
      : { axis: 'X', value: requestedLinearMps, unit: 'm/s' });
  const activeModeLabel = DRIVE_MODES.find(mode => mode.id === driveMode)?.label || driveMode;
  return (
    <section
      className={`telemetry-section manual-drive-panel ${collapsed ? 'collapsed' : 'expanded'}`}
      data-ui="manual-drive-panel"
      data-connected={manual.connected ? 'true' : 'false'}
      data-armed={manual.armed ? 'true' : 'false'}
      data-mode={driveMode}
      data-linear-limit-mps={Number(manual.limits?.linear_x_mps || 0)}
      data-lateral-limit-mps={Number(manual.limits?.lateral_y_mps || 0)}
      data-angular-limit-radps={Number(manual.limits?.angular_z_radps || 0)}
      aria-label="CARLA manual drive controls"
    >
      <div className="manual-drive-header">
        <div className="manual-drive-title">
          <h3>CARLA 수동주행</h3>
          <p>수동 시작 한 번으로 기존 목표를 취소하고 CAMROD safety gate를 통해 조작합니다.</p>
        </div>
        <div className="manual-drive-compact-status" aria-label="접힌 수동주행 상태">
          <strong>{activeModeLabel}</strong>
          <span>X {Number(command.linear_x || 0).toFixed(2)}</span>
          <span>Y {Number(command.linear_y || 0).toFixed(2)}</span>
          <span>Yaw {Number(command.angular_z || 0).toFixed(2)}</span>
        </div>
        <div className="manual-drive-header-actions">
          <div
            className={`manual-drive-state ${manual.armed ? 'armed' : (armPending ? 'pending' : '')}`}
            data-ui="manual-drive-state"
          >
            <span />{manual.armed ? 'ARMED' : (armPending ? 'ARMING' : (manual.connected ? 'STANDBY' : 'OFFLINE'))}
          </div>
          <div className="manual-drive-compact-actions">
            <button
              type="button"
              className="manual-drive-compact-stop"
              data-ui="manual-drive-zero"
              onClick={() => clearMotion(false)}
              disabled={!manual.armed}
            >ZERO</button>
            <button
              type="button"
              className="manual-drive-compact-disarm"
              data-ui="manual-drive-disarm"
              onClick={() => clearMotion(true)}
              disabled={!manual.armed && !armPending}
            >DISARM</button>
          </div>
          <button
            type="button"
            className="manual-drive-toggle"
            data-ui="manual-drive-toggle"
            aria-expanded={!collapsed}
            aria-controls="manual-drive-expanded-controls"
            onClick={toggleCollapsed}
          >
            {collapsed ? '제어 열기' : '제어 접기'}
          </button>
        </div>
      </div>

      <div
        id="manual-drive-expanded-controls"
        className="manual-drive-layout"
        hidden={collapsed}
      >
        <div className="manual-drive-controls">
          <div className="manual-drive-mode-selector" role="status" aria-label="자동 분류 주행 모드">
            {DRIVE_MODES.map(mode => (
              <div
                key={mode.id}
                aria-current={driveMode === mode.id ? 'true' : undefined}
                className={`manual-drive-mode ${driveMode === mode.id ? 'active' : ''}`}
              >
                <strong>{mode.label}</strong>
                <small>{mode.help}</small>
              </div>
            ))}
          </div>

          <div className={`manual-drive-pad mode-${driveMode}`}>
            <button {...buttonProps('KeyW')}>W<small>전진</small></button>
            <div className="manual-drive-pad-row">
              <button {...buttonProps('KeyA')}>
                A<small>좌조향 · 반시계</small>
              </button>
              <button
                className="manual-drive-key stop"
                onClick={() => clearMotion(false)}
                disabled={!manual.armed}
              >SPACE<small>즉시 정지</small></button>
              <button {...buttonProps('KeyD')}>
                D<small>우조향 · 시계</small>
              </button>
            </div>
            <button {...buttonProps('KeyS')}>S<small>후진</small></button>
            <div className="manual-drive-pad-row crab">
              <button {...buttonProps('KeyZ')}>Z<small>좌 Crab</small></button>
              <button {...buttonProps('KeyC')}>C<small>우 Crab</small></button>
            </div>
          </div>
        </div>

        <div className="manual-drive-config">
          <button
            type="button"
            className={`manual-drive-arm ${manual.armed ? 'active' : ''}`}
            data-ui="manual-drive-arm"
            onClick={toggleArm}
            disabled={!manual.available || !manual.connected || armPending}
          >
            {manual.armed
              ? '수동주행 종료 · ZERO'
              : (armPending ? '수동주행 준비 중 · ARM ACK 대기' : '수동주행 시작 · STOP 후 ENGAGE')}
          </button>
          <label>
            속도 {activeSpeed.axis} {activeSpeed.value.toFixed(2)} {activeSpeed.unit} · {Math.round(scale * 100)}%
            <input
              type="range"
              data-ui="manual-drive-scale"
              min="0.1"
              max="1.0"
              step="0.05"
              value={scale}
              onChange={updateScale}
            />
          </label>
          <div className="manual-drive-limits">
            선택값 X ±{requestedLinearMps.toFixed(2)} m/s · Crab ±{requestedLateralMps.toFixed(2)} m/s · Yaw ±{requestedAngularRadps.toFixed(2)} rad/s
          </div>
          <div className="manual-drive-readout">
            <span>X <b>{Number(command.linear_x || 0).toFixed(2)}</b> m/s</span>
            <span>Y <b>{Number(command.linear_y || 0).toFixed(2)}</b> m/s</span>
            <span>Yaw <b>{Number(command.angular_z || 0).toFixed(2)}</b> rad/s</span>
          </div>
          <p className="manual-drive-hint">
            W/S + A/D: 직진·조향 · A/D 단독: 제자리회전 · Z/C 단독: Crab · 모드 변경 시 자동 ZERO
          </p>
          <p className={`manual-drive-message ${transportError || inputError ? 'error' : ''}`}>
            {transportError || inputError || manual.reason || '명령 대기'}
          </p>
        </div>
      </div>
    </section>
  );
}

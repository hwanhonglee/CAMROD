import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

const OUTBOUND_STATES = new Set([
  'MOVING_TO_SITE', 'GUEST_RECALL_SERVICE', 'RECALL_TO_SITE_ROAD',
  'DEPARTING_CHARGER', 'DEPARTING_DROP_ZONE', 'SITE_ENTRY',
]);
const RETURN_STATES = new Set([
  'RETURNING_TO_DROP_ZONE', 'RETURN_WITH_CARGO', 'DROP_ZONE_PARKING',
]);
// HH_261001 - Read the bounded navigation cache at <=10 Hz only while visible.
// Rendering interpolates received poses; this is not a control-loop frequency.
const POLL_MS = 100;
const REQUEST_TIMEOUT_MS = 3000;
const now = () => performance.now();

function unavailable(reason) {
  // HH_261001 - An unavailable response must not retain geometry or a misleading 0 speed.
  return {
    connected: false,
    unavailable_reason: reason,
    pose: null,
    route: null,
    perception: null,
    motion: { speed_mps: null },
    battery: { percentage: null },
    progress: { valid: false, remaining_distance_m: null, remaining_time_s: null },
  };
}

// HH_261001 - Never make an old ROS sample young merely because HTTP delivered it again.
function advanceAge(source, elapsedSeconds) {
  if (!source || typeof source !== 'object') return source;
  if (!Number.isFinite(source.age_s)) return source;
  return { ...source, age_s: source.age_s + elapsedSeconds };
}

function advancePerceptionAge(source, elapsedSeconds) {
  const result = advanceAge(source, elapsedSeconds);
  if (!result || typeof result !== 'object') return result;
  // HH_261001 - Independent streams must age independently while the next GET is delayed.
  const agedStreams = Object.fromEntries(['points_age_s', 'objects_age_s']
    .filter(key => Number.isFinite(result[key]))
    .map(key => [key, result[key] + elapsedSeconds]));
  return { ...result, ...agedStreams };
}

/** HH_261001 - A display-only observer. No mission command is sent by this hook. */
export default function useDrivingDisplay({
  missionDispatch = {}, serviceStateName = '', missionPhase = '',
  connected = false, blocked = false, injectedSnapshot = null,
}) {
  const { active = false, generation = 0, site = '', intent = '', owner = '' } = missionDispatch;
  const leg = RETURN_STATES.has(serviceStateName) ? 'return'
    : OUTBOUND_STATES.has(serviceStateName) ? 'outbound' : '';
  const missionIdentity = `${generation}:${site}:${intent}`;
  const hasMission = active && Number.isSafeInteger(generation) && generation > 0
    && Boolean(site) && ['delivery', 'recall'].includes(intent);
  // HH_261002 - SAFETY_STOP retains the accepted leg and its display latch.
  // It is an in-mission hold; STOPPED/ERROR are terminal presentation phases.
  const eligible = hasMission && Boolean(leg)
    && !['INITIALIZING', 'STOPPED', 'ERROR'].includes(missionPhase);
  const key = eligible ? `${missionIdentity}:${leg}` : '';
  const dismissedKeys = useRef(new Set());
  const previousIdentity = useRef(missionIdentity);
  const [openKey, setOpenKey] = useState('');
  const [idleOpen, setIdleOpen] = useState(false);
  const [received, setReceived] = useState(null);
  const [clock, setClock] = useState(now);
  const hasInjectedSnapshot = injectedSnapshot !== null;
  const injectedAt = useMemo(() => injectedSnapshot === null ? 0 : now(), [injectedSnapshot]);
  const idleEligible = !active;
  const canOpen = Boolean(connected && !blocked && (eligible || idleEligible));
  // HH_261001 - Connection is required to open the display; an already-open display stays
  // visible on connection loss so it can show unavailable instead of old data.
  const visible = Boolean(!blocked && ((eligible && openKey === key)
    || (idleEligible && idleOpen)));
  const displayIdentity = eligible ? missionIdentity : 'idle';

  useEffect(() => {
    // HH_261001 - A dismissed outbound/return leg stays dismissed during heartbeats and
    // parking sub-states. A truly new mission identity clears that latch.
    if (previousIdentity.current !== missionIdentity) {
      dismissedKeys.current.clear();
      previousIdentity.current = missionIdentity;
    }
    // HH_261002 - A manually opened idle map never replaces the next accepted mission.
    if (active) setIdleOpen(false);
    if (connected) {
      setOpenKey(key && !dismissedKeys.current.has(key) ? key : '');
    } else {
      setOpenKey(previous => previous === key ? previous : '');
    }
  }, [key, missionIdentity, connected, active]);

  const dismiss = useCallback(() => {
    if (key) dismissedKeys.current.add(key);
    setOpenKey('');
    setIdleOpen(false);
  }, [key]);

  const open = useCallback(() => {
    if (!canOpen) return;
    if (eligible && key) {
      dismissedKeys.current.delete(key);
      setOpenKey(key);
    } else if (idleEligible) {
      // HH_261002 - Opening the static map at home is an explicit display action,
      // not a dispatch, engagement, or automatic transition out of the home screen.
      setIdleOpen(true);
    }
  }, [canOpen, eligible, idleEligible, key]);

  useEffect(() => {
    setReceived(null);
    if (!visible || !connected) return undefined;
    if (hasInjectedSnapshot) {
      // HH_261001 - Local preview exercises exactly the same visibility/dismissal policy,
      // but cannot acquire a live telemetry transport under any circumstances.
      const timer = setInterval(() => setClock(now()), POLL_MS);
      return () => clearInterval(timer);
    }
    let disposed = false;
    let pollTimer = null;
    let timeoutTimer = null;
    let controller = null;
    const ageTimer = setInterval(() => setClock(now()), POLL_MS);
    const store = body => {
      if (disposed) return;
      const at = now();
      setClock(at);
      setReceived({ body, at, identity: displayIdentity });
    };
    const schedule = () => {
      // HH_261001 - Normal replies schedule the next GET after completion. A timed-out
      // request is aborted before scheduling its replacement.
      if (!disposed) pollTimer = setTimeout(poll, POLL_MS);
    };
    const poll = async () => {
      if (disposed) return;
      controller = new AbortController();
      const requestController = controller;
      let expired = false;
      timeoutTimer = setTimeout(() => {
        expired = true;
        requestController.abort();
        store(unavailable('timeout'));
        schedule();
      }, REQUEST_TIMEOUT_MS);
      try {
        const response = await fetch('/api/driving', {
          method: 'GET', cache: 'no-store', signal: requestController.signal,
        });
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const body = await response.json();
        if (disposed || expired) return;
        const identity = body?.mission;
        const matches = eligible ? identity?.active === true
          && identity.generation === generation && identity.site === site
          && identity.intent === intent : idleEligible && identity?.active === false;
        if (!matches) {
          store(unavailable('mission_mismatch'));
        } else if (body.connected === false) {
          store(unavailable('unavailable'));
        } else {
          store({ ...body, connected: true });
        }
      } catch (error) {
        if (!disposed && !expired) store(unavailable('unavailable'));
      } finally {
        if (!expired) {
          clearTimeout(timeoutTimer);
          schedule();
        }
      }
    };
    poll();
    return () => {
      disposed = true;
      clearInterval(ageTimer);
      clearTimeout(pollTimer);
      clearTimeout(timeoutTimer);
      controller?.abort();
    };
  }, [visible, connected, displayIdentity, eligible, idleEligible,
    generation, site, intent, hasInjectedSnapshot]);

  const snapshot = useMemo(() => {
    const injectedIdentity = injectedSnapshot?.mission;
    const injectionMatches = eligible ? injectedIdentity?.active === true
      && injectedIdentity.generation === generation && injectedIdentity.site === site
      && injectedIdentity.intent === intent : idleEligible && injectedIdentity?.active === false;
    // HH_261001 - Identity and visibility are checked before exposing any cached geometry.
    // Delayed replies from the previous mission cannot repaint the new one.
    const current = !visible || !connected ? null : hasInjectedSnapshot
      ? { body: injectionMatches ? injectedSnapshot : unavailable('mission_mismatch'), at: injectedAt }
      : received?.identity === displayIdentity ? received : null;
    const body = current?.body || unavailable(connected ? 'waiting' : 'disconnected');
    const elapsed = current ? Math.max(0, clock - current.at) / 1000 : 0;
    return {
      ...body,
      pose: advanceAge(body.pose, elapsed),
      route: advanceAge(body.route, elapsed),
      motion: advanceAge(body.motion, elapsed),
      wheel_telemetry: advanceAge(body.wheel_telemetry, elapsed),
      perception: advancePerceptionAge(body.perception, elapsed),
      mission: {
        ...(body.mission || {}),
        active, generation: active ? generation : 0, site: active ? site : '',
        intent: active ? intent : '', owner: active ? owner : '',
        service_state_name: serviceStateName,
        phase: missionPhase,
      },
    };
  }, [received, visible, displayIdentity, eligible, idleEligible, clock, connected, active, generation,
    site, intent, owner, serviceStateName, missionPhase, hasInjectedSnapshot, injectedSnapshot, injectedAt]);

  return { visible, canOpen, snapshot, dismiss, open };
}

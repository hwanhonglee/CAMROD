function createManagerMonitoring(selectTab, showLogin) {
  const $ = id => document.getElementById(id);
  const views = new Set(['overview', 'mission', 'vehicle', 'diagnostics']);
  const missionLabels = {
    DROP_ZONE_WAIT: '드랍존 대기', MOVING_TO_SITE: '사이트로 이동 중', SITE_ARRIVED: '사이트 도착',
    RETURNING_TO_DROP_ZONE: '드랍존 복귀 중', GUEST_RECALL_SERVICE: '호출 운행 중',
    SITE_ENTRY: '사이트 진입 중', UNLOAD_WAIT: '하차 대기', RECALL_TO_SITE_ROAD: '호출 지점으로 이동 중',
    GUEST_LOADING_WAIT: '적재 대기', RETURN_WITH_CARGO: '화물 싣고 복귀 중', DROP_ZONE_PARKING: '드랍존 주차 중',
    WAITING_FOR_RETURN_REQUEST: '복귀 요청 대기', WAITING_FOR_CHARGING: '충전 대기', CHARGING: '충전 중',
    DEPARTING_CHARGER: '충전기에서 출차 중', DEPARTING_DROP_ZONE: '드랍존에서 출차 중', OPERATOR_STOPPED: '운행 정지',
  };
  const intentLabels = { delivery: '배송', recall: '호출', return: '복귀' };
  const controlModes = ['대기', 'CAN · 자율', 'UART', 'RC · 수동'];
  const motionModes = ['듀얼 애커먼', '평행 주행', '제자리 회전', '횡슬립'];
  const diagnostics = $('monitorDiagnostics');
  const cameraSelect = $('monitorCameraSelect');
  const cameraButton = $('monitorCameraToggle');
  const cameraImage = $('monitorCameraImage');
  const clock = new Intl.DateTimeFormat('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit', hourCycle: 'h23' });
  let activeView = null;
  let polling = null;
  let timer;
  let cameraPlaying = false;
  let cameraUrl;
  let diagnosticRows = '';

  function text(id, value) { $(id).textContent = value ?? '—'; }

  function badge(id, label, tone = '') {
    const el = $(id);
    el.textContent = label;
    el.className = `monitor-badge ${tone}`;
  }

  function element(tag, value, className) {
    const node = document.createElement(tag);
    node.textContent = value;
    if (className) node.className = className;
    return node;
  }

  function measurement(value, decimals, unit) {
    return Number.isFinite(value) ? `${value.toFixed(decimals)}${unit}` : '—';
  }

  function missionPresentation(mission) {
    const returnsToDropZone = ['RETURNING_TO_DROP_ZONE', 'RETURN_WITH_CARGO', 'DROP_ZONE_PARKING'].includes(mission.state);
    return {
      label: missionLabels[mission.state] || mission.state || '수신 대기',
      destination: returnsToDropZone ? '드랍존' : mission.site || '—',
      intent: intentLabels[mission.intent] || mission.intent || '—',
    };
  }

  function vehiclePresentation(vehicle) {
    return {
      battery: measurement(vehicle.battery_pct, 0, '%'),
      speed: measurement((vehicle.motion?.speed_mps ?? NaN) * 3.6, 1, ' km/h'),
      control: controlModes[vehicle.control_mode] || '—',
      state: vehicle.estop ? '비상정지' : ['정상', '비상정지', '차대 이상'][vehicle.state] || '수신 대기',
      tone: vehicle.estop || vehicle.state > 0 ? 'error' : vehicle.state === 0 ? 'on' : '',
      estop: vehicle.estop === true ? '작동 중' : vehicle.estop === false ? '해제' : '—',
    };
  }

  function svgShape(tag, attrs) {
    const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
    Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
    return node;
  }

  function drawMap(data, destination) {
    const map = data.map || {};
    const roads = map.polylines || [];
    const areas = map.areas || [];
    const pose = data.pose || {};
    const poseValid = pose.frame_id === map.frame_id && [pose.x, pose.y].every(Number.isFinite);
    const route = data.route?.valid && data.route.frame_id === map.frame_id ? data.route.points : [];
    const footprint = areas.length ? areas.flatMap(area => area.points) : roads.flatMap(road => road.points);
    if (poseValid) footprint.push([pose.x, pose.y]);
    footprint.push(...route);
    const svg = $('monitorMap');
    $('mapEmpty').hidden = footprint.length > 0;
    text('mapGnss', data.sensors?.gnss?.label ? `GNSS · ${data.sensors.gnss.label}` : '');
    text('mapDestination', destination === '—' ? '' : destination);
    text('mapPosition', poseValid ? `위치 수신 ${measurement(pose.age_s, 1, '초 전')}` : '차량 위치 수신 대기');
    if (!footprint.length) { svg.replaceChildren(); return; }

    const xs = footprint.map(point => point[0]);
    const ys = footprint.map(point => point[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs);
    const minY = Math.min(...ys), maxY = Math.max(...ys);
    const scale = Math.min(584 / Math.max(maxX - minX, 1), 304 / Math.max(maxY - minY, 1));
    const midX = (minX + maxX) / 2, midY = (minY + maxY) / 2;
    const project = ([x, y]) => [320 + (x - midX) * scale, 180 - (y - midY) * scale];
    const worldPoints = points => points.map(point => project(point).map(value => value.toFixed(1)).join(',')).join(' ');
    const shapes = document.createDocumentFragment();
    roads.forEach(road => {
      shapes.append(svgShape('polyline', { class: road.namespace === 'lanelet/centerline' ? 'map-centerline' : 'map-road', points: worldPoints(road.points) }));
    });
    areas.forEach(area => {
      const goal = destination === '드랍존' ? area.id === 'drop_zone' : area.site === destination;
      shapes.append(svgShape('polygon', { class: goal ? 'map-goal' : 'map-site', points: worldPoints(area.points) }));
    });
    if (route.length) shapes.append(svgShape('polyline', { class: 'map-route', points: worldPoints(route) }));
    if (poseValid) {
      const [cx, cy] = project([pose.x, pose.y]);
      shapes.append(svgShape('circle', { class: 'map-vehicle', cx, cy, r: 6 }));
    }
    svg.replaceChildren(shapes);
  }

  function renderOverview(data) {
    const mission = data.mission || {};
    const current = missionPresentation(mission);
    const vehicle = vehiclePresentation(data.vehicle || {});
    const health = data.health;
    const healthLabels = { OK: '시스템 정상', NORMAL: '시스템 정상', WARN: '시스템 경고', WARNING: '시스템 경고', ERROR: '시스템 오류', STARTING: '시스템 시작 중' };
    const healthTones = { OK: 'on', NORMAL: 'on', WARN: 'warn', WARNING: 'warn', ERROR: 'error' };
    badge('monitorHealth', healthLabels[health] || health || '수신 대기', healthTones[health]);
    text('monitorUpdated', health ? `갱신 ${clock.format(new Date())}` : '');
    const alert = $('monitorAlert');
    alert.hidden = !data.alert_count;
    alert.textContent = `진단 경고·오류 ${data.alert_count || 0}건 · 상세 보기`;
    badge('overviewMissionState', current.label, mission.active ? 'on' : '');
    text('overviewDestination', current.destination === '—' ? '지정된 목적지 없음' : current.destination);
    text('overviewMissionPhase', mission.description || mission.phase || '');
    badge('overviewVehicleState', vehicle.state, vehicle.tone);
    text('overviewBattery', vehicle.battery);
    text('overviewSpeed', vehicle.speed);
    text('overviewControl', vehicle.control);
    drawMap(data, current.destination);
  }

  function renderMission(data) {
    const mission = data.mission || {};
    const current = missionPresentation(mission);
    badge('missionStatus', current.label, mission.active ? 'on' : '');
    text('missionDestination', `목적지 ${current.destination}`);
    text('missionPhase', mission.phase);
    text('missionIntent', current.intent);
    text('missionEstop', vehiclePresentation(data.vehicle || {}).estop);
    const progress = data.progress?.valid ? data.progress : {};
    text('missionDistance', measurement(progress.remaining_distance_m, 0, ' m'));
    text('missionTime', measurement(progress.remaining_time_s, 0, '초'));
  }

  function renderVehicle(data) {
    const raw = data.vehicle || {};
    const vehicle = vehiclePresentation(raw);
    badge('vehicleStatus', vehicle.state, vehicle.tone);
    text('vehicleBattery', vehicle.battery);
    text('vehicleCharging', raw.charging === true ? '충전 중' : raw.charging === false ? '충전 중 아님' : '');
    text('vehicleSpeed', vehicle.speed);
    text('vehicleControl', vehicle.control);
    text('vehicleMotion', motionModes[raw.motion_mode]);
    text('vehicleError', Number.isInteger(raw.error_code) ? `0x${raw.error_code.toString(16).padStart(4, '0').toUpperCase()}` : '—');
    text('vehicleEstop', vehicle.estop);
    text('vehicleAge', measurement(raw.age_s, 1, '초 전'));
  }

  function renderDiagnostics(data) {
    const rows = data.diagnostics || [];
    badge('diagnosticsCount', rows.length ? `경고·오류 ${data.alert_count || 0}건` : '수신 대기', data.alert_count ? 'warn' : '');
    const signature = JSON.stringify(rows);
    if (signature === diagnosticRows) return;
    diagnosticRows = signature;
    const expanded = new Set([...diagnostics.querySelectorAll('details[open]')].map(row => row.dataset.name));
    const focused = diagnostics.contains(document.activeElement) ? document.activeElement.closest('details')?.dataset.name : null;
    const focusedCamera = document.activeElement.matches('.diagnostic-camera');
    const items = [...rows].sort((a, b) => b.level - a.level).map(item => {
      const row = element('details', '', 'diagnostic-row');
      row.dataset.name = item.name;
      row.open = expanded.has(item.name);
      const summary = element('summary', '');
      const severity = item.level >= 2 ? '오류' : item.level === 1 ? '경고' : '정상';
      const tone = item.level >= 2 ? 'error' : item.level === 1 ? 'warn' : 'on';
      summary.append(element('span', severity, `monitor-badge ${tone}`), document.createTextNode(` ${item.name}`));
      row.append(summary, element('p', item.message || '—', 'diagnostic-note'));
      if (item.values?.length) {
        const facts = element('dl', '', 'monitor-facts');
        item.values.forEach(({ key, value }) => {
          const fact = element('div', '');
          fact.append(element('dt', key), element('dd', value));
          facts.append(fact);
        });
        row.append(facts);
      }
      const source = `${item.module || ''} ${item.name}`.toLowerCase();
      if (/camera|카메라|econ_(front|rear)/.test(source)) {
        const link = element('button', '카메라 열기', 'btn btn-ghost monitor-button diagnostic-camera');
        link.type = 'button';
        link.addEventListener('click', () => {
          if (/rear|후방/.test(source)) cameraSelect.value = 'rear';
          else if (/front|전방/.test(source)) cameraSelect.value = 'front';
          jump('camera');
        });
        row.append(link);
      }
      return row;
    });
    diagnostics.replaceChildren(...(items.length ? items : [element('p', '진단 데이터 수신 대기', 'diagnostic-note monitor-meta')]));
    if (focused) items.find(row => row.dataset.name === focused)
      ?.querySelector(focusedCamera ? '.diagnostic-camera' : 'summary')?.focus({ preventScroll: true });
  }

  const renderers = { overview: renderOverview, mission: renderMission, vehicle: renderVehicle, diagnostics: renderDiagnostics };

  function notice(message) {
    $('monitorNotice').hidden = !message;
    text('monitorNotice', message);
  }

  function stopPolling() {
    clearTimeout(timer);
    polling?.abort();
    polling = null;
  }

  function startPolling(task, interval) {
    polling = new AbortController();
    const signal = polling.signal;
    async function refresh() {
      await task(signal);
      if (!signal.aborted) timer = setTimeout(refresh, interval);
    }
    refresh();
  }

  function request(path, signal) {
    return fetch(path, { credentials: 'same-origin', cache: 'no-store', signal });
  }

  async function refreshSnapshot(view, signal) {
    try {
      const response = await request(`/api/monitoring/${view}`, signal);
      if (signal.aborted) return;
      if (response.status === 401) { showLogin(); return; }
      if (!response.ok) throw new Error('unavailable');
      const data = await response.json();
      if (signal.aborted) return;
      renderers[view](data);
      notice('');
    } catch (error) {
      if (signal.aborted) return;
      renderers[view]({});
      if (view === 'overview') badge('monitorHealth', '연결 끊김', 'error');
      notice('로봇 상태를 가져오지 못했습니다. 다시 연결 중입니다.');
    }
  }

  function clearCameraFrame(message) {
    if (cameraUrl) URL.revokeObjectURL(cameraUrl);
    cameraUrl = undefined;
    cameraImage.hidden = true;
    cameraImage.removeAttribute('src');
    $('monitorCameraMessage').hidden = false;
    text('monitorCameraMessage', message);
    text('monitorCameraSize', '—');
    text('monitorCameraTime', '—');
  }

  function resetCamera() {
    cameraPlaying = false;
    cameraButton.textContent = '영상 보기';
    badge('cameraStatus', '대기');
    clearCameraFrame('영상 보기를 누르면 카메라에 연결합니다.');
  }

  async function refreshCamera(camera, signal) {
    try {
      const response = await request(`/api/monitoring/camera/${camera}`, signal);
      if (signal.aborted) return;
      if (response.status === 401) { showLogin(); return; }
      if (!response.ok || response.status === 204) throw new Error('frame unavailable');
      const frame = await response.blob();
      if (signal.aborted) return;
      if (cameraUrl) URL.revokeObjectURL(cameraUrl);
      cameraUrl = URL.createObjectURL(frame);
      cameraImage.src = cameraUrl;
      cameraImage.alt = `${camera === 'front' ? '전방' : '후방'} 카메라 영상`;
      cameraImage.hidden = false;
      $('monitorCameraMessage').hidden = true;
      badge('cameraStatus', '수신 중', 'on');
      text('monitorCameraTime', clock.format(new Date()));
    } catch (error) {
      if (signal.aborted) return;
      clearCameraFrame('새 프레임을 기다리는 중입니다.');
      badge('cameraStatus', '수신 대기', 'warn');
    }
  }

  function playCamera() {
    stopPolling();
    cameraPlaying = true;
    cameraButton.textContent = '정지';
    badge('cameraStatus', '연결 중');
    clearCameraFrame('카메라에 연결하는 중입니다.');
    const camera = cameraSelect.value;
    startPolling(signal => refreshCamera(camera, signal), 500);
  }

  function activate(view) {
    stopPolling();
    resetCamera();
    notice('');
    activeView = view;
    if (views.has(view) && !document.hidden) startPolling(signal => refreshSnapshot(view, signal), 2000);
  }

  function jump(view) {
    selectTab(view);
    document.querySelector(`nav [data-tab="${view}"]`).focus({ preventScroll: true });
  }

  $('monitorAlert').addEventListener('click', () => jump('diagnostics'));
  document.querySelectorAll('[data-monitor-link]').forEach(button => {
    button.addEventListener('click', () => jump(button.dataset.monitorLink));
  });
  cameraButton.addEventListener('click', () => {
    if (!cameraPlaying) { playCamera(); return; }
    stopPolling();
    resetCamera();
  });
  cameraSelect.addEventListener('change', () => { if (cameraPlaying) playCamera(); });
  cameraImage.addEventListener('load', () => text('monitorCameraSize', `${cameraImage.naturalWidth} × ${cameraImage.naturalHeight}`));
  document.addEventListener('visibilitychange', () => activate(activeView));
  return { activate };
}

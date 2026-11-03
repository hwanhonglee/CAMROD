/* HH_261102 - Render the production UI against loopback fixtures only; this is not a robot/CARLA test. */
const http = require('http');
const fs = require('fs');
const path = require('path');
const os = require('os');
const crypto = require('crypto');
const { spawn } = require('child_process');
const repo = path.resolve(__dirname, '../../../..');
const frontend = path.join(repo, 'camrod_ui/camrod_ui_robot/assets/frontend');
const WS = require(path.join(frontend, 'node_modules/ws'));
const build = path.join(frontend, 'build');
const initial = { phase: 'inactive', message: '충전 위치에서 대기 중입니다. 짐 작업 전 대기 위치로 이동해 주세요.',
  can_leave: false, cargo_access_ready: false, battery_percentage: 34,
  minimum_battery_percentage: 35, platform_is_charging: true, platform_feedback_fresh: true,
  electrical_stop_available: false, physical_disconnect_verified: false, target_label: 'drop_zone 내 대기 위치' };
// HH_261102 - Phone screenshots use the real Guest page and a separate idle WebSocket snapshot.
const guestFrame = { sites: Array.from({ length: 13 }, (_, i) => `B${i + 1}`), identity_revision: 1,
  phase: 'idle', service_state: 0, service_state_name: 'DROP_ZONE_WAIT', site: '', request_intent: '',
  request_owner: '', battery: 80, minimum_battery_percentage: 35, mission_battery_ready: true,
  safety_hold: false, mission_execution_error: '' };

(async () => {
  let snapshot = { ...initial };
  let guestMode = false;
  const commands = [];
  const frames = [];
  const server = http.createServer((req, res) => {
    const url = new URL(req.url, 'http://localhost');
    const json = body => { res.writeHead(200, { 'Content-Type': 'application/json', 'Cache-Control': 'no-store' }); res.end(JSON.stringify(body)); };
    if (url.pathname === '/api/charging-access') return json(snapshot);
    if (url.pathname === '/api/service-metrics/summary') return json({ available: true, current_service: null });
    if (url.pathname === '/ui/charging_access' && req.method === 'POST') {
      const action = url.searchParams.get('action');
      commands.push({ action, confirm: url.searchParams.get('confirm') });
      if (action !== 'leave' || url.searchParams.get('confirm') !== 'true') {
        res.writeHead(400, { 'Content-Type': 'application/json' }); return res.end(JSON.stringify({ success: false }));
      }
      // HH_261102 - The fixture now models only the requested contact-clearance move and stopped staging state.
      snapshot = { ...snapshot, phase: 'departing', can_leave: false, cargo_access_ready: false,
        message: 'drop_zone 내 대기 위치로 이동 중입니다. 짐과 충전 접점에서 손을 떼고 로봇 주변을 비워 주세요.' };
      return json({ ...snapshot, success: true });
    }
    if (url.pathname === '/ui/stop') { commands.push({ action: 'stop' }); snapshot = { ...snapshot, phase: 'inactive' }; return json({ success: true }); }
    if (url.pathname === '/guest/') {
      res.writeHead(200, { 'Content-Type': 'text/html' });
      return fs.createReadStream(path.join(repo, 'camrod_ui/camrod_ui_guest/assets/guest_frontend/index.html')).pipe(res);
    }
    let file = path.join(build, decodeURIComponent(url.pathname));
    if (!file.startsWith(build + path.sep) || !fs.existsSync(file) || !fs.statSync(file).isFile()) file = path.join(build, 'index.html');
    const mime = { '.html': 'text/html', '.js': 'application/javascript', '.css': 'text/css', '.png': 'image/png', '.jpg': 'image/jpeg', '.glb': 'model/gltf-binary' }[path.extname(file)] || 'application/octet-stream';
    res.writeHead(200, { 'Content-Type': mime }); fs.createReadStream(file).pipe(res);
  });
  const wss = new WS.Server({ server });
  wss.on('connection', socket => socket.send(JSON.stringify(guestMode ? guestFrame : { states: {}, battery: 34, service_state: 13,
    service_state_name: 'CHARGING', mission_dispatch_active: false, mission_dispatch_generation: 0,
    system_health: 'OK', mission_phase: 'READY', engage: false })));
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  const origin = `http://127.0.0.1:${server.address().port}`;
  const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'camrod-charging-ui-'));
  const chrome = spawn('/usr/bin/google-chrome', ['--headless=new', '--no-sandbox', '--disable-dev-shm-usage',
    '--disable-gpu', '--remote-debugging-port=0', `--user-data-dir=${profile}`, 'about:blank'], { stdio: ['ignore', 'ignore', 'pipe'] });
  const endpoint = await new Promise((resolve, reject) => {
    let log = '';
    chrome.stderr.on('data', bytes => { log += bytes; const match = log.match(/DevTools listening on (ws:\/\/[^\s]+)/); if (match) resolve(match[1]); });
    chrome.on('error', reject); setTimeout(() => reject(new Error('Chrome DevTools startup timeout')), 10000).unref();
  });
  const socket = new WS(endpoint);
  await new Promise(resolve => socket.once('open', resolve));
  let id = 0;
  const waiting = new Map();
  socket.on('message', raw => { const data = JSON.parse(raw); if (data.id) { const pending = waiting.get(data.id); waiting.delete(data.id);
    if (data.error) pending.reject(new Error(JSON.stringify(data.error))); else pending.resolve(data.result); } });
  const send = (method, params = {}, sessionId) => new Promise((resolve, reject) => {
    const next = ++id; waiting.set(next, { resolve, reject });
    socket.send(JSON.stringify({ id: next, method, params, ...(sessionId ? { sessionId } : {}) }));
  });
  const { targetId } = await send('Target.createTarget', { url: 'about:blank' });
  const { sessionId } = await send('Target.attachToTarget', { targetId, flatten: true });
  const cdp = (method, params) => send(method, params, sessionId);
  const evaluate = async expression => (await cdp('Runtime.evaluate', { expression, returnByValue: true, awaitPromise: true })).result.value;
  const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
  const until = async expression => { for (let i = 0; i < 80; i++) { if (await evaluate(`Boolean(${expression})`)) return; await wait(100); } throw new Error(`UI wait failed: ${expression}`); };
  const click = key => evaluate(`document.querySelector('[data-ui="${key}"]').click()`);
  const shot = async name => {
    await wait(250);
    const stopVisible = await evaluate(`(() => { const stop = document.querySelector('[data-ui="charging-safety-stop"]'); if (!stop) return null; const r = stop.getBoundingClientRect(); return r.top >= 0 && r.bottom <= innerHeight; })()`);
    if (stopVisible === false) throw new Error('Public stop is outside the visible viewport');
    const { data } = await cdp('Page.captureScreenshot', { format: 'png' });
    fs.writeFileSync(path.join(__dirname, `${name}.png`), Buffer.from(data, 'base64'));
    frames.push({ name, snapshot: { ...(guestMode ? guestFrame : snapshot) }, public_stop_visible: stopVisible });
  };
  try {
    await cdp('Page.enable'); await cdp('Runtime.enable');
    await cdp('Emulation.setDeviceMetricsOverride', { width: 1280, height: 800, deviceScaleFactor: 1, mobile: false });
    await cdp('Page.navigate', { url: origin });
    await until(`document.querySelector('[data-ui="charging-access-leave"]') && document.querySelector('[data-ui="charging-contact-state"]').textContent.includes('충전 중')`);
    await evaluate(`document.body.insertAdjacentHTML('beforeend','<div style="position:fixed;bottom:4px;left:6px;z-index:99999;padding:5px 10px;background:#333;color:white;font:12px sans-serif;pointer-events:none">UI FIXTURE · NO ROBOT / NO CARLA</div>')`);
    await shot('01_soc34_blocked');
    snapshot = { ...snapshot, battery_percentage: 35, can_leave: true };
    // HH_261102 - Keep the existing menu's WebSocket status consistent with the charging API fixture.
    wss.clients.forEach(client => client.send(JSON.stringify({ battery: 35 })));
    await until(`!document.querySelector('[data-ui="charging-access-leave"]').disabled`); await shot('02_soc35_eligible');
    await click('charging-access-leave'); await until(`document.querySelector('[data-ui="charging-confirm-move"]')`); await shot('03_leave_confirmation');
    if (commands.length) throw new Error('An unconfirmed UI issued a command');
    await click('charging-confirm-move'); await until(`document.querySelector('[data-ui="charging-screen"]').dataset.phase === 'departing'`); await shot('04_departing');
    snapshot = { ...snapshot, phase: 'ready', cargo_access_ready: true, platform_is_charging: false,
      message: 'drop_zone 내 대기 위치에 정차했습니다. 짐을 꺼내실 수 있습니다.' };
    wss.clients.forEach(client => client.send(JSON.stringify({ battery: 35, service_state: 0, service_state_name: 'DROP_ZONE_WAIT' })));
    await until(`document.querySelector('[data-ui="charging-screen"]').dataset.phase === 'ready' && !document.querySelector('[data-ui="charging-open-services"]').disabled`); await shot('05_ready_cargo_wait');
    await wait(2200); if (commands.length !== 1) throw new Error('Cargo wait unexpectedly dispatched');
    await click('charging-open-services'); await until(`document.querySelector('[data-ui="usage-safety-confirm"]')`);
    await click('usage-safety-confirm'); await until(`document.querySelector('[data-ui="operator-service-selection-screen"]')`); await shot('06_existing_service_menu');
    // HH_261102 - Capture concise service copy without bypassing the original service confirmation.
    await click('operator-intent-delivery'); await until(`document.querySelector('[data-ui="operator-service-delivery-confirm"]')`); await shot('07_delivery_confirmation');
    await evaluate(`document.querySelector('.move-confirm-no').click()`);
    await evaluate(`document.querySelector('.service-selection-back').click()`); await until(`document.querySelector('[data-ui="charging-screen"]')`);
    snapshot = { ...snapshot, cargo_access_ready: false, platform_feedback_fresh: false };
    await until(`document.querySelector('[data-ui="charging-screen"]') && document.querySelector('[data-ui="charging-contact-state"]').textContent.includes('확인 중')`); await shot('08_stale_vehicle_feedback');
    await click('charging-safety-stop'); await until(`document.querySelector('[data-ui="charging-confirm-stop"]')`); await shot('09_stop_confirmation_unavailable');
    await click('charging-confirm-cancel');
    if (JSON.stringify(commands) !== JSON.stringify([{ action: 'leave', confirm: 'true' }])) throw new Error('Unexpected fixture commands');
    guestMode = true;
    await cdp('Emulation.setDeviceMetricsOverride', { width: 430, height: 932, deviceScaleFactor: 1, mobile: true });
    await cdp('Page.navigate', { url: `${origin}/guest/` });
    await until(`document.querySelectorAll('#siteGrid .site-btn').length === 13 && document.querySelector('#siteCard').style.display !== 'none'`);
    await evaluate(`document.body.insertAdjacentHTML('beforeend','<div style="position:fixed;bottom:4px;left:6px;z-index:99999;padding:5px 10px;background:#333;color:white;font:11px sans-serif;pointer-events:none">UI FIXTURE · NO ROBOT / NO CARLA</div>')`);
    await shot('10_guest_phone');
    await evaluate(`document.querySelector('.site-btn[data-site="B2"]').click()`);
    await until(`!document.querySelector('#navigateBtn').disabled && document.querySelector('#navigateBtn').textContent.includes('B2로 호출')`);
    await evaluate(`document.querySelector('#navigateBtn').click()`);
    await until(`document.querySelector('#confirmOverlay').classList.contains('show')`);
    await shot('11_guest_confirmation');
    const sourceSha256 = {};
    for (const name of ['App.js', 'ChargingAccessScreen.js', 'ChargingAccessScreen.css']) sourceSha256[name] = crypto.createHash('sha256').update(fs.readFileSync(path.join(frontend, 'src', name))).digest('hex');
    sourceSha256['guest/index.html'] = crypto.createHash('sha256').update(fs.readFileSync(path.join(repo, 'camrod_ui/camrod_ui_guest/assets/guest_frontend/index.html'))).digest('hex');
    fs.writeFileSync(path.join(__dirname, 'manifest.json'), JSON.stringify({ mode: 'LOOPBACK UI FIXTURE — NO ROBOT / NO CARLA', generated_at: new Date().toISOString(), viewport: { robot: '1280x800', guest: '430x932' },
      commands, frames, source_sha256: sourceSha256, build_assets: JSON.parse(fs.readFileSync(path.join(build, 'asset-manifest.json'), 'utf8')) }, null, 2) + '\n');
    console.log(JSON.stringify({ output: __dirname, frames: frames.length, commands }));
  } finally {
    socket.close();
    // HH_261102 - Remove only this fixture's freshly-created Chrome profile after its own browser exits.
    const exited = new Promise(resolve => chrome.once('exit', resolve));
    chrome.kill('SIGTERM'); await exited;
    wss.clients.forEach(client => client.terminate()); wss.close();
    await new Promise(resolve => server.close(resolve));
    if (profile.startsWith(path.join(os.tmpdir(), 'camrod-charging-ui-'))) fs.rmSync(profile, { recursive: true, force: true, maxRetries: 10, retryDelay: 100 });
  }
})().catch(error => { console.error(error); process.exitCode = 1; });

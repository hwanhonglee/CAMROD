const $ = (id) => document.getElementById(id);
let latest = null;
let socket = null;

const phaseLabels = {
  ready: "대기",
  move_to_site: "사이트 이동",
  site_arrived: "사이트 도착",
  return_to_drop_zone: "드랍존 복귀",
  drop_zone_return_complete: "드랍존 복귀 완료",
  docking_start: "도킹 시작",
  docking_complete: "도킹 완료",
};

async function call(path, options = {}) {
  const response = await fetch(path, options);
  if (!response.ok) {
    const body = await response.json().catch(() => ({detail: response.statusText}));
    throw new Error(body.detail || response.statusText);
  }
  return response.json();
}

function render(state) {
  latest = state;
  $("service-state").textContent = state.service_state;
  $("planning-state").textContent = state.planning_state;
  $("gate-state").textContent = state.gate_state;
  $("battery-value").textContent = `${state.battery_percent.toFixed(1)}%`;
  $("pose-value").textContent = `${state.pose.x.toFixed(1)}, ${state.pose.y.toFixed(1)}`;
  $("progress-value").textContent = `${Math.round(state.route_progress * 100)}%`;
  $("site-value").textContent = state.active_site || "—";
  $("workflow-value").textContent = state.workflow_type === "recall"
    ? "리콜 서비스"
    : state.workflow_type === "delivery" ? "일반 서비스" : "—";
  $("phase-value").textContent = phaseLabels[state.workflow_phase] || state.workflow_phase || "—";
  $("scenario").textContent = `${state.mode} · ${state.scenario}`;
  $("progress-bar").style.width = `${Math.round(state.route_progress * 100)}%`;
  $("battery").value = Math.round(state.battery_percent);
  $("battery-output").textContent = `${Math.round(state.battery_percent)}%`;
  $("pause").textContent = state.paused ? "Resume" : "Pause";

  document.querySelectorAll("[data-mode]").forEach((button) => {
    button.classList.toggle("active", button.dataset.mode === state.mode);
  });
  document.querySelectorAll("[data-speed]").forEach((button) => {
    button.classList.toggle("active", Number(button.dataset.speed) === Number(state.speed_scale));
  });
  document.querySelectorAll("[data-toggle]").forEach((button) => {
    button.classList.toggle("active", Boolean(state[button.dataset.toggle]));
  });
  document.querySelectorAll("[data-workflow][data-phase]").forEach((button) => {
    button.classList.toggle(
      "active",
      button.dataset.workflow === state.workflow_type
        && button.dataset.phase === state.workflow_phase,
    );
  });
  document.querySelectorAll("[data-workflow-card]").forEach((card) => {
    card.classList.toggle("active", card.dataset.workflowCard === state.workflow_type);
  });

  const sitePicker = $("workflow-site");
  if (sitePicker.options.length === 0 && Array.isArray(state.available_sites)) {
    const sites = [...state.available_sites].sort((a, b) => Number(a.slice(1)) - Number(b.slice(1)));
    sitePicker.innerHTML = sites.map((site) => `<option value="${escapeHtml(site)}">${escapeHtml(site)}</option>`).join("");
    if (state.active_site) sitePicker.value = state.active_site;
  }
  const container = $("events");
  const events = [...(state.events || [])].reverse();
  container.innerHTML = events.map((event) => `
    <div class="event">
      <time>${event.wall_time}</time>
      <span class="topic">${escapeHtml(event.topic)}</span>
      <span class="summary">${escapeHtml(event.summary)}</span>
    </div>`).join("");
}

function escapeHtml(value) {
  return String(value).replace(/[&<>'"]/g, (char) => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;"
  })[char]);
}

function connect() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${protocol}://${location.host}/ws/sim`);
  socket.onopen = () => {
    $("connection").textContent = "실시간 연결";
    $("connection-dot").classList.add("online");
  };
  socket.onmessage = (event) => render(JSON.parse(event.data));
  socket.onclose = () => {
    $("connection").textContent = "재연결 중";
    $("connection-dot").classList.remove("online");
    setTimeout(connect, 1000);
  };
}

document.querySelectorAll("[data-mode]").forEach((button) => {
  button.addEventListener("click", () => call(`/api/sim/mode/${button.dataset.mode}`, {method: "POST"}));
});
document.querySelectorAll("[data-speed]").forEach((button) => {
  button.addEventListener("click", () => call(`/api/sim/speed/${button.dataset.speed}`, {method: "POST"}));
});
document.querySelectorAll("[data-scenario]").forEach((button) => {
  button.addEventListener("click", () => call(`/api/sim/scenario/${button.dataset.scenario}`, {method: "POST"}));
});
document.querySelectorAll("[data-signal]").forEach((button) => {
  button.addEventListener("click", () => call(`/api/sim/signal/${button.dataset.signal}`, {method: "POST"}));
});
document.querySelectorAll("[data-toggle]").forEach((button) => {
  button.addEventListener("click", () => {
    const field = button.dataset.toggle;
    return call("/api/sim/state", {
      method: "PATCH",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({[field]: !Boolean(latest && latest[field])}),
    });
  });
});

document.querySelectorAll("[data-workflow][data-phase]").forEach((button) => {
  button.addEventListener("click", async () => {
    const site = $("workflow-site").value;
    const workflow = button.dataset.workflow;
    const phase = button.dataset.phase;
    const message = $("workflow-message");
    message.classList.remove("error");
    message.textContent = `${site} · ${phaseLabels[phase]} 적용 중…`;
    try {
      await call(`/api/sim/workflow/${workflow}/${phase}`, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({site}),
      });
      message.textContent = `${site} · ${workflow === "recall" ? "리콜" : "일반"} · ${phaseLabels[phase]}`;
    } catch (error) {
      message.classList.add("error");
      message.textContent = error.message;
    }
  });
});

$("battery").addEventListener("input", (event) => {
  $("battery-output").textContent = `${event.target.value}%`;
});
$("battery").addEventListener("change", (event) => call("/api/sim/state", {
  method: "PATCH",
  headers: {"Content-Type": "application/json"},
  body: JSON.stringify({battery_percent: Number(event.target.value)}),
}));
$("pause").addEventListener("click", () => call("/api/sim/state", {
  method: "PATCH",
  headers: {"Content-Type": "application/json"},
  body: JSON.stringify({paused: !(latest && latest.paused)}),
}));
$("reset").addEventListener("click", () => call("/api/sim/reset", {method: "POST"}));

connect();

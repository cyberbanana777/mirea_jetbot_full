// jetbot_mirea · веб-настройщик ПИД
// Без внешних библиотек: страница должна работать на роботе без интернета.
'use strict';

const SIGNALS = ['v', 'w', 'vl', 'vr'];
const HISTORY_S = 35;            // сколько секунд держать в памяти браузера
const HEARTBEAT_MS = 250;
const PID_KEYS = ['kp_l', 'ki_l', 'kd_l', 'kp_r', 'ki_r', 'kd_r'];

const $ = (id) => document.getElementById(id);
const state = {
  config: null,
  serverT: 0,            // время узла из последней телеметрии, с
  lastMsgAt: 0,          // performance.now() последнего сообщения
  data: Object.fromEntries(SIGNALS.map((s) => [s, []])),   // [t, value]
  sp: Object.fromEntries(SIGNALS.map((s) => [s, []])),     // [t, setpoint] - ступенчатая функция
  pid: null,
  status: null,
  paused: false,
  driveMode: 'base',
  stepMode: 'wheels',
  stepState: 'idle',
  dirty: true,
};

// ================================================================ сеть
async function api(name, body = {}) {
  try {
    const r = await fetch('/api/' + name, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(j.error || r.statusText);
    return j;
  } catch (e) {
    toast(e.message || String(e), true);
    return null;
  }
}

function stopNow(reason) {
  // sendBeacon доходит даже при закрытии вкладки
  const body = new Blob([JSON.stringify({ reason })], { type: 'application/json' });
  if (!navigator.sendBeacon || !navigator.sendBeacon('/api/stop', body)) {
    fetch('/api/stop', { method: 'POST', body: JSON.stringify({ reason }),
      headers: { 'Content-Type': 'application/json' }, keepalive: true }).catch(() => {});
  }
}

function connect() {
  const es = new EventSource('/events');
  es.addEventListener('config', (e) => onConfig(JSON.parse(e.data)));
  es.addEventListener('telemetry', (e) => onTelemetry(JSON.parse(e.data)));
  es.onerror = () => setPill('st-link', 'нет связи', 'bad');
}

setInterval(() => {
  if (document.visibilityState === 'visible') {
    fetch('/api/heartbeat', { method: 'POST', body: '{}',
      headers: { 'Content-Type': 'application/json' } }).catch(() => {});
  }
  if (performance.now() - state.lastMsgAt > 1500) setPill('st-link', 'нет связи', 'bad');
}, HEARTBEAT_MS);

// Свернули или закрыли вкладку - робот должен остановиться
document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'hidden' && state.status && state.status.command) {
    stopNow('вкладка скрыта');
  }
});
window.addEventListener('pagehide', () => stopNow('страница закрыта'));

document.addEventListener('keydown', (e) => {
  if (e.code === 'Space' || e.key === 'Escape') {
    e.preventDefault();
    api('stop', { reason: 'клавиша ' + (e.code === 'Space' ? 'пробел' : 'Esc') });
  }
});
$('btn-stop').addEventListener('click', () => api('stop', { reason: 'кнопка СТОП' }));

// ================================================================ входящие данные
function onConfig(cfg) {
  state.config = cfg;
  updateDriveLabels();
  updateStepLabels();
  const st = cfg.step_test;
  $('step-duration').max = st.max_duration;
  $('step-duration').value = st.default_duration;
  if (cfg.pid) { state.pid = cfg.pid; showCurrentPid(); fillPidIfEmpty(); }
  // кадры одометрии узел шлёт в лог, только если включён log_odometry_frames
  $('log-odom-box').classList.toggle('hidden', !cfg.log_odometry_frames);
}

function onTelemetry(msg) {
  state.lastMsgAt = performance.now();
  state.serverT = msg.t;
  setPill('st-link', 'связь есть', 'ok');

  // измерения - сигналы v, w, vl, vr; уставки - sp_v, sp_w, sp_vl, sp_vr
  for (const [t, signal, value] of msg.samples) {
    if (signal.startsWith('sp_')) {
      const arr = state.sp[signal.slice(3)];
      if (arr) arr.push([t, value]);
    } else if (state.data[signal]) {
      state.data[signal].push([t, value]);
    }
  }
  trimHistory(msg.t);

  if (msg.log.length) appendLog(msg.log);
  if (msg.pid) { state.pid = msg.pid; showCurrentPid(); fillPidIfEmpty(); }
  state.status = msg.status;
  updateStatus(msg.status);
  updateStep(msg.status.step);
  state.dirty = true;
}

function trimHistory(now) {
  const from = now - HISTORY_S;
  for (const s of SIGNALS) {
    const d = state.data[s];
    let i = 0;
    while (i < d.length && d[i][0] < from) i++;
    if (i) d.splice(0, i);
    const p = state.sp[s];
    // для уставки оставляем одну точку до окна: она задаёт значение в начале
    let j = 0;
    while (j + 1 < p.length && p[j + 1][0] < from) j++;
    if (j) p.splice(0, j);
  }
}

// ================================================================ статус
function setPill(id, text, cls) {
  const el = $(id);
  el.textContent = text;
  el.className = 'pill' + (cls ? ' ' + cls : '');
}

function fmt(x, digits = 3) {
  return (x === null || x === undefined || Number.isNaN(x)) ? '—' : Number(x).toFixed(digits);
}

function updateStatus(st) {
  const age = st.frame_age;
  if (age === null || age > 1.0) setPill('st-esp', 'ESP32: нет данных', 'bad');
  else if (st.esp_rate === 0) setPill('st-esp', 'ESP32: подключение…', 'warn');
  else setPill('st-esp', `ESP32: ${st.esp_rate.toFixed(0)} Гц`, st.esp_rate >= 40 ? 'ok' : 'warn');

  if (st.command) {
    const c = st.command;
    const txt = c.mode === 'base'
      ? `v=${fmt(c.a, 2)} м/с, ω=${fmt(c.b, 2)} рад/с`
      : `колёса ${fmt(c.a, 2)} / ${fmt(c.b, 2)} м/с`;
    setPill('st-cmd', 'едет: ' + txt, 'warn');
  } else {
    setPill('st-cmd', 'команда: нет', '');
  }

  const pubs = st.cmd_vel_publishers;
  const w = $('st-cmdvel');
  if (pubs > 0) {
    w.textContent = `/cmd_vel: ${pubs} издател${pubs === 1 ? 'ь' : 'я'} — Nav2/teleop будут перебивать команды`;
    w.classList.remove('hidden');
  } else {
    w.classList.add('hidden');
  }

  const last = (s) => { const d = state.data[s]; return d.length ? d[d.length - 1][1] : null; };
  $('drive-readout').textContent =
    `измерено: v=${fmt(last('v'))} м/с  ω=${fmt(last('w'))} рад/с  ` +
    `левое=${fmt(last('vl'))} м/с  правое=${fmt(last('vr'))} м/с`;
}

// ================================================================ вкладки
document.querySelectorAll('.tab').forEach((tab) => tab.addEventListener('click', () => {
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t === tab));
  document.querySelectorAll('.tab-body').forEach((b) =>
    b.classList.toggle('hidden', b.id !== 'tab-' + tab.dataset.tab));
  state.dirty = true;
}));

function segmented(id, onChange) {
  const root = $(id);
  root.querySelectorAll('.seg').forEach((btn) => btn.addEventListener('click', () => {
    root.querySelectorAll('.seg').forEach((b) => b.classList.toggle('active', b === btn));
    onChange(btn.dataset.mode);
  }));
}

function limitsText(mode) {
  if (!state.config) return '';
  const l = state.config.limits;
  return mode === 'base'
    ? `Ограничения: |v| ≤ ${l.max_linear} м/с, |ω| ≤ ${l.max_angular} рад/с, колёса ≤ ${l.max_wheel_speed} м/с.`
    : `Ограничение: |скорость колеса| ≤ ${l.max_wheel_speed} м/с.`;
}

// ---- Движение
segmented('drive-mode', (m) => { state.driveMode = m; updateDriveLabels(); });
function updateDriveLabels() {
  const base = state.driveMode === 'base';
  $('drive-a-label').textContent = base ? 'Линейная v, м/с' : 'Левое колесо, м/с';
  $('drive-b-label').textContent = base ? 'Угловая ω, рад/с' : 'Правое колесо, м/с';
  $('drive-b').step = base ? '0.1' : '0.01';
  $('drive-limits').textContent = limitsText(state.driveMode);
}

function readNumber(id) {
  const v = parseFloat($(id).value);
  if (!Number.isFinite(v)) { toast('Введите число: ' + $(id).closest('label').innerText.trim(), true); return null; }
  return v;
}

$('btn-drive').addEventListener('click', async () => {
  const a = readNumber('drive-a'); const b = readNumber('drive-b');
  if (a === null || b === null) return;
  const r = await api('command', { mode: state.driveMode, a, b });
  if (r && (r.a !== a || r.b !== b)) toast(`Ограничено до ${fmt(r.a, 3)} и ${fmt(r.b, 3)}`);
});
$('btn-drive-zero').addEventListener('click', () =>
  api('command', { mode: state.driveMode, a: 0, b: 0 }));

// ---- ПИД
function showCurrentPid() {
  for (const k of PID_KEYS) $('cur-' + k).textContent = state.pid ? fmt(state.pid[k], 4) : '—';
}
function fillPidIfEmpty() {
  if (!state.pid) return;
  if (PID_KEYS.every((k) => $(k).value === '')) copyPidToFields();
}
function copyPidToFields() {
  if (!state.pid) { toast('Текущие коэффициенты ещё не получены', true); return; }
  for (const k of PID_KEYS) $(k).value = state.pid[k];
}
function readPid() {
  const pid = {};
  for (const k of PID_KEYS) {
    const v = parseFloat($(k).value);
    if (!Number.isFinite(v) || v < 0) { toast(`${k}: нужно число ≥ 0`, true); $(k).focus(); return null; }
    pid[k] = v;
  }
  return pid;
}
['kp', 'ki', 'kd'].forEach((c) => $(c + '_l').addEventListener('input', () => {
  if ($('pid-same').checked) $(c + '_r').value = $(c + '_l').value;
}));
$('pid-same').addEventListener('change', () => {
  if ($('pid-same').checked) ['kp', 'ki', 'kd'].forEach((c) => { $(c + '_r').value = $(c + '_l').value; });
});
$('btn-pid-get').addEventListener('click', () => api('pid/request'));
$('btn-pid-copy').addEventListener('click', copyPidToFields);
$('btn-pid-set').addEventListener('click', async () => {
  const pid = readPid();
  if (pid && await api('pid/set', { pid })) toast('Коэффициенты отправлены (до перезагрузки ESP32)');
});
$('btn-pid-flash').addEventListener('click', async () => {
  const pid = readPid();
  if (!pid) return;
  const text = PID_KEYS.map((k) => `${k} = ${pid[k]}`).join('\n');
  if (!confirm('Записать коэффициенты во flash ESP32?\n\n' + text)) return;
  if (await api('pid/flash', { pid })) toast('Коэффициенты записаны во flash');
});

// ---- Ступенчатый тест
segmented('step-mode', (m) => { state.stepMode = m; updateStepLabels(); });
function updateStepLabels() {
  const base = state.stepMode === 'base';
  $('step-a-label').textContent = base ? 'Линейная v, м/с' : 'Левое, м/с';
  $('step-b-label').textContent = base ? 'Угловая ω, рад/с' : 'Правое, м/с';
  $('step-limits').textContent = limitsText(state.stepMode) +
    (state.config ? ` Длительность до ${state.config.step_test.max_duration} с.` : '');
}

$('btn-step-start').addEventListener('click', async () => {
  const a = readNumber('step-a'); const b = readNumber('step-b'); const duration = readNumber('step-duration');
  if (a === null || b === null || duration === null) return;
  $('step-results').classList.add('hidden');
  $('step-note').classList.add('hidden');
  $('step-chart-box').classList.add('hidden');
  await api('step_test/start', { mode: state.stepMode, a, b, duration });
});
$('btn-step-abort').addEventListener('click', () => api('step_test/abort'));

const SIGNAL_NAMES = { v: 'Линейная v', w: 'Угловая ω', vl: 'Левое колесо', vr: 'Правое колесо' };
const SIGNAL_UNITS = { v: 'м/с', w: 'рад/с', vl: 'м/с', vr: 'м/с' };

function updateStep(st) {
  const running = st.state === 'running' || st.state === 'tail';
  $('btn-step-start').disabled = running;
  $('btn-step-abort').disabled = !running;
  $('btn-drive').disabled = running;
  $('btn-step-csv').classList.toggle('disabled', st.state !== 'done');
  const total = st.duration ? st.duration + 1.0 : 1;
  $('step-bar').style.width = running ? Math.min(100, 100 * st.elapsed / total) + '%'
    : (st.state === 'done' ? '100%' : '0');
  const texts = {
    idle: 'Робот поедет — проверьте, что вокруг есть место.',
    running: `Ступенька: ${fmt(st.elapsed, 1)} из ${fmt(st.duration, 1)} с`,
    tail: 'Ступенька закончилась, записываю торможение…',
    done: 'Готово. Показатели — по отрезку ступеньки.',
    aborted: 'Прервано: ' + (st.reason || ''),
  };
  $('step-state').textContent = texts[st.state] || st.state;
  if (st.state === 'done' && state.stepState !== 'done') loadStepResult();
  state.stepState = st.state;
}

async function loadStepResult() {
  const r = await fetch('/api/step_test/recording').then((x) => x.json()).catch(() => null);
  if (!r) return;
  const result = r.status.result || {};
  const tbody = $('step-results').querySelector('tbody');
  tbody.textContent = '';
  for (const s of SIGNALS) {
    const m = result[s];
    if (!m) continue;
    const tr = document.createElement('tr');
    const cells = m.error ? [SIGNAL_NAMES[s], fmt(m.target), m.error, '', '', ''] : [
      SIGNAL_NAMES[s],
      `${fmt(m.target)} ${SIGNAL_UNITS[s]}`,
      fmt(m.rise_time, 2),
      fmt(m.overshoot, 1),
      m.settling_time === null ? 'не устан.' : fmt(m.settling_time, 2),
      `${fmt(m.steady_error)} (${fmt(m.steady_error_pct, 1)} %)`,
    ];
    for (const c of cells) { const td = document.createElement('td'); td.textContent = c; tr.appendChild(td); }
    tbody.appendChild(tr);
  }
  $('step-results').classList.remove('hidden');
  $('step-note').classList.remove('hidden');
  $('step-chart-box').classList.remove('hidden');
  stepChart.setRecording(r.samples, r.status.setpoints, r.status.duration);
}

// ================================================================ лог
const LOG_MAX = 400;
function appendLog(entries) {
  const box = $('log');
  const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;
  const showOdom = $('log-odom').checked;
  const frag = document.createDocumentFragment();
  for (const [t, dir, text] of entries) {
    if (dir === 'rx' && !showOdom && text.startsWith('$1;')) continue;
    const line = document.createElement('div');
    line.className = 'log-line ' + dir;
    const ts = document.createElement('span'); ts.className = 't'; ts.textContent = t.toFixed(2);
    const d = document.createElement('span'); d.className = 'd';
    d.textContent = { tx: '→', rx: '←', info: '•' }[dir] || dir;
    const tx = document.createElement('span'); tx.textContent = text;
    line.append(ts, d, tx);
    frag.appendChild(line);
  }
  box.appendChild(frag);
  while (box.childElementCount > LOG_MAX) box.firstElementChild.remove();
  if (atBottom) box.scrollTop = box.scrollHeight;
}
$('btn-log-clear').addEventListener('click', () => { $('log').textContent = ''; });

// ================================================================ графики (canvas)
function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function niceStep(span, ticks) {
  const raw = span / ticks;
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  const n = raw / p;
  return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * p;
}

class Chart {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
  }

  _prepare() {
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth; const h = this.canvas.clientHeight;
    if (this.canvas.width !== Math.round(w * dpr) || this.canvas.height !== Math.round(h * dpr)) {
      this.canvas.width = Math.round(w * dpr); this.canvas.height = Math.round(h * dpr);
    }
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { w, h };
  }

  // lines: [{points: [[t,v]...], color, dash, step}], xr: [t0,t1], xLabel(t)
  draw(lines, xr, xLabel) {
    const { w, h } = this._prepare();
    const ctx = this.ctx;
    ctx.clearRect(0, 0, w, h);
    if (w < 50 || h < 50) return;
    const m = { l: 46, r: 8, t: 6, b: 20 };
    const pw = w - m.l - m.r; const ph = h - m.t - m.b;

    let lo = 0; let hi = 0;
    for (const ln of lines) for (const [t, v] of ln.points) {
      if (t >= xr[0] - 1 && t <= xr[1]) { if (v < lo) lo = v; if (v > hi) hi = v; }
    }
    if (hi - lo < 0.05) { hi += 0.025; lo -= 0.025; }
    const pad = (hi - lo) * 0.1; lo -= pad; hi += pad;
    const X = (t) => m.l + (t - xr[0]) / (xr[1] - xr[0]) * pw;
    const Y = (v) => m.t + (hi - v) / (hi - lo) * ph;

    ctx.font = '11px ' + cssVar('--mono');
    ctx.lineWidth = 1;
    ctx.strokeStyle = cssVar('--grid');
    ctx.fillStyle = cssVar('--muted');
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
    const ys = niceStep(hi - lo, 4);
    for (let v = Math.ceil(lo / ys) * ys; v <= hi; v += ys) {
      ctx.beginPath(); ctx.moveTo(m.l, Y(v)); ctx.lineTo(w - m.r, Y(v)); ctx.stroke();
      ctx.fillText(Math.abs(v) < ys / 1e3 ? '0' : v.toFixed(ys < 0.1 ? 2 : 1), m.l - 4, Y(v));
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    const xs = niceStep(xr[1] - xr[0], 5);
    for (let t = Math.ceil(xr[0] / xs) * xs; t <= xr[1] + 1e-9; t += xs) {
      ctx.beginPath(); ctx.moveTo(X(t), m.t); ctx.lineTo(X(t), m.t + ph); ctx.stroke();
      ctx.fillText(xLabel(t), X(t), m.t + ph + 4);
    }

    ctx.save();
    ctx.beginPath(); ctx.rect(m.l, m.t, pw, ph); ctx.clip();
    for (const ln of lines) {
      const pts = ln.points;
      if (!pts.length) continue;
      ctx.strokeStyle = ln.color; ctx.lineWidth = ln.width || 1.6;
      ctx.setLineDash(ln.dash || []);
      ctx.beginPath();
      let started = false; let prev = null;
      for (const [t, v] of pts) {
        // точки левее окна обрезает clip; у ступенчатой уставки начало
        // прижимаем к краю окна, чтобы было видно текущее значение
        if (ln.step && prev !== null) ctx.lineTo(X(Math.max(t, xr[0])), Y(prev));
        const x = ln.step ? X(Math.max(t, xr[0])) : X(t);
        if (!started) { ctx.moveTo(x, Y(v)); started = true; } else ctx.lineTo(x, Y(v));
        prev = v;
      }
      if (ln.step && prev !== null) ctx.lineTo(X(ln.stepEnd ?? xr[1]), Y(prev));
      ctx.stroke();
    }
    ctx.restore();
    ctx.setLineDash([]);
  }
}

const liveCharts = Object.fromEntries(SIGNALS.map((s) => [s, new Chart($('chart-' + s))]));

function drawLive() {
  const win = parseFloat($('window').value);
  const now = state.serverT;
  const xr = [now - win, now];
  for (const s of SIGNALS) {
    liveCharts[s].draw([
      { points: state.sp[s], color: cssVar('--c-sp'), dash: [5, 4], step: true },
      { points: state.data[s], color: cssVar('--c-meas') },
    ], xr, (t) => { const r = Math.round(t - now); return (r === 0 ? '0' : r) + ' с'; });
  }
}

class StepChart extends Chart {
  setRecording(samples, setpoints, duration) {
    this.rec = { samples, setpoints, duration };
    this.render();
  }
  render() {
    if (!this.rec) return;
    const { samples, setpoints, duration } = this.rec;
    const end = duration + 1.0;
    const spLine = (s) => ({ points: [[0, setpoints[s] || 0], [duration, 0]], step: true, stepEnd: end,
      color: s === 'vl' ? cssVar('--c-left') : cssVar('--c-right'), dash: [5, 4], width: 1.2 });
    this.draw([
      spLine('vl'), spLine('vr'),
      { points: samples.vl || [], color: cssVar('--c-left') },
      { points: samples.vr || [], color: cssVar('--c-right') },
    ], [0, end], (t) => t.toFixed(1) + ' с');
  }
}
const stepChart = new StepChart($('step-chart'));

$('btn-pause').addEventListener('click', () => {
  state.paused = !state.paused;
  $('btn-pause').textContent = state.paused ? 'Продолжить' : 'Пауза';
  state.dirty = true;
});
$('window').addEventListener('change', () => { state.dirty = true; });
window.addEventListener('resize', () => { state.dirty = true; stepChart.render(); });
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => {
  state.dirty = true; stepChart.render();
});

let lastDraw = 0;
function frame(ts) {
  if (state.dirty && !state.paused && ts - lastDraw > 33) {
    drawLive(); state.dirty = false; lastDraw = ts;
  }
  requestAnimationFrame(frame);
}

// ================================================================ toast
let toastTimer = null;
function toast(text, error = false) {
  const el = $('toast');
  el.textContent = text;
  el.className = 'toast' + (error ? ' error' : '');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add('hidden'), error ? 5000 : 2500);
}

// ================================================================ старт
updateDriveLabels();
updateStepLabels();
connect();
requestAnimationFrame(frame);

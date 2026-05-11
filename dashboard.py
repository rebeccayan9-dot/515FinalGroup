#!/usr/bin/env python3
"""SafeStep real-time dashboard.
   pip install flask pyserial
   python dashboard.py
   Open: http://localhost:5000
"""

import csv, json, re, threading, time
from pathlib import Path
from queue import Empty, Queue

import joblib
import serial
import serial.tools.list_ports
from flask import Flask, Response, jsonify, request

MODEL_PATH = Path(__file__).parent / "model.pkl"
DATA_DIR   = Path(__file__).parent / "data"
BAUD       = 115200

PATTERN = re.compile(
    r"LiDAR:([\d.]+|OUT)\s*\|\s*F:([-\d.]+)\s+L:([-\d.]+)\s+R:([-\d.]+)"
    r"\s*\|\s*AX:([-\d.]+)\s+AY:([-\d.]+)\s+AZ:([-\d.]+)"
    r"\s+GX:([-\d.]+)\s+GY:([-\d.]+)\s+GZ:([-\d.]+)"
    r"(?:\s*\|\s*>>\s*(\S+))?"
)

app    = Flask(__name__)
_q     = Queue(maxsize=500)
_state = {"running": False}
_rec   = {"active": False, "label": None, "writer": None,
          "file": None, "start": 0.0, "count": 0}
_thr   = None


def _load_model():
    if not MODEL_PATH.exists():
        return {"importances": [], "classes": [], "n_estimators": "–",
                "max_depth": "–", "n_features": 0, "window": 5}
    b  = joblib.load(MODEL_PATH)
    fi = b["model"].feature_importances_
    fn = b["features"]
    pairs = sorted(zip(fn, fi.tolist()), key=lambda x: -x[1])[:15]
    return {
        "importances": [{"name": n, "value": round(v * 100, 2)} for n, v in pairs],
        "classes":     list(b["label_encoder"].classes_),
        "n_estimators": int(b["model"].n_estimators),
        "max_depth":   int(b["model"].max_depth),
        "n_features":  len(fn),
        "window":      int(b.get("window", 5)),
    }


MI = _load_model()


def _sse(event, data):
    try:
        _q.put_nowait(f"event:{event}\ndata:{json.dumps(data)}\n\n")
    except Exception:
        pass


def _reader(port):
    _state["running"] = True
    _sse("status", {"connected": True, "port": port})
    try:
        with serial.Serial(port, BAUD, timeout=2) as ser:
            while _state["running"]:
                raw = ser.readline().decode("utf-8", errors="replace").strip()
                if not raw:
                    continue
                m = PATTERN.search(raw)
                if not m:
                    continue
                lr, f, l, r, ax, ay, az, gx, gy, gz, pred = m.groups()
                lidar = 400.0 if lr == "OUT" else float(lr)
                pkt = {
                    "t":     round(time.time() * 1000),
                    "lidar": lidar,
                    "front": float(f),  "left": float(l),  "right": float(r),
                    "ax":    float(ax), "ay":   float(ay), "az":    float(az),
                    "gx":    float(gx), "gy":   float(gy), "gz":    float(gz),
                    "pred":  pred or "...",
                }
                _sse("sensor", pkt)
                if _rec["active"] and _rec["writer"]:
                    elapsed = round(time.time() - _rec["start"], 3)
                    _rec["writer"].writerow([
                        elapsed, lidar,
                        float(f), float(l), float(r),
                        float(ax), float(ay), float(az),
                        float(gx), float(gy), float(gz),
                        _rec["label"],
                    ])
                    _rec["file"].flush()
                    _rec["count"] += 1
                    _sse("rec_tick", {"n": _rec["count"]})
    except Exception as e:
        _sse("status", {"connected": False, "error": str(e)})
    finally:
        _state["running"] = False
        _sse("status", {"connected": False})


@app.route("/")
def index():
    return Response(HTML, mimetype="text/html")

@app.route("/stream")
def stream():
    def gen():
        while True:
            try:
                yield _q.get(timeout=15)
            except Empty:
                yield "event:ping\ndata:{}\n\n"
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

@app.route("/ports")
def ports_route():
    return jsonify([{"device": p.device, "desc": p.description}
                    for p in serial.tools.list_ports.comports()])

@app.route("/model-info")
def model_info():
    return jsonify(MI)

@app.route("/connect", methods=["POST"])
def connect():
    global _thr
    port = (request.json or {}).get("port")
    if not port:
        return jsonify({"error": "no port"}), 400
    if _state["running"]:
        _state["running"] = False
        time.sleep(0.5)
    _thr = threading.Thread(target=_reader, args=(port,), daemon=True)
    _thr.start()
    return jsonify({"ok": True})

@app.route("/disconnect", methods=["POST"])
def disconnect():
    _state["running"] = False
    return jsonify({"ok": True})

@app.route("/record/start", methods=["POST"])
def rec_start():
    label = (request.json or {}).get("label", "unlabeled").strip().replace(" ", "_")
    ts    = time.strftime("%Y%m%d_%H%M%S")
    path  = DATA_DIR / f"{label}_{ts}.csv"
    DATA_DIR.mkdir(exist_ok=True)
    fh = open(path, "w", newline="")
    _rec.update(active=True, label=label, file=fh, start=time.time(),
                count=0, writer=csv.writer(fh))
    _rec["writer"].writerow(["timestamp_s", "lidar_cm", "front_cm", "left_cm", "right_cm",
                              "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "label"])
    _sse("rec_status", {"active": True, "label": label, "file": path.name})
    return jsonify({"ok": True, "path": str(path)})

@app.route("/record/stop", methods=["POST"])
def rec_stop():
    _rec["active"] = False
    if _rec["file"]:
        _rec["file"].close()
        _rec["file"]   = None
        _rec["writer"] = None
    _sse("rec_status", {"active": False, "count": _rec["count"]})
    return jsonify({"ok": True, "count": _rec["count"]})


# ─────────────────────────────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SafeStep Dashboard</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.2/dist/chart.umd.min.js"></script>
<style>
:root {
  --bg:    #0b0e16;
  --card:  #131828;
  --bdr:   #1e2540;
  --text:  #dde3f0;
  --muted: #6a7591;
  --acc:   #4f8ef7;
  --green: #22c55e;
  --amber: #f59e0b;
  --red:   #ef4444;
}
*,*::before,*::after { box-sizing:border-box; margin:0; padding:0; }
html,body { height:100%; font-family:'Segoe UI',system-ui,sans-serif;
            background:var(--bg); color:var(--text); font-size:12px; }

/* ── header ── */
header {
  display:flex; align-items:center; gap:10px;
  padding:0 14px; height:50px;
  background:var(--card); border-bottom:1px solid var(--bdr);
  position:sticky; top:0; z-index:100;
}
header h1 { font-size:14px; font-weight:700; color:var(--acc); flex:1; letter-spacing:.4px; }
.dot { width:8px; height:8px; border-radius:50%; background:var(--muted); flex-shrink:0; }
.dot.on { background:var(--green); box-shadow:0 0 7px var(--green); }

select,button {
  background:var(--card); color:var(--text); border:1px solid var(--bdr);
  padding:4px 9px; border-radius:6px; cursor:pointer; font-size:11px;
}
button:hover { border-color:var(--acc); }
button.pri  { background:var(--acc);   color:#fff; border-color:var(--acc); }
button.dng  { background:#b91c1c;      color:#fff; border-color:#b91c1c; }
button.rec  { background:#dc2626;      color:#fff; border-color:#dc2626; }
button:disabled { opacity:.4; cursor:default; }
#conn-lbl { font-size:11px; color:var(--muted); min-width:80px; }

/* ── layout ── */
main {
  display:grid;
  grid-template-columns:1fr 230px 1fr;
  gap:7px; padding:7px;
  height:calc(100vh - 50px - 54px);
  overflow:hidden;
}
.col { display:flex; flex-direction:column; gap:7px; min-height:0; }

/* ── cards ── */
.card {
  background:var(--card); border:1px solid var(--bdr);
  border-radius:8px; padding:9px 10px; overflow:hidden;
}
.card-title {
  font-size:9px; font-weight:700; letter-spacing:1.2px;
  text-transform:uppercase; color:var(--muted); margin-bottom:7px;
}
.chart-wrap { position:relative; flex:1; min-height:0; }
.chart-wrap canvas { width:100%!important; height:100%!important; }

/* ── gauges ── */
.gr { display:flex; align-items:center; gap:7px; margin-bottom:5px; }
.gl { width:36px; font-size:10px; color:var(--muted); flex-shrink:0; }
.gt { flex:1; height:7px; background:#1e2540; border-radius:4px; overflow:hidden; }
.gf { height:100%; border-radius:4px; transition:width .12s,background .15s; background:var(--green); }
.gv { width:62px; text-align:right; font-size:10px; font-variant-numeric:tabular-nums; }

/* ── prediction ── */
#pred-box {
  font-size:18px; font-weight:700; text-align:center;
  padding:11px 14px; border-radius:9px; width:100%;
  transition:background .25s,color .25s,border-color .25s;
  border:2px solid var(--bdr); background:#1e2540; letter-spacing:.4px;
}
#pred-box.left_turn    { background:#162043; border-color:#3b82f6; color:#93c5fd; }
#pred-box.right_turn   { background:#1e1043; border-color:#8b5cf6; color:#c4b5fd; }
#pred-box.walking      { background:#0d2e18; border-color:#22c55e; color:#86efac; }
#pred-box.stop         { background:#2d1e00; border-color:#f59e0b; color:#fde68a; }
#pred-box.step_up      { background:#2d1400; border-color:#f97316; color:#fdba74; }
#pred-box.step_down    { background:#2d0a0a; border-color:#ef4444; color:#fca5a5; }
#pred-box.obstacle_avoid { background:#2d0720; border-color:#ec4899; color:#f9a8d4; }

/* ── session bars ── */
.sb-row { display:flex; align-items:center; gap:6px; margin-bottom:4px; }
.sb-lbl { width:86px; font-size:10px; color:var(--muted); white-space:nowrap; overflow:hidden; text-overflow:ellipsis; flex-shrink:0; }
.sb-trk { flex:1; background:#1e2540; border-radius:3px; height:6px; overflow:hidden; }
.sb-bar { height:100%; border-radius:3px; transition:width .3s; }
.sb-cnt { width:26px; text-align:right; font-size:10px; color:var(--muted); }

/* ── recent list ── */
#rec-list { list-style:none; overflow-y:auto; max-height:130px; }
#rec-list li {
  display:flex; justify-content:space-between;
  padding:2px 6px; margin-bottom:2px; border-radius:4px;
  font-size:10px; background:#1e2540; color:var(--muted);
}
#rec-list li:first-child { background:#252d4a; color:var(--text); }
#rec-list .ts { color:#3d4a6a; }

/* ── DSP table ── */
#win-tbl { width:100%; border-collapse:collapse; font-size:10px; }
#win-tbl th {
  color:var(--muted); font-weight:600; padding:2px 4px;
  text-align:right; border-bottom:1px solid var(--bdr);
}
#win-tbl th:first-child { text-align:left; }
#win-tbl td { padding:2px 4px; text-align:right; font-variant-numeric:tabular-nums; }
#win-tbl td:first-child { text-align:left; color:var(--muted); }
#win-tbl tr:nth-child(even) { background:rgba(255,255,255,.025); }

/* ── model pills ── */
.pills { display:flex; gap:5px; flex-wrap:wrap; }
.pill { padding:2px 8px; background:#1e2540; border-radius:10px; font-size:10px; color:var(--muted); }
.pill b { color:var(--text); }

/* ── footer ── */
footer {
  display:flex; align-items:center; gap:9px; flex-wrap:wrap;
  padding:0 14px; height:54px;
  background:var(--card); border-top:1px solid var(--bdr);
}
footer label { font-size:10px; color:var(--muted); }
#rec-info { font-size:10px; color:var(--muted); }
#rec-cnt  { font-size:11px; color:var(--green); font-weight:600; }
.rdot { width:7px; height:7px; border-radius:50%; background:var(--red); display:none; }
.rdot.on { display:block; animation:blink 1s infinite; }
@keyframes blink { 0%,100%{opacity:1} 50%{opacity:.2} }
</style>
</head>
<body>

<header>
  <h1>&#x1F9BE; SafeStep Live Dashboard</h1>
  <div class="dot" id="conn-dot"></div>
  <span id="conn-lbl">Disconnected</span>
  <select id="port-sel" style="width:170px"><option value="">Select port&hellip;</option></select>
  <button onclick="refreshPorts()" title="Refresh ports">&#x21BB;</button>
  <button id="conn-btn" class="pri" onclick="toggleConn()">Connect</button>
</header>

<main>

  <!-- ── LEFT: Sensors ── -->
  <div class="col">

    <div class="card" style="flex:0 0 auto">
      <div class="card-title">Distance Sensors</div>
      <div class="gr"><span class="gl">LiDAR</span><div class="gt"><div class="gf" id="g-lidar" style="width:0%"></div></div><span class="gv" id="v-lidar">&#x2014;</span></div>
      <div class="gr"><span class="gl">Front</span> <div class="gt"><div class="gf" id="g-front" style="width:0%"></div></div><span class="gv" id="v-front">&#x2014;</span></div>
      <div class="gr"><span class="gl">Left</span>  <div class="gt"><div class="gf" id="g-left"  style="width:0%"></div></div><span class="gv" id="v-left">&#x2014;</span></div>
      <div class="gr"><span class="gl">Right</span> <div class="gt"><div class="gf" id="g-right" style="width:0%"></div></div><span class="gv" id="v-right">&#x2014;</span></div>
    </div>

    <div class="card" style="flex:1;display:flex;flex-direction:column">
      <div class="card-title">Distance Time-Series (cm)</div>
      <div class="chart-wrap"><canvas id="c-dist"></canvas></div>
    </div>

    <div class="card" style="flex:1;display:flex;flex-direction:column">
      <div class="card-title">Accelerometer (g)</div>
      <div class="chart-wrap"><canvas id="c-acc"></canvas></div>
    </div>

    <div class="card" style="flex:1;display:flex;flex-direction:column">
      <div class="card-title">Gyroscope (dps)</div>
      <div class="chart-wrap"><canvas id="c-gyr"></canvas></div>
    </div>

  </div>

  <!-- ── CENTER: ML Prediction ── -->
  <div class="col">

    <div class="card" style="flex:0 0 auto;display:flex;flex-direction:column;align-items:center;gap:6px">
      <div class="card-title" style="align-self:flex-start">Current Prediction</div>
      <div id="pred-box">Waiting&hellip;</div>
      <div id="pred-age" style="font-size:9px;color:var(--muted)">&#x2014;</div>
    </div>

    <div class="card" style="flex:1;overflow:auto">
      <div class="card-title">Session Activity</div>
      <div id="sess-bars"></div>
    </div>

    <div class="card" style="flex:1;overflow:auto">
      <div class="card-title">Recent Predictions</div>
      <ul id="rec-list"></ul>
    </div>

  </div>

  <!-- ── RIGHT: Analysis ── -->
  <div class="col">

    <div class="card" style="flex:2;display:flex;flex-direction:column">
      <div class="card-title">Feature Importance &mdash; Random Forest (top 15)</div>
      <div style="font-size:9px;color:var(--muted);margin-bottom:4px">
        <span style="color:#4f8ef7">&#x25A0;</span> Distance &nbsp;
        <span style="color:#22c55e">&#x25A0;</span> Accel &nbsp;
        <span style="color:#f59e0b">&#x25A0;</span> Gyro
      </div>
      <div class="chart-wrap"><canvas id="c-imp"></canvas></div>
    </div>

    <div class="card" style="flex:1;overflow:auto">
      <div class="card-title">DSP Window Analysis &mdash; last 5 samples &rarr; 40 features</div>
      <table id="win-tbl">
        <thead>
          <tr><th>Channel</th><th>Mean</th><th>Std</th><th>Min</th><th>Max</th></tr>
        </thead>
        <tbody id="win-body">
          <tr><td colspan="5" style="color:var(--muted);text-align:center;padding:10px">Waiting for data&hellip;</td></tr>
        </tbody>
      </table>
    </div>

    <div class="card" style="flex:0 0 auto">
      <div class="card-title">Model Architecture</div>
      <div class="pills" id="model-pills"></div>
    </div>

  </div>

</main>

<footer>
  <div class="rdot" id="rdot"></div>
  <label>Label:</label>
  <select id="rec-lbl">
    <option value="walking">walking</option>
    <option value="left_turn">left_turn</option>
    <option value="right_turn">right_turn</option>
    <option value="stop">stop</option>
    <option value="step_up">step_up</option>
    <option value="step_down">step_down</option>
    <option value="obstacle_avoid">obstacle_avoid</option>
  </select>
  <button id="btn-start" class="pri" onclick="startRec()">&#x25CF; Start Recording</button>
  <button id="btn-stop"  class="dng" onclick="stopRec()" disabled>&#x25A0; Stop</button>
  <span id="rec-info">Ready to record</span>
  <span id="rec-cnt"></span>
</footer>

<script>
// ── constants ────────────────────────────────────────────────────────────────
const MAX_PTS  = 60;
const WIN_SIZE = 5;
const ALL_LBLS = ['left_turn','obstacle_avoid','right_turn',
                  'step_down','step_up','stop','walking'];
const LBL_CLR  = {
  left_turn:     '#3b82f6',
  right_turn:    '#8b5cf6',
  walking:       '#22c55e',
  stop:          '#f59e0b',
  step_up:       '#f97316',
  step_down:     '#ef4444',
  obstacle_avoid:'#ec4899',
};

// ── state ────────────────────────────────────────────────────────────────────
let winBuf     = [];
let sessCounts = {};
let recentPreds= [];
let lastPredTs = null;
let connected  = false;
let evtSrc     = null;

// ── Chart.js defaults ────────────────────────────────────────────────────────
Chart.defaults.color       = '#6a7591';
Chart.defaults.borderColor = '#1e2540';
Chart.defaults.font.size   = 9;

function mkLine(id, defs, yMin, yMax) {
  return new Chart(document.getElementById(id), {
    type: 'line',
    data: {
      labels: Array(MAX_PTS).fill(''),
      datasets: defs.map(d => ({
        label: d.lbl,
        data:  Array(MAX_PTS).fill(null),
        borderColor: d.clr, borderWidth: 1.3,
        pointRadius: 0, tension: 0.25, fill: false,
      }))
    },
    options: {
      animation: false, responsive: true, maintainAspectRatio: false,
      plugins: { legend: { position:'bottom', labels:{ boxWidth:8, padding:5, font:{size:9} } } },
      scales: {
        x: { display: false },
        y: { min:yMin, max:yMax, grid:{ color:'#1e2540' }, ticks:{ maxTicksLimit:5 } }
      }
    }
  });
}

function mkBar(id) {
  return new Chart(document.getElementById(id), {
    type: 'bar',
    data: { labels:[], datasets:[{ data:[], backgroundColor:[], borderRadius:3 }] },
    options: {
      indexAxis: 'y', animation:false, responsive:true, maintainAspectRatio:false,
      plugins: { legend:{ display:false } },
      scales: {
        x: { grid:{ color:'#1e2540' }, ticks:{ font:{size:9} } },
        y: { grid:{ display:false },   ticks:{ font:{size:9} } }
      }
    }
  });
}

const cDist = mkLine('c-dist',
  [{lbl:'LiDAR',clr:'#4f8ef7'},{lbl:'Front',clr:'#22c55e'},
   {lbl:'Left', clr:'#f59e0b'},{lbl:'Right',clr:'#ec4899'}], 0, 400);

const cAcc = mkLine('c-acc',
  [{lbl:'AX',clr:'#ef4444'},{lbl:'AY',clr:'#22c55e'},{lbl:'AZ',clr:'#4f8ef7'}], -4, 4);

const cGyr = mkLine('c-gyr',
  [{lbl:'GX',clr:'#ef4444'},{lbl:'GY',clr:'#22c55e'},{lbl:'GZ',clr:'#4f8ef7'}], -250, 250);

const cImp = mkBar('c-imp');

// ── helpers ──────────────────────────────────────────────────────────────────
function push(chart, i, v) {
  const d = chart.data.datasets[i].data;
  d.push(v);
  if (d.length > MAX_PTS) d.shift();
}

function setGauge(key, val) {
  const pct = Math.min(100, (val / 400) * 100).toFixed(1);
  const f   = document.getElementById('g-' + key);
  const v   = document.getElementById('v-' + key);
  f.style.width      = pct + '%';
  f.style.background = val < 50 ? '#ef4444' : val < 150 ? '#f59e0b' : '#22c55e';
  v.textContent      = val >= 399 ? 'OUT' : val.toFixed(1) + ' cm';
}

// ── DSP window analysis ──────────────────────────────────────────────────────
const WIN_FLDS = ['lidar','front','left','right','ax','ay','az','gx','gy','gz'];
const WIN_NAMS = ['LiDAR (cm)','Front (cm)','Left (cm)','Right (cm)',
                  'AX (g)','AY (g)','AZ (g)','GX (dps)','GY (dps)','GZ (dps)'];

function updateWinAnalysis() {
  if (winBuf.length < WIN_SIZE) return;
  const rows = WIN_FLDS.map((f, i) => {
    const vals = winBuf.map(p => p[f]);
    const n    = vals.length;
    const mean = vals.reduce((a,b)=>a+b,0) / n;
    const std  = Math.sqrt(vals.reduce((a,b)=>a+(b-mean)**2,0) / n);
    const min  = Math.min(...vals);
    const max  = Math.max(...vals);
    return `<tr>
      <td>${WIN_NAMS[i]}</td>
      <td>${mean.toFixed(3)}</td>
      <td>${std.toFixed(3)}</td>
      <td>${min.toFixed(3)}</td>
      <td>${max.toFixed(3)}</td>
    </tr>`;
  });
  document.getElementById('win-body').innerHTML = rows.join('');
}

// ── prediction ───────────────────────────────────────────────────────────────
function onPrediction(lbl) {
  const el = document.getElementById('pred-box');
  el.textContent = lbl;
  el.className   = lbl;
  lastPredTs     = Date.now();

  sessCounts[lbl] = (sessCounts[lbl] || 0) + 1;
  recentPreds.unshift({ lbl, t: new Date().toLocaleTimeString() });
  if (recentPreds.length > 15) recentPreds.pop();

  renderSess();
  renderRecent();
}

function renderSess() {
  const tot = Math.max(1, Object.values(sessCounts).reduce((a,b)=>a+b,0));
  document.getElementById('sess-bars').innerHTML = ALL_LBLS.map(l => {
    const n   = sessCounts[l] || 0;
    const pct = (n / tot * 100).toFixed(1);
    return `<div class="sb-row">
      <span class="sb-lbl">${l}</span>
      <div class="sb-trk"><div class="sb-bar" style="width:${pct}%;background:${LBL_CLR[l]}"></div></div>
      <span class="sb-cnt">${n}</span>
    </div>`;
  }).join('');
}

function renderRecent() {
  document.getElementById('rec-list').innerHTML = recentPreds.map(p =>
    `<li><span>${p.lbl}</span><span class="ts">${p.t}</span></li>`
  ).join('');
}

// ── model info ───────────────────────────────────────────────────────────────
async function loadModelInfo() {
  let info;
  try { info = await (await fetch('/model-info')).json(); }
  catch { return; }

  if (info.importances && info.importances.length) {
    cImp.data.labels = info.importances.map(f => f.name);
    cImp.data.datasets[0].data = info.importances.map(f => f.value);
    cImp.data.datasets[0].backgroundColor = info.importances.map(f => {
      const n = f.name;
      if (n.startsWith('lidar') || n.startsWith('front') ||
          n.startsWith('left')  || n.startsWith('right')) return '#4f8ef7';
      if (n.startsWith('ax') || n.startsWith('ay') || n.startsWith('az')) return '#22c55e';
      return '#f59e0b';
    });
    cImp.update();
  }

  document.getElementById('model-pills').innerHTML = [
    `<div class="pill">Trees: <b>${info.n_estimators}</b></div>`,
    `<div class="pill">Max depth: <b>${info.max_depth}</b></div>`,
    `<div class="pill">Features: <b>${info.n_features}</b></div>`,
    `<div class="pill">Window: <b>${info.window} samples</b></div>`,
    `<div class="pill">Classes: <b>${(info.classes||[]).length}</b></div>`,
  ].join('');
}

// ── SSE ──────────────────────────────────────────────────────────────────────
function connectSSE() {
  if (evtSrc) evtSrc.close();
  evtSrc = new EventSource('/stream');

  evtSrc.addEventListener('sensor', e => {
    const p = JSON.parse(e.data);

    // gauges
    setGauge('lidar', p.lidar); setGauge('front', p.front);
    setGauge('left',  p.left);  setGauge('right', p.right);

    // distance chart
    push(cDist,0,p.lidar); push(cDist,1,p.front);
    push(cDist,2,p.left);  push(cDist,3,p.right);
    cDist.update('none');

    // accel chart
    push(cAcc,0,p.ax); push(cAcc,1,p.ay); push(cAcc,2,p.az);
    cAcc.update('none');

    // gyro chart
    push(cGyr,0,p.gx); push(cGyr,1,p.gy); push(cGyr,2,p.gz);
    cGyr.update('none');

    // DSP window
    winBuf.push(p);
    if (winBuf.length > WIN_SIZE) winBuf.shift();
    updateWinAnalysis();

    // prediction
    if (p.pred && p.pred !== '...') onPrediction(p.pred);
  });

  evtSrc.addEventListener('status', e => {
    const s = JSON.parse(e.data);
    setConn(s.connected);
    if (s.error) console.warn('Serial error:', s.error);
  });

  evtSrc.addEventListener('rec_status', e => {
    const s = JSON.parse(e.data);
    if (s.active) {
      document.getElementById('rec-info').textContent = `Recording: ${s.label}  →  ${s.file}`;
      document.getElementById('rdot').classList.add('on');
      document.getElementById('rec-cnt').textContent  = '0 samples';
    } else {
      document.getElementById('rec-info').textContent = `Saved ${s.count} samples`;
      document.getElementById('rdot').classList.remove('on');
      document.getElementById('rec-cnt').textContent  = '';
    }
  });

  evtSrc.addEventListener('rec_tick', e => {
    const d = JSON.parse(e.data);
    document.getElementById('rec-cnt').textContent = `${d.n} samples`;
  });

  evtSrc.onerror = () => setTimeout(connectSSE, 2500);
}

// ── connection ───────────────────────────────────────────────────────────────
function setConn(on) {
  connected = on;
  document.getElementById('conn-dot').className = 'dot' + (on ? ' on' : '');
  document.getElementById('conn-lbl').textContent = on ? 'Connected' : 'Disconnected';
  const btn = document.getElementById('conn-btn');
  btn.textContent = on ? 'Disconnect' : 'Connect';
  btn.className   = on ? 'dng' : 'pri';
}

async function refreshPorts() {
  let ports;
  try { ports = await (await fetch('/ports')).json(); }
  catch { return; }
  const sel = document.getElementById('port-sel');
  const cur = sel.value;
  sel.innerHTML = '<option value="">Select port…</option>' +
    ports.map(p =>
      `<option value="${p.device}" ${p.device===cur?'selected':''}>${p.device} – ${p.desc}</option>`
    ).join('');
}

async function toggleConn() {
  if (connected) {
    await fetch('/disconnect', { method:'POST' });
    setConn(false);
  } else {
    const port = document.getElementById('port-sel').value;
    if (!port) { alert('Select a serial port first.'); return; }
    await fetch('/connect', {
      method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({ port })
    });
  }
}

// ── recording ────────────────────────────────────────────────────────────────
async function startRec() {
  const label = document.getElementById('rec-lbl').value;
  await fetch('/record/start', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({ label })
  });
  document.getElementById('btn-start').disabled = true;
  document.getElementById('btn-stop').disabled  = false;
}

async function stopRec() {
  await fetch('/record/stop', { method:'POST' });
  document.getElementById('btn-start').disabled = false;
  document.getElementById('btn-stop').disabled  = true;
}

// ── init ─────────────────────────────────────────────────────────────────────
connectSSE();
loadModelInfo();
refreshPorts();
renderSess();
setInterval(refreshPorts, 6000);

setInterval(() => {
  if (!lastPredTs) return;
  const s = Math.round((Date.now() - lastPredTs) / 1000);
  document.getElementById('pred-age').textContent =
    s < 3 ? 'just now' : `${s}s ago`;
}, 1000);
</script>
</body>
</html>"""


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("SafeStep Dashboard  →  http://localhost:8050")
    app.run(port=8050, threaded=True, debug=False)

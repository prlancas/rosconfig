/*
 * Droidal robot control dashboard.
 *
 * Served by android_bridge.py at http://<host>:8791/.
 * All robot data arrives via WebSocket request/response + server push events.
 * Map PNG is fetched via HTTP (binary, used in drawImage).
 *
 * Layers (all toggleable):
 *   map · nav-path · lidar-scan · frontiers · waypoints · objects · robot
 *
 * Interactions:
 *   Left-click map  → navigate to that world coordinate
 *   Click waypoint pin → go to that waypoint
 *   Click object pin   → show detail popover
 *   Right-click        → cancel goal / deselect
 *   Scroll             → zoom (centred on cursor)
 *   Shift+left-drag / middle-drag → pan
 *   F key / Fit Map button → reset view to fit map
 */
(() => {
  'use strict';

  // ── Constants ──────────────────────────────────────────────────────────────
  const REFRESH_MAP_MS  = 4000;  // map PNG + JSON refresh
  const REFRESH_FAST_MS =  500;  // pose, path, scan
  const REFRESH_SLOW_MS = 5000;  // waypoints, objects, frontiers

  // Laser-frame → base_link: roll=π flips angles, yaw=π/2 rotates.
  // Effective: angle_map = π/2 - scan_angle + robot_yaw
  const LASER_YAW_OFF = Math.PI / 2;

  const ROBOT_RADIUS_PX = 10;  // robot body radius at zoom=1 display-px
  const PIN_R = 6;              // object/waypoint pin radius
  const BATTERY_MIN_V = 20.0;
  const BATTERY_MAX_V = 25.2;
  const UNKNOWN_GRAY  = 205;   // matches android_bridge map_png() unknown value
  const DOOR_PROBE_M  = 1.2;

  // ── DOM refs ───────────────────────────────────────────────────────────────
  const $ = id => document.getElementById(id);
  const canvas = $('map');
  const ctx    = canvas.getContext('2d');

  // ── App state ──────────────────────────────────────────────────────────────
  let mapMeta    = null;  // {resolution, width, height, origin:{x,y,yaw}}
  let mapImg     = null;  // HTMLImageElement
  let mapPixels  = null;  // ImageData for unexplored-door probe
  let robotPose  = null;  // {x, y, yaw, stamp}
  let navPath    = [];    // [{x,y},...] from /plan
  let scanData   = null;  // {angle_min, angle_increment, range_min, range_max, ranges}
  let objects    = [];    // [{canonical, label, worldX, worldY, isDoor, ...}]
  let waypoints  = {};    // {label: {x, y, yaw}}
  let frontiers  = [];    // [{x, y, size, is_doorway}]
  let navStatus  = { status: 'IDLE', target: null, elapsed_s: 0 };
  let exploreEnabled = false;
  let batteryV   = null;
  let clickTarget = null;   // {x, y} world-coords of pending goal marker
  let selectedObjId = null;

  // Canvas view transform
  let zoom = 1;
  let panX = 0, panY = 0;        // canvas-px offset of map origin
  let mapPxScale = 1;            // native map pixels → canvas px at zoom=1
  let isPanning = false;
  let panStart  = null;
  let hasFitOnce = false;

  // ── Layer definitions ──────────────────────────────────────────────────────
  const LAYERS = {
    map:       { label: 'Map',       on: true  },
    path:      { label: 'Nav Path',  on: true  },
    scan:      { label: 'LIDAR',     on: false }, // off by default (perf)
    frontiers: { label: 'Frontiers', on: true  },
    waypoints: { label: 'Waypoints', on: true  },
    objects:   { label: 'Objects',   on: true  },
    robot:     { label: 'Robot',     on: true  },
  };

  // ── WebSocket client ───────────────────────────────────────────────────────
  class RobotWS {
    constructor() {
      this._ws = null;
      this._pending = new Map();
      this._handlers = {};
      this._backoff = 1000;
      this._connect();
    }

    _connect() {
      const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
      this._ws = new WebSocket(`${proto}//${location.host}`);

      this._ws.addEventListener('open', () => {
        setConnected(true);
        this._backoff = 1000;
        this._emit('open');
      });
      this._ws.addEventListener('close', () => {
        setConnected(false);
        setTimeout(() => this._connect(), this._backoff);
        this._backoff = Math.min(this._backoff * 2, 16000);
        this._emit('close');
      });
      this._ws.addEventListener('error', () => {}); // handled by close

      this._ws.addEventListener('message', ev => {
        let msg;
        try { msg = JSON.parse(ev.data); } catch { return; }

        // Request/response reply
        if (msg.id && this._pending.has(msg.id)) {
          const { resolve, reject, timer } = this._pending.get(msg.id);
          this._pending.delete(msg.id);
          clearTimeout(timer);
          msg.error ? reject(new Error(msg.error)) : resolve(msg.result);
          return;
        }
        // Server push event
        if (msg.type === 'event') this._emit(msg.event, msg);
      });
    }

    /** Send a request and wait for the matching reply. */
    request(method, path, body = null) {
      return new Promise((resolve, reject) => {
        if (!this._ws || this._ws.readyState !== WebSocket.OPEN) {
          reject(new Error('not connected')); return;
        }
        const id = crypto.randomUUID();
        const timer = setTimeout(() => {
          this._pending.delete(id);
          reject(new Error('timeout'));
        }, 6000);
        this._pending.set(id, { resolve, reject, timer });
        this._ws.send(JSON.stringify({ type: 'request', id, method, path, body }));
      });
    }

    /** Fire-and-forget command. */
    command(cmd, extra = {}) {
      if (this._ws?.readyState === WebSocket.OPEN)
        this._ws.send(JSON.stringify({ type: 'command', command: cmd, ...extra }));
    }

    on(event, fn) { (this._handlers[event] ??= []).push(fn); }
    _emit(event, data) { (this._handlers[event] ?? []).forEach(fn => fn(data)); }
  }

  const ws = new RobotWS();

  // ── Coordinate transforms ──────────────────────────────────────────────────
  // android_bridge flips rows when making the PNG (north-up), so:
  //   native row 0 = world y_max,  native row (height-1) = world y_min.
  //
  // We draw the map image at canvas position (panX, panY) with scale = mapPxScale * zoom.

  function worldToCanvas(wx, wy) {
    if (!mapMeta) return { x: 0, y: 0 };
    const col = (wx - mapMeta.origin.x) / mapMeta.resolution;
    const row = (mapMeta.height - 1) - (wy - mapMeta.origin.y) / mapMeta.resolution;
    return {
      x: panX + col * mapPxScale * zoom,
      y: panY + row * mapPxScale * zoom,
    };
  }

  function canvasToWorld(cx, cy) {
    if (!mapMeta) return null;
    const col = (cx - panX) / (mapPxScale * zoom);
    const row = (cy - panY) / (mapPxScale * zoom);
    return {
      x: mapMeta.origin.x + col * mapMeta.resolution,
      y: mapMeta.origin.y + ((mapMeta.height - 1) - row) * mapMeta.resolution,
    };
  }

  // ── Canvas sizing + fit ────────────────────────────────────────────────────
  function resizeCanvas() {
    const s = $('stage');
    canvas.width  = s.clientWidth;
    canvas.height = s.clientHeight;
  }

  function fitMap(force = false) {
    if (!mapMeta || !mapImg) return;
    if (!force && hasFitOnce) return;
    const s = $('stage');
    const sw = s.clientWidth, sh = s.clientHeight;
    mapPxScale = Math.min(sw / mapMeta.width, sh / mapMeta.height);
    zoom = 1;
    panX = (sw - mapMeta.width  * mapPxScale) / 2;
    panY = (sh - mapMeta.height * mapPxScale) / 2;
    hasFitOnce = true;
  }

  // ── Draw functions ─────────────────────────────────────────────────────────
  function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if (!mapMeta) return;

    const mapW = mapMeta.width  * mapPxScale * zoom;
    const mapH = mapMeta.height * mapPxScale * zoom;

    // Map background
    if (LAYERS.map.on && mapImg) {
      ctx.drawImage(mapImg, panX, panY, mapW, mapH);
    }

    // LIDAR scan
    if (LAYERS.scan.on && scanData && robotPose) drawScan();

    // Nav2 planned path
    if (LAYERS.path.on && navPath.length > 1) drawPath();

    // Frontier clusters
    if (LAYERS.frontiers.on && frontiers.length > 0) drawFrontiers();

    // Waypoint pins
    if (LAYERS.waypoints.on) drawWaypointPins();

    // Object pins
    if (LAYERS.objects.on) drawObjectPins();

    // Robot body + heading
    if (LAYERS.robot.on && robotPose) drawRobot();

    // Pending click-to-navigate target
    if (clickTarget) drawClickTarget();
  }

  function drawScan() {
    ctx.save();
    ctx.fillStyle = 'rgba(87,211,140,0.55)';
    const { x: rx, y: ry, yaw } = robotPose;
    const { angle_min, angle_increment, range_min, range_max, ranges } = scanData;
    for (let i = 0; i < ranges.length; i++) {
      const r = ranges[i];
      if (r == null || r < range_min || r > range_max) continue;
      // laser_frame→map: π/2 - scan_angle + robot_yaw  (derived from TF roll=π, yaw=π/2)
      const angle = LASER_YAW_OFF - (angle_min + i * angle_increment) + yaw;
      const p = worldToCanvas(rx + r * Math.cos(angle), ry + r * Math.sin(angle));
      ctx.fillRect(p.x - 1.5, p.y - 1.5, 3, 3);
    }
    ctx.restore();
  }

  function drawPath() {
    ctx.save();
    ctx.strokeStyle = 'rgba(78,161,255,0.85)';
    ctx.lineWidth   = Math.max(1.5, 2.5 * zoom);
    ctx.lineJoin    = 'round';
    ctx.beginPath();
    const p0 = worldToCanvas(navPath[0].x, navPath[0].y);
    ctx.moveTo(p0.x, p0.y);
    for (let i = 1; i < navPath.length; i++) {
      const p = worldToCanvas(navPath[i].x, navPath[i].y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.restore();
  }

  function drawFrontiers() {
    ctx.save();
    for (const f of frontiers) {
      const p = worldToCanvas(f.x, f.y);
      const r = Math.max(4 * zoom, Math.sqrt(f.size) * zoom * 0.8);
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle   = f.is_doorway ? 'rgba(255,107,107,0.45)' : 'rgba(255,152,0,0.4)';
      ctx.fill();
      ctx.strokeStyle = f.is_doorway ? '#ff6b6b' : '#ff9800';
      ctx.lineWidth   = 1.5;
      ctx.stroke();
    }
    ctx.restore();
  }

  function drawWaypointPins() {
    ctx.save();
    ctx.font = `bold ${Math.round(11 * Math.sqrt(zoom))}px system-ui`;
    ctx.textBaseline = 'middle';
    for (const [label, wp] of Object.entries(waypoints)) {
      const p = worldToCanvas(wp.x, wp.y);
      const r = PIN_R * Math.sqrt(zoom);
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle   = '#e040fb';
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth   = 1.5;
      ctx.stroke();
      if (zoom > 0.5) {
        ctx.fillStyle   = '#fff';
        ctx.textAlign   = 'left';
        ctx.fillText(label, p.x + r + 3, p.y);
      }
    }
    ctx.restore();
  }

  function drawObjectPins() {
    const q       = $('objSearch').value.trim().toLowerCase();
    const dOnly   = $('doorsOnly').checked;
    ctx.save();
    for (const o of objects) {
      if (typeof o.worldX !== 'number') continue;
      if (dOnly && !o.isDoor) continue;
      if (q) {
        const hay = [o.canonical, o.label, ...(o.aliases ?? [])].join(' ').toLowerCase();
        if (!hay.includes(q)) continue;
      }
      const p      = worldToCanvas(o.worldX, o.worldY);
      const active = o.id === selectedObjId;
      const r      = (active ? 9 : PIN_R) * Math.sqrt(zoom);
      ctx.beginPath();
      ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
      ctx.fillStyle   = o.isDoor ? (o._unexplored ? '#ff6b6b' : '#ffb454') : '#4ea1ff';
      ctx.fill();
      ctx.strokeStyle = active ? '#fff' : 'rgba(0,0,0,.6)';
      ctx.lineWidth   = 2;
      ctx.stroke();
      if (active && zoom > 0.5) {
        ctx.fillStyle    = '#fff';
        ctx.font         = `12px system-ui`;
        ctx.textAlign    = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(o.label ?? o.canonical, p.x + r + 4, p.y);
      }
    }
    ctx.restore();
  }

  function drawRobot() {
    const { x, y, yaw } = robotPose;
    const p = worldToCanvas(x, y);
    const r = ROBOT_RADIUS_PX * Math.sqrt(zoom);
    ctx.save();

    // Body
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fillStyle   = 'rgba(255,82,82,0.92)';
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth   = 2;
    ctx.stroke();

    // Heading arrow: canvas y is inverted vs world y
    const dx =  Math.cos(yaw) * r * 1.9;
    const dy = -Math.sin(yaw) * r * 1.9;
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(p.x + dx, p.y + dy);
    ctx.strokeStyle = '#fff';
    ctx.lineWidth   = 2.5;
    ctx.lineCap     = 'round';
    ctx.stroke();

    ctx.restore();
  }

  function drawClickTarget() {
    const p = worldToCanvas(clickTarget.x, clickTarget.y);
    const r = 9 * Math.sqrt(zoom);
    ctx.save();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth   = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#ff5252';
    ctx.beginPath();
    ctx.arc(p.x, p.y, 3 * Math.sqrt(zoom), 0, Math.PI * 2);
    ctx.fill();
    // Cross-hair lines
    ctx.strokeStyle = 'rgba(255,255,255,0.7)';
    ctx.lineWidth   = 1;
    ctx.beginPath();
    ctx.moveTo(p.x - r * 1.6, p.y); ctx.lineTo(p.x + r * 1.6, p.y);
    ctx.moveTo(p.x, p.y - r * 1.6); ctx.lineTo(p.x, p.y + r * 1.6);
    ctx.stroke();
    ctx.restore();
  }

  // ── Data refresh ───────────────────────────────────────────────────────────
  async function refreshMap() {
    try {
      const meta = await fetch('/map.json').then(r => r.json());
      mapMeta = meta;

      const img = new Image();
      await new Promise((res, rej) => {
        img.onload = res; img.onerror = rej;
        img.src = '/map.png?t=' + Date.now();
      });
      mapImg = img;
      fitMap();                 // no-op after first fit
      updateMapPixels();        // for door unexplored probe
      $('empty').classList.add('hidden');
    } catch {
      if (!mapMeta) $('empty').classList.remove('hidden');
    }
  }

  async function refreshPose() {
    try {
      const p = await ws.request('GET', '/pose');
      if (p) {
        robotPose = p;
        $('poseX').textContent   = `x: ${p.x.toFixed(3)} m`;
        $('poseY').textContent   = `y: ${p.y.toFixed(3)} m`;
        $('poseYaw').textContent = `yaw: ${(p.yaw * 180 / Math.PI).toFixed(1)}°`;
      }
    } catch { /* no TF yet */ }
  }

  async function refreshPath() {
    try {
      const res = await ws.request('GET', '/nav_path');
      if (res?.points) navPath = res.points;
    } catch {}
  }

  async function refreshScan() {
    if (!LAYERS.scan.on) return;
    try {
      const s = await ws.request('GET', '/scan');
      if (s) scanData = s;
    } catch {}
  }

  async function refreshWaypoints() {
    try {
      const res = await ws.request('GET', '/waypoints');
      if (res && typeof res === 'object') {
        waypoints = res;
        renderWaypointList();
      }
    } catch {}
  }

  async function refreshObjects() {
    try {
      const res = await ws.request('GET', '/objects');
      if (Array.isArray(res)) {
        objects = res;
        objects.forEach(o => { o._unexplored = o.isDoor ? doorHasUnexplored(o) : false; });
        renderObjectList();
      }
    } catch {}
  }

  async function refreshFrontiers() {
    if (!LAYERS.frontiers.on) return;
    try {
      const res = await ws.request('GET', '/explore/targets');
      if (res?.frontiers) {
        frontiers = res.frontiers;
        $('frontierCount').textContent = frontiers.length ? `${frontiers.length} frontiers` : '';
      }
    } catch {}
  }

  async function refreshTelemetry() {
    try {
      const res = await ws.request('GET', '/telemetry');
      if (!res) return;
      if (res.battery_v != null) { batteryV = res.battery_v; updateBattery(); }
      if (res.explore_enabled != null) { exploreEnabled = res.explore_enabled; updateExploreUI(); }
      if (res.nav_status) applyNavStatus(res.nav_status);
    } catch {}
  }

  // ── Unknown-space probe (door "leads somewhere new") ───────────────────────
  function updateMapPixels() {
    if (!mapImg || !mapMeta) return;
    const off = document.createElement('canvas');
    off.width = mapMeta.width; off.height = mapMeta.height;
    const oc = off.getContext('2d');
    oc.drawImage(mapImg, 0, 0);
    try { mapPixels = oc.getImageData(0, 0, mapMeta.width, mapMeta.height); }
    catch { mapPixels = null; }
  }

  function nativeIsUnknown(col, row) {
    if (!mapPixels) return false;
    const x = Math.round(col), y = Math.round(row);
    if (x < 0 || y < 0 || x >= mapMeta.width || y >= mapMeta.height) return false;
    return Math.abs(mapPixels.data[(y * mapMeta.width + x) * 4] - UNKNOWN_GRAY) < 10;
  }

  function doorHasUnexplored(obj) {
    if (!mapMeta) return false;
    const steps = Math.max(1, Math.round(DOOR_PROBE_M / mapMeta.resolution));
    for (let dx = -steps; dx <= steps; dx++) {
      for (let dy = -steps; dy <= steps; dy++) {
        const wx = obj.worldX + dx * mapMeta.resolution;
        const wy = obj.worldY + dy * mapMeta.resolution;
        const col = (wx - mapMeta.origin.x) / mapMeta.resolution;
        const row = (mapMeta.height - 1) - (wy - mapMeta.origin.y) / mapMeta.resolution;
        if (nativeIsUnknown(col, row)) return true;
      }
    }
    return false;
  }

  // ── UI update helpers ──────────────────────────────────────────────────────
  function setConnected(on) {
    const el = $('connDot');
    el.className = 'conn-dot ' + (on ? 'connected' : 'disconnected');
    el.title = on ? 'Connected' : 'Disconnected — reconnecting…';
  }

  function applyNavStatus(ns) {
    navStatus = ns;
    const badge = $('navBadge');
    badge.textContent = ns.status;
    badge.className   = 'nav-badge ' + ns.status.toLowerCase();
    $('navInfo').textContent = ns.target
      ? `→ ${ns.target.x.toFixed(2)}, ${ns.target.y.toFixed(2)}`
      : (ns.status === 'IDLE' ? 'Click map to set goal' : '');
    $('navElapsed').textContent = (ns.elapsed_s > 0)
      ? `Elapsed: ${ns.elapsed_s.toFixed(0)}s` : '';
    $('navTarget').textContent = ns.target
      ? `x:${ns.target.x.toFixed(2)} y:${ns.target.y.toFixed(2)}` : '';
    $('btnCancelNav').disabled = ns.status !== 'NAVIGATING';
    if (ns.status !== 'NAVIGATING') clickTarget = null;
  }

  function updateBattery() {
    if (batteryV == null) return;
    $('batteryVal').textContent = batteryV.toFixed(1) + 'V';
    const pct  = Math.max(0, Math.min(1, (batteryV - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V)));
    const fill = $('batteryFill');
    fill.style.width      = (pct * 100) + '%';
    fill.style.background = pct > 0.5 ? '#57d38c' : pct > 0.25 ? '#ffb454' : '#ff6b6b';
    const el = $('batteryVal');
    el.style.color = pct < 0.15 ? '#ff6b6b' : '';
  }

  function updateExploreUI() {
    $('exploreStatus').textContent  = exploreEnabled ? '● Exploring' : 'Disabled';
    $('exploreStatus').className    = 'info-line mt-4 ' + (exploreEnabled ? 'ok' : 'muted');
  }

  // ── Right-panel list rendering ─────────────────────────────────────────────
  function renderWaypointList() {
    const list    = $('wpList');
    const entries = Object.entries(waypoints);
    $('wpCount').textContent = entries.length ? `(${entries.length})` : '';
    list.innerHTML = '';
    for (const [label, wp] of entries) {
      const li    = document.createElement('li');
      li.className = 'item';

      const dot  = document.createElement('span');
      dot.className = 'dot wp-dot';

      const name = document.createElement('span');
      name.className = 'item-name';
      name.textContent = label;

      const meta = document.createElement('span');
      meta.className = 'item-meta';
      meta.textContent = `${wp.x.toFixed(1)}, ${wp.y.toFixed(1)}`;

      const goBtn = document.createElement('button');
      goBtn.className = 'btn-xs btn-accent';
      goBtn.textContent = '▶';
      goBtn.title = 'Go to ' + label;
      goBtn.addEventListener('click', e => { e.stopPropagation(); gotoWaypoint(label); });

      const delBtn = document.createElement('button');
      delBtn.className = 'btn-xs btn-danger';
      delBtn.textContent = '✕';
      delBtn.title = 'Delete ' + label;
      delBtn.addEventListener('click', e => { e.stopPropagation(); deleteWaypoint(label); });

      li.append(dot, name, meta, goBtn, delBtn);

      // Click row → pan map to that waypoint
      li.addEventListener('click', () => panToWorld(wp.x, wp.y));
      list.appendChild(li);
    }
  }

  function renderObjectList() {
    const q       = $('objSearch').value.trim().toLowerCase();
    const dOnly   = $('doorsOnly').checked;
    const filtered = objects.filter(o => {
      if (dOnly && !o.isDoor) return false;
      if (!q) return true;
      return [o.canonical, o.label, ...(o.aliases ?? [])].join(' ').toLowerCase().includes(q);
    });
    $('objCount').textContent = filtered.length ? `(${filtered.length})` : '';
    const list = $('objList');
    list.innerHTML = '';
    for (const o of filtered) {
      const li = document.createElement('li');
      li.className = 'item' + (o.id === selectedObjId ? ' active' : '');

      const dot = document.createElement('span');
      dot.className = 'dot' + (o.isDoor ? (o._unexplored ? ' door-open' : ' door') : '');

      const name = document.createElement('span');
      name.className = 'item-name';
      name.textContent = o.label ?? o.canonical;

      const meta = document.createElement('span');
      meta.className = 'item-meta';
      meta.textContent = o.isDoor ? (o._unexplored ? 'door·new' : 'door') : (o.canonical ?? '');

      li.append(dot, name, meta);
      li.addEventListener('click', () => selectObject(o.id));
      list.appendChild(li);
    }
  }

  function selectObject(id) {
    selectedObjId = id;
    const o = objects.find(x => x.id === id);
    if (!o) { $('detail').classList.add('hidden'); renderObjectList(); return; }

    $('detail').classList.remove('hidden');
    const thumb = $('detailThumb');
    if (o.thumb) {
      thumb.src = o.thumb + '?t=' + Date.now();
      thumb.classList.remove('hidden');
    } else {
      thumb.classList.add('hidden');
    }
    const aliases = (o.aliases ?? []).join(', ');
    $('detailBody').innerHTML =
      `<div class="dt">${esc(o.label ?? o.canonical)}</div>` +
      `<div class="dk">canonical: ${esc(o.canonical ?? '')}</div>` +
      (aliases ? `<div class="dk">also: ${esc(aliases)}</div>` : '') +
      `<div class="dk">map: ${fmt(o.worldX)}, ${fmt(o.worldY)}</div>` +
      (o.isDoor ? `<div class="dk">${o._unexplored ? 'door — unexplored beyond' : 'door'}</div>` : '');

    if (typeof o.worldX === 'number') panToWorld(o.worldX, o.worldY);
    renderObjectList();
  }

  // ── Navigation + waypoint actions ──────────────────────────────────────────
  async function sendGoal(wx, wy) {
    // Yaw faces the direction of travel from robot's current position
    let yaw = 0;
    if (robotPose) yaw = Math.atan2(wy - robotPose.y, wx - robotPose.x);
    clickTarget = { x: wx, y: wy };
    try {
      await ws.request('POST', '/goal', { x: wx, y: wy, yaw });
    } catch (e) {
      console.warn('goal failed:', e.message);
      clickTarget = null;
    }
  }

  async function cancelGoal() {
    clickTarget = null;
    try { await ws.request('POST', '/goal/cancel', {}); } catch {}
  }

  async function gotoWaypoint(label) {
    try { await ws.request('POST', '/waypoint/goto', { label }); }
    catch (e) { console.warn('goto failed:', e.message); }
  }

  async function saveWaypoint() {
    const label = $('wpName').value.trim();
    if (!label) return;
    try {
      await ws.request('POST', '/waypoint/save', { label });
      $('wpName').value = '';
      await refreshWaypoints();
    } catch (e) { console.warn('waypoint save failed:', e.message); }
  }

  async function deleteWaypoint(label) {
    if (!confirm(`Delete waypoint "${label}"?`)) return;
    try {
      await ws.request('POST', '/waypoint/delete', { label });
      await refreshWaypoints();
    } catch (e) { console.warn('waypoint delete failed:', e.message); }
  }

  // ── View helpers ───────────────────────────────────────────────────────────
  function panToWorld(wx, wy) {
    if (!mapMeta) return;
    const p = worldToCanvas(wx, wy);
    const s = $('stage');
    panX += s.clientWidth  / 2 - p.x;
    panY += s.clientHeight / 2 - p.y;
  }

  // ── Layer checkboxes ───────────────────────────────────────────────────────
  function initLayers() {
    const container = $('layerToggles');
    for (const [key, layer] of Object.entries(LAYERS)) {
      const label = document.createElement('label');
      label.className = 'chk layer-chk';
      const cb = document.createElement('input');
      cb.type    = 'checkbox';
      cb.checked = layer.on;
      cb.id      = 'layer-' + key;
      cb.addEventListener('change', () => {
        layer.on = cb.checked;
        if (key === 'scan'      && layer.on) refreshScan();
        if (key === 'frontiers' && layer.on) refreshFrontiers();
      });
      label.append(cb, document.createTextNode(' ' + layer.label));
      container.appendChild(label);
    }
  }

  // ── Canvas interaction (click, zoom, pan) ──────────────────────────────────
  function initCanvas() {
    const stage = $('stage');

    canvas.addEventListener('click', e => {
      if (isPanning) return; // was a drag, ignore click

      const rect = canvas.getBoundingClientRect();
      const cx   = e.clientX - rect.left;
      const cy   = e.clientY - rect.top;

      // Hit-test waypoint pins first
      for (const [label, wp] of Object.entries(waypoints)) {
        const p = worldToCanvas(wp.x, wp.y);
        if (Math.hypot(p.x - cx, p.y - cy) < (PIN_R + 4) * Math.sqrt(zoom)) {
          gotoWaypoint(label); return;
        }
      }

      // Hit-test object pins
      for (const o of objects) {
        if (typeof o.worldX !== 'number') continue;
        const p = worldToCanvas(o.worldX, o.worldY);
        if (Math.hypot(p.x - cx, p.y - cy) < (PIN_R + 6) * Math.sqrt(zoom)) {
          selectObject(o.id); return;
        }
      }

      // Otherwise navigate
      const world = canvasToWorld(cx, cy);
      if (world) sendGoal(world.x, world.y);
    });

    // Right-click → cancel / deselect
    canvas.addEventListener('contextmenu', e => {
      e.preventDefault();
      clickTarget = null;
      if (selectedObjId) {
        selectedObjId = null;
        $('detail').classList.add('hidden');
        renderObjectList();
      } else {
        cancelGoal();
      }
    });

    // Scroll to zoom centred on cursor
    stage.addEventListener('wheel', e => {
      e.preventDefault();
      const rect   = canvas.getBoundingClientRect();
      const mx     = e.clientX - rect.left;
      const my     = e.clientY - rect.top;
      const factor = e.deltaY < 0 ? 1.15 : 1 / 1.15;
      panX = mx + (panX - mx) * factor;
      panY = my + (panY - my) * factor;
      zoom = Math.max(0.15, Math.min(20, zoom * factor));
    }, { passive: false });

    // Middle-button or Shift+left → pan
    let dragMoved = false;
    canvas.addEventListener('mousedown', e => {
      if (e.button === 1 || (e.button === 0 && e.shiftKey)) {
        isPanning  = true;
        dragMoved  = false;
        panStart   = { x: e.clientX - panX, y: e.clientY - panY };
        canvas.style.cursor = 'grabbing';
        e.preventDefault();
      }
    });
    window.addEventListener('mousemove', e => {
      if (!isPanning) return;
      const newPx = e.clientX - panStart.x;
      const newPy = e.clientY - panStart.y;
      if (Math.abs(newPx - panX) > 2 || Math.abs(newPy - panY) > 2) dragMoved = true;
      panX = newPx; panY = newPy;
    });
    window.addEventListener('mouseup', () => {
      if (isPanning) {
        isPanning = false;
        canvas.style.cursor = 'crosshair';
        // Let the click handler ignore this if it was a drag
        setTimeout(() => { isPanning = false; }, 50);
      }
    });

    // Touch pinch-zoom
    let touchDist = null;
    canvas.addEventListener('touchstart', e => {
      if (e.touches.length === 2) {
        const t = e.touches;
        touchDist = Math.hypot(t[0].clientX - t[1].clientX, t[0].clientY - t[1].clientY);
      }
    }, { passive: true });
    canvas.addEventListener('touchmove', e => {
      if (e.touches.length === 2 && touchDist != null) {
        e.preventDefault();
        const t = e.touches;
        const d = Math.hypot(t[0].clientX - t[1].clientX, t[0].clientY - t[1].clientY);
        zoom = Math.max(0.15, Math.min(20, zoom * (d / touchDist)));
        touchDist = d;
      }
    }, { passive: false });
    canvas.addEventListener('touchend', () => { touchDist = null; }, { passive: true });

    // Resize observer
    new ResizeObserver(() => { resizeCanvas(); fitMap(); }).observe(stage);

    // 'F' key to fit
    window.addEventListener('keydown', e => {
      if (e.key === 'f' || e.key === 'F') fitMap(true);
      if (e.key === 'Escape') {
        clickTarget = null;
        selectedObjId = null;
        $('detail').classList.add('hidden');
        renderObjectList();
      }
    });
  }

  // ── Button event wiring ────────────────────────────────────────────────────
  function initControls() {
    $('btnFreeze').addEventListener('click', () => ws.command('freeze'));

    $('btnExploreOn').addEventListener('click', () => {
      ws.command('explore', { enable: true });
      exploreEnabled = true;
      updateExploreUI();
    });
    $('btnExploreOff').addEventListener('click', () => {
      ws.command('explore', { enable: false });
      exploreEnabled = false;
      updateExploreUI();
    });

    $('btnCancelNav').addEventListener('click', cancelGoal);
    $('btnWpSave').addEventListener('click', saveWaypoint);
    $('wpName').addEventListener('keydown', e => { if (e.key === 'Enter') saveWaypoint(); });

    $('btnFitMap').addEventListener('click', () => fitMap(true));

    $('btnSlamSave').addEventListener('click', async () => {
      const btn = $('btnSlamSave');
      btn.disabled = true;
      btn.textContent = '⏳ Saving…';
      try {
        const res = await ws.request('POST', '/slam/save', {});
        btn.textContent = res?.result === 'ok' ? '✓ Saved!' : '✗ Failed';
      } catch { btn.textContent = '✗ Error'; }
      setTimeout(() => { btn.disabled = false; btn.textContent = '💾 Save Map'; }, 3000);
    });

    $('btnSlamReset').addEventListener('click', async () => {
      if (!confirm('Reset the SLAM map? This clears ALL mapping data and cannot be undone.')) return;
      const btn = $('btnSlamReset');
      btn.disabled = true;
      btn.textContent = '⏳ Resetting…';
      try { await ws.request('POST', '/slam/reset', {}); }
      catch {}
      setTimeout(() => { btn.disabled = false; btn.textContent = '↺ Reset Map'; }, 3000);
    });

    $('detailClose').addEventListener('click', () => {
      selectedObjId = null;
      $('detail').classList.add('hidden');
      renderObjectList();
    });

    $('objSearch').addEventListener('input',  () => { renderObjectList(); });
    $('doorsOnly').addEventListener('change', () => { renderObjectList(); });
  }

  // ── WebSocket push event handlers ──────────────────────────────────────────
  function initWSEvents() {
    ws.on('nav_status', msg => {
      applyNavStatus({ status: msg.status, target: msg.target, elapsed_s: msg.elapsed_s ?? 0 });
    });
    ws.on('telemetry', msg => {
      if (msg.battery_v != null)      { batteryV = msg.battery_v; updateBattery(); }
      if (msg.explore_enabled != null){ exploreEnabled = msg.explore_enabled; updateExploreUI(); }
      if (msg.nav_status)             applyNavStatus(msg.nav_status);
    });
    ws.on('open', () => {
      // Immediately hydrate on (re)connect
      Promise.allSettled([
        refreshTelemetry(),
        refreshWaypoints(),
        refreshObjects(),
        refreshFrontiers(),
      ]);
    });
  }

  // ── Refresh loops ──────────────────────────────────────────────────────────
  function startLoops() {
    // Animation frame loop (draw only — no data fetching here)
    const animLoop = () => { draw(); requestAnimationFrame(animLoop); };
    animLoop();

    // Map PNG + JSON (slow — it's a big fetch)
    const mapLoop = async () => {
      await refreshMap();
      setTimeout(mapLoop, REFRESH_MAP_MS);
    };
    mapLoop();

    // Fast: pose, path, scan
    const fastLoop = async () => {
      await Promise.allSettled([refreshPose(), refreshPath(), refreshScan()]);
      setTimeout(fastLoop, REFRESH_FAST_MS);
    };
    setTimeout(fastLoop, 300); // slight delay so WS connects first

    // Slow: waypoints, objects, frontiers
    const slowLoop = async () => {
      await Promise.allSettled([refreshWaypoints(), refreshObjects(), refreshFrontiers()]);
      setTimeout(slowLoop, REFRESH_SLOW_MS);
    };
    setTimeout(slowLoop, 600);

    // Telemetry backup poll (bridge also pushes every 2s via WS event)
    setInterval(refreshTelemetry, 6000);
  }

  // ── Utilities ──────────────────────────────────────────────────────────────
  function fmt(v) { return typeof v === 'number' ? v.toFixed(2) : '—'; }
  function esc(s) {
    return String(s).replace(/[&<>"]/g,
      c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]);
  }

  // ── Boot ───────────────────────────────────────────────────────────────────
  function init() {
    resizeCanvas();
    initLayers();
    initCanvas();
    initControls();
    initWSEvents();
    setConnected(false);
    applyNavStatus(navStatus);
    updateExploreUI();
    startLoops();
  }

  init();
})();

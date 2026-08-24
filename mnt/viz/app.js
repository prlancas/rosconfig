/*
 * Droidal object-map visualiser.
 *
 * Served by android_bridge.py's HTTP server. Draws the SLAM occupancy grid
 * (GET /map.png + GET /map.json) and overlays the object landmarks the phone
 * pushed (GET /objects), with search, thumbnails, and door / "unexplored
 * beyond" markers. No build step, no dependencies — plain canvas + fetch.
 */
(() => {
  "use strict";

  const REFRESH_MS = 4000;
  const UNKNOWN_GRAY = 205;      // matches android_bridge map_png() unknown value
  const DOOR_PROBE_M = 1.2;      // how far past a door we look for unknown space

  const els = {
    canvas: document.getElementById("map"),
    empty: document.getElementById("empty"),
    status: document.getElementById("status"),
    search: document.getElementById("search"),
    doorsOnly: document.getElementById("doorsOnly"),
    list: document.getElementById("list"),
    count: document.getElementById("count"),
    detail: document.getElementById("detail"),
    detailThumb: document.getElementById("detailThumb"),
    detailBody: document.getElementById("detailBody"),
    detailClose: document.getElementById("detailClose"),
  };
  const ctx = els.canvas.getContext("2d");

  let meta = null;          // {resolution,width,height,origin:{x,y,yaw}}
  let mapImage = null;      // <img> of /map.png
  let mapPixels = null;     // ImageData of the native-res map (for unknown probe)
  let objects = [];
  let selectedId = null;
  let scale = 1;

  const DISPLAY_MAX = 1000; // longest displayed edge in px

  function setStatus(text, cls) {
    els.status.textContent = text;
    els.status.className = "status" + (cls ? " " + cls : "");
  }

  // --- world <-> pixel ------------------------------------------------------
  // map.png is north-up (android_bridge flips rows), so native row 0 is max-y.
  function worldToNative(x, y) {
    const col = (x - meta.origin.x) / meta.resolution;
    const rowFromBottom = (y - meta.origin.y) / meta.resolution;
    const row = (meta.height - 1) - rowFromBottom;
    return { px: col, py: row };
  }

  function nativeIsUnknown(px, py) {
    if (!mapPixels) return false;
    const x = Math.round(px), y = Math.round(py);
    if (x < 0 || y < 0 || x >= meta.width || y >= meta.height) return false;
    const i = (y * meta.width + x) * 4;
    return Math.abs(mapPixels.data[i] - UNKNOWN_GRAY) < 10;
  }

  // A door "leads somewhere new" if there's unknown map within DOOR_PROBE_M.
  function doorHasUnexplored(obj) {
    if (!meta) return false;
    const stepM = meta.resolution;
    const steps = Math.max(1, Math.round(DOOR_PROBE_M / stepM));
    for (let dx = -steps; dx <= steps; dx++) {
      for (let dy = -steps; dy <= steps; dy++) {
        const wx = obj.worldX + dx * stepM;
        const wy = obj.worldY + dy * stepM;
        const p = worldToNative(wx, wy);
        if (nativeIsUnknown(p.px, p.py)) return true;
      }
    }
    return false;
  }

  // --- data fetching --------------------------------------------------------
  async function loadMap() {
    const r = await fetch("/map.json");
    if (!r.ok) throw new Error("map " + r.status);
    meta = await r.json();

    const img = new Image();
    await new Promise((res, rej) => {
      img.onload = res;
      img.onerror = rej;
      img.src = "/map.png?t=" + Date.now();
    });
    mapImage = img;

    // Grab native pixels once for the unexplored-beyond probe.
    const off = document.createElement("canvas");
    off.width = meta.width;
    off.height = meta.height;
    const octx = off.getContext("2d");
    octx.drawImage(img, 0, 0);
    try {
      mapPixels = octx.getImageData(0, 0, meta.width, meta.height);
    } catch (e) {
      mapPixels = null; // e.g. cross-origin; probe just disabled
    }

    scale = Math.min(1, DISPLAY_MAX / Math.max(meta.width, meta.height));
    els.canvas.width = Math.round(meta.width * scale);
    els.canvas.height = Math.round(meta.height * scale);
  }

  async function loadObjects() {
    const r = await fetch("/objects?t=" + Date.now());
    if (!r.ok) throw new Error("objects " + r.status);
    objects = await r.json();
    objects.forEach((o) => { o._unexplored = o.isDoor ? doorHasUnexplored(o) : false; });
  }

  // --- rendering ------------------------------------------------------------
  function filtered() {
    const q = els.search.value.trim().toLowerCase();
    return objects.filter((o) => {
      if (els.doorsOnly.checked && !o.isDoor) return false;
      if (!q) return true;
      const hay = [o.canonical, o.label, ...(o.aliases || [])].join(" ").toLowerCase();
      return hay.includes(q);
    });
  }

  function pinColor(o) {
    if (o.isDoor) return o._unexplored ? "#ff6b6b" : "#ffb454";
    return "#4ea1ff";
  }

  function draw() {
    if (!meta || !mapImage) return;
    ctx.clearRect(0, 0, els.canvas.width, els.canvas.height);
    ctx.drawImage(mapImage, 0, 0, els.canvas.width, els.canvas.height);

    for (const o of filtered()) {
      if (typeof o.worldX !== "number") continue;
      const p = worldToNative(o.worldX, o.worldY);
      const x = p.px * scale, y = p.py * scale;
      const active = o.id === selectedId;
      ctx.beginPath();
      ctx.arc(x, y, active ? 8 : 5, 0, Math.PI * 2);
      ctx.fillStyle = pinColor(o);
      ctx.fill();
      ctx.lineWidth = 2;
      ctx.strokeStyle = active ? "#fff" : "rgba(0,0,0,.6)";
      ctx.stroke();
      if (active) {
        ctx.fillStyle = "#fff";
        ctx.font = "12px system-ui";
        ctx.fillText(o.label || o.canonical, x + 10, y + 4);
      }
    }
  }

  function renderList() {
    const items = filtered();
    els.count.textContent = "(" + items.length + ")";
    els.list.innerHTML = "";
    for (const o of items) {
      const li = document.createElement("li");
      li.dataset.id = o.id;
      if (o.id === selectedId) li.classList.add("active");
      const dot = document.createElement("span");
      dot.className = "dot" + (o.isDoor ? (o._unexplored ? " door-open" : " door") : "");
      const name = document.createElement("span");
      name.className = "name";
      name.textContent = o.label || o.canonical;
      const metaEl = document.createElement("span");
      metaEl.className = "meta";
      metaEl.textContent = o.isDoor ? (o._unexplored ? "door · unexplored" : "door") : o.canonical;
      li.append(dot, name, metaEl);
      li.addEventListener("click", () => select(o.id));
      els.list.appendChild(li);
    }
  }

  function select(id) {
    selectedId = id;
    const o = objects.find((x) => x.id === id);
    draw();
    renderList();
    if (!o) { els.detail.classList.add("hidden"); return; }
    els.detail.classList.remove("hidden");
    if (o.thumb) {
      els.detailThumb.src = o.thumb + "?t=" + Date.now();
      els.detailThumb.classList.remove("hidden");
    } else {
      els.detailThumb.classList.add("hidden");
    }
    const aliases = (o.aliases || []).join(", ");
    els.detailBody.innerHTML =
      `<div class="t">${escapeHtml(o.label || o.canonical)}</div>` +
      `<div class="k">canonical: ${escapeHtml(o.canonical || "")}</div>` +
      (aliases ? `<div class="k">also: ${escapeHtml(aliases)}</div>` : "") +
      `<div class="k">map: ${fmt(o.worldX)}, ${fmt(o.worldY)}</div>` +
      (o.isDoor ? `<div class="k">${o._unexplored ? "door — unexplored area beyond" : "door"}</div>` : "");
  }

  function fmt(v) { return typeof v === "number" ? v.toFixed(2) : "?"; }
  function escapeHtml(s) {
    return String(s).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
  }

  // --- canvas click -> nearest pin -----------------------------------------
  els.canvas.addEventListener("click", (e) => {
    const rect = els.canvas.getBoundingClientRect();
    const mx = (e.clientX - rect.left);
    const my = (e.clientY - rect.top);
    let best = null, bestD = 14;
    for (const o of filtered()) {
      if (typeof o.worldX !== "number") continue;
      const p = worldToNative(o.worldX, o.worldY);
      const d = Math.hypot(p.px * scale - mx, p.py * scale - my);
      if (d < bestD) { bestD = d; best = o; }
    }
    if (best) select(best.id);
  });

  els.detailClose.addEventListener("click", () => { selectedId = null; els.detail.classList.add("hidden"); draw(); renderList(); });
  els.search.addEventListener("input", () => { draw(); renderList(); });
  els.doorsOnly.addEventListener("change", () => { draw(); renderList(); });

  // --- loops ----------------------------------------------------------------
  async function refresh() {
    try {
      if (!meta || !mapImage) await loadMap();
      else {
        // Re-pull the map periodically too (it grows during mapping).
        await loadMap();
      }
      els.empty.classList.add("hidden");
      await loadObjects();
      setStatus(objects.length + " objects · map " + meta.width + "×" + meta.height, "ok");
      draw();
      renderList();
    } catch (e) {
      if (!meta) els.empty.classList.remove("hidden");
      setStatus("waiting for map/robot… (" + e.message + ")", "err");
    }
  }

  refresh();
  setInterval(refresh, REFRESH_MS);
})();

/* Moon Debug Station — read-only UI (+ ZED process control) */

function fmt(n, d = 2) {
  if (n === null || n === undefined || Number.isNaN(n)) return "—";
  return Number(n).toFixed(d);
}

function age(s) {
  if (s === undefined || s === null || s > 100) return "stale";
  return s.toFixed(2) + "s";
}

function setLayer(key, info) {
  const el = document.querySelector(`.layer[data-key="${key}"]`);
  if (!el || !info) return;
  el.classList.remove("green", "yellow", "red");
  el.classList.add(info.status || "red");
  el.querySelector("small").textContent = info.detail || "—";
}

function kv(el, rows) {
  el.innerHTML = rows
    .map(
      ([k, v]) =>
        `<div class="item"><label>${k}</label><span>${v}</span></div>`
    )
    .join("");
}

function renderObs(o, a) {
  const bars = document.getElementById("obs-bars");
  if (!o || Object.keys(o).length === 0) {
    bars.innerHTML = `<div class="bar"><div class="v">—</div><div class="l">no data</div></div>`;
  } else {
    bars.innerHTML = ["left_m", "center_m", "right_m"]
      .map((k) => {
        const label = k.replace("_m", "");
        return `<div class="bar"><div class="v">${fmt(o[k], 2)}</div><div class="l">${label} m</div></div>`;
      })
      .join("");
  }
  document.getElementById("obs-json").textContent = JSON.stringify(
    { ...o, age: age(a) },
    null,
    2
  );
}

function renderZed(z) {
  const st = document.getElementById("zed-status");
  if (!z) {
    st.textContent = "状态: —";
    st.className = "zed-status";
    return;
  }
  const on = !!z.running;
  st.textContent = on
    ? "状态: 开启" + (z.pids?.length ? ` (pid ${z.pids.join(",")})` : "")
    : "状态: 关闭";
  st.className = "zed-status " + (on ? "on" : "off");
}

async function zedAction(which) {
  const onBtn = document.getElementById("btn-zed-on");
  const offBtn = document.getElementById("btn-zed-off");
  const msg = document.getElementById("zed-msg");
  onBtn.disabled = true;
  offBtn.disabled = true;
  msg.textContent = which === "start" ? "正在开启 ZED…" : "正在关闭 ZED…";
  try {
    const r = await fetch(`/api/zed/${which}`, { method: "POST" });
    const j = await r.json();
    msg.textContent = j.msg || JSON.stringify(j);
    renderZed(j);
    const img = document.getElementById("fpv");
    if (which === "start" && j.ok) {
      img.dataset.off = "0";
      img.src = `/fpv/stream.mjpg?t=${Date.now()}`;
    }
    if (which === "stop") {
      img.dataset.off = "1";
      img.removeAttribute("src");
    }
  } catch (e) {
    msg.textContent = "请求失败: " + e;
  } finally {
    onBtn.disabled = false;
    offBtn.disabled = false;
  }
}

document.getElementById("btn-zed-on").addEventListener("click", () => zedAction("start"));
document.getElementById("btn-zed-off").addEventListener("click", () => zedAction("stop"));

async function tick() {
  try {
    const r = await fetch("/api/snapshot", { cache: "no-store" });
    const s = await r.json();

    document.getElementById("meta").innerHTML =
      `ros=${s.ros_ok ? "ok" : "off"} · t=${new Date(s.t * 1000).toLocaleTimeString()}` +
      `<br/>log=${(s.uwb_log_path || "").split("/").pop() || "—"}`;

    setLayer("perception", s.layers?.perception);
    setLayer("decision", s.layers?.decision);
    setLayer("execution", s.layers?.execution);

    renderZed(s.zed);

    const fpvUrl = `/fpv/stream.mjpg`;
    const img = document.getElementById("fpv");
    if (s.zed?.running) {
      if (!img.src || img.dataset.off === "1" || !String(img.src).includes("/fpv/stream")) {
        img.dataset.off = "0";
        img.src = fpvUrl + "?t=" + Date.now();
      }
    }
    document.getElementById("fpv-link").innerHTML =
      `同端口代理 <a href="/fpv/" target="_blank" rel="noopener">/fpv/</a> · 上游 :8080`;

    renderObs(s.obstacle, s.obstacle_age);

    kv(document.getElementById("uwb-kv"), [
      ["dist cm", fmt(s.uwb?.dist_cm, 1)],
      ["ang °", fmt(s.uwb?.ang_deg, 1)],
      ["age", age(s.uwb_age)],
      ["source", s.uwb?.source || "—"],
    ]);
    document.getElementById("uwb-json").textContent = JSON.stringify(s.uwb || {}, null, 2);

    kv(document.getElementById("dec-kv"), [
      ["ctrl", s.decision?.ctrl || "—"],
      ["fwd", fmt(s.decision?.fwd, 3)],
      ["rot", fmt(s.decision?.rot, 3)],
      ["gate", s.decision?.gate || "—"],
      ["soft", fmt(s.decision?.soft, 2)],
      ["cmd_vx", fmt(s.decision?.cmd_vx, 3)],
    ]);
    document.getElementById("dec-json").textContent = JSON.stringify(s.decision || {}, null, 2);

    const fsmMap = {
      5: "EXEC_DEFAULT",
      8: "PROTECTION_SHUTDOWN",
    };
    kv(document.getElementById("exec-kv"), [
      ["fsm", s.fsm_state == null ? "—" : `${s.fsm_state} ${fsmMap[s.fsm_state] || ""}`],
      ["cmd hz", fmt(s.cmd_hz, 1)],
      ["vx", fmt(s.cmd_vel?.vx, 3)],
      ["wz", fmt(s.cmd_vel?.wz, 3)],
      ["cmd age", age(s.cmd_age)],
      ["joy lv", fmt(s.joy_msg?.l_vertical, 3)],
    ]);
    document.getElementById("exec-json").textContent = JSON.stringify(
      { fsm: s.fsm_state, cmd_vel: s.cmd_vel, joy_msg: s.joy_msg },
      null,
      2
    );

    document.getElementById("events").textContent = (s.events || []).join("\n") || "(no events)";
  } catch (e) {
    document.getElementById("meta").textContent = "API error: " + e;
  }

  try {
    const lr = await fetch("/api/logs?n=80", { cache: "no-store" });
    const lj = await lr.json();
    document.getElementById("log").textContent =
      (lj.lines || []).join("\n") || "(no log lines)";
  } catch (_) {
    /* ignore */
  }
}

tick();
setInterval(tick, 200);

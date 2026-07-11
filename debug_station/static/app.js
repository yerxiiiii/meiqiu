/* Moon 语音链路测试上位机 */

function fmt(n, d = 2) {
  if (n === null || n === undefined || Number.isNaN(n)) return "—";
  return Number(n).toFixed(d);
}

function age(s) {
  if (s === undefined || s === null || s > 100) return "stale";
  return s.toFixed(2) + "s";
}

function setStep(key, info) {
  const el = document.querySelector(`.chain-step[data-key="${key}"]`);
  if (!el || !info) return;
  el.classList.remove("green", "yellow", "red");
  el.classList.add(info.status || "red");
  el.querySelector("small").textContent = info.detail || "—";
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
    .map(([k, v]) => `<div class="item"><label>${k}</label><span>${v}</span></div>`)
    .join("");
}

let quickActions = [];

function renderQuickButtons(groups) {
  const root = document.getElementById("quick-buttons");
  if (!groups || !groups.length) {
    root.innerHTML = "<p class='hint'>无配置</p>";
    return;
  }
  quickActions = [];
  let html = "";
  groups.forEach((g) => {
    html += `<div class="quick-group"><div class="quick-title">${g.group || "命令"}</div><div class="quick-row">`;
    (g.items || []).forEach((it) => {
      const idx = quickActions.length;
      quickActions.push(it);
      html += `<button type="button" class="btn quick-btn" data-idx="${idx}">${it.label}</button>`;
    });
    html += "</div></div>";
  });
  root.innerHTML = html;

  root.querySelectorAll(".quick-btn").forEach((btn) => {
    btn.addEventListener("click", async () => {
      const action = quickActions[Number(btn.dataset.idx)];
      if (!action) return;
      if (action.type === "snippet") {
        document.getElementById("terminal-in").value = action.data || "";
        document.getElementById("terminal-in").focus();
        return;
      }
      btn.disabled = true;
      try {
        const r = await fetch("/api/cmd/action", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(action),
        });
        const j = await r.json();
        await refreshTerminal();
        if (!j.ok) alert(j.msg || "失败");
      } catch (e) {
        alert("请求失败: " + e);
      } finally {
        btn.disabled = false;
      }
    });
  });
}

function renderProcs(procs) {
  const el = document.getElementById("proc-list");
  const names = {
    kws_trigger: "KWS (guide)",
    kws_brain: "KWS (brain)",
    mode_arbiter: "mode_arbiter",
    guide_demo: "guide_demo",
    uwb_follow: "uwb_follow",
    zed_obstacle: "zed_obstacle",
  };
  el.innerHTML = Object.entries(names)
    .map(([k, label]) => {
      const p = (procs || {})[k] || {};
      const on = !!p.running;
      return `<div class="proc ${on ? "on" : "off"}"><span class="pdot"></span>${label}</div>`;
    })
    .join("");
}

function renderImu(imu, imuAge) {
  const grid = document.getElementById("imu-grid");
  const bars = document.getElementById("imu-bars");
  if (!imu || !Object.keys(imu).length) {
    grid.innerHTML = "<p class='hint'>无 IMU 数据（检查 yesense / /imu/data）</p>";
    bars.innerHTML = "";
    document.getElementById("imu-json").textContent = "—";
    return;
  }
  kv(grid, [
    ["roll °", fmt(imu.roll_deg, 1)],
    ["pitch °", fmt(imu.pitch_deg, 1)],
    ["yaw °", fmt(imu.yaw_deg, 1)],
    ["age", age(imuAge)],
  ]);
  const acc = ["ax", "ay", "az"].map((k) => Math.abs(imu[k] || 0));
  const gyr = ["gx", "gy", "gz"].map((k) => Math.abs(imu[k] || 0));
  bars.innerHTML =
    `<div class="bar-group"><div class="bar-title">线性加速度</div>` +
    ["ax", "ay", "az"]
      .map((k, i) => {
        const v = imu[k] || 0;
        const pct = Math.min(100, acc[i] * 20);
        return `<div class="bar-row"><span>${k}</span><div class="bar-track"><div class="bar-fill" style="width:${pct}%"></div></div><span>${fmt(v, 2)}</span></div>`;
      })
      .join("") +
    `</div><div class="bar-group"><div class="bar-title">角速度</div>` +
    ["gx", "gy", "gz"]
      .map((k, i) => {
        const v = imu[k] || 0;
        const pct = Math.min(100, gyr[i] * 30);
        return `<div class="bar-row"><span>${k}</span><div class="bar-track"><div class="bar-fill gyro" style="width:${pct}%"></div></div><span>${fmt(v, 2)}</span></div>`;
      })
      .join("") +
    "</div>";
  document.getElementById("imu-json").textContent = JSON.stringify({ ...imu, age: imuAge }, null, 2);
}

function renderMic(mic, chain) {
  const rms = Number(mic?.rms) || 0;
  const fill = document.getElementById("mic-fill");
  const pct = Math.min(100, rms * 500);
  fill.style.width = pct + "%";
  fill.className = "mic-fill" + (chain?.mic?.open ? " open" : "");
  document.getElementById("mic-label").textContent =
    `RMS ${rms.toFixed(4)} · ${chain?.mic?.open ? "开麦" : "关麦/待机"}`;
}

async function refreshTerminal() {
  try {
    const r = await fetch("/api/terminal?n=120", { cache: "no-store" });
    const j = await r.json();
    const out = document.getElementById("terminal-out");
    out.textContent = (j.lines || []).join("\n") || "(终端就绪)";
    out.scrollTop = out.scrollHeight;
  } catch (_) {
    /* ignore */
  }
}

document.getElementById("terminal-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const inp = document.getElementById("terminal-in");
  const line = inp.value.trim();
  if (!line) return;
  inp.value = "";
  try {
    const r = await fetch("/api/cmd", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ cmd: line }),
    });
    const j = await r.json();
    await refreshTerminal();
    if (!j.ok && j.msg) {
      /* already logged */
    }
  } catch (err) {
    alert("终端错误: " + err);
  }
});

function renderZed(z) {
  const st = document.getElementById("zed-status");
  if (!z) {
    st.textContent = "—";
    return;
  }
  st.textContent = z.running ? "ZED 开启" : "ZED 关闭";
  st.className = "zed-status " + (z.running ? "on" : "off");
}

async function zedAction(which) {
  const msg = document.getElementById("zed-status");
  try {
    const r = await fetch(`/api/zed/${which}`, { method: "POST" });
    const j = await r.json();
    msg.textContent = j.msg || JSON.stringify(j);
    renderZed(j);
    const img = document.getElementById("fpv");
    if (which === "start" && j.ok) {
      img.src = `/fpv/stream.mjpg?t=${Date.now()}`;
    }
    if (which === "stop") img.removeAttribute("src");
  } catch (e) {
    msg.textContent = "失败: " + e;
  }
}

document.getElementById("btn-zed-on").addEventListener("click", () => zedAction("start"));
document.getElementById("btn-zed-off").addEventListener("click", () => zedAction("stop"));

let quickRendered = false;

async function tick() {
  try {
    const r = await fetch("/api/snapshot", { cache: "no-store" });
    const s = await r.json();

    document.getElementById("meta").innerHTML =
      `ros=${s.ros_ok ? "ok" : "off"} · ${new Date(s.t * 1000).toLocaleTimeString()}` +
      `<br/><a href="${s.links?.mic_meter || "http://localhost:8091/"}" target="_blank" rel="noopener">麦克监视 :8091</a>`;

    const vc = s.voice_chain || {};
    setStep("mic", vc.mic);
    setStep("kws", vc.kws);
    setStep("mode", vc.mode);
    setStep("react", vc.react);

    setLayer("perception", s.layers?.perception);
    setLayer("decision", s.layers?.decision);
    setLayer("execution", s.layers?.execution);

    if (!quickRendered && s.quick_buttons) {
      renderQuickButtons(s.quick_buttons);
      quickRendered = true;
    }

    renderProcs(s.processes);
    renderImu(s.imu, s.imu_age);
    renderMic(s.mic, vc);

    kv(document.getElementById("flow-kv"), [
      ["/moon/mode", `${s.moon_mode || "—"} (${age(s.moon_mode_age)})`],
      ["/guide/state", `${s.guide_state || "—"} (${age(s.guide_state_age)})`],
      ["最近 voice_cmd", s.last_voice_cmd || "—"],
      ["最近 guide_cmd", s.last_guide_cmd || "—"],
      ["KWS 命中", vc.kws?.last_hit || "—"],
      ["麦克", vc.mic?.open ? "开麦" : "关麦"],
    ]);

    kv(document.getElementById("sense-kv"), [
      ["UWB dist", fmt(s.uwb?.dist_cm, 1) + " cm"],
      ["UWB ang", fmt(s.uwb?.ang_deg, 1) + " °"],
      ["UWB age", age(s.uwb_age)],
      ["障碍 L/C/R", [s.obstacle?.left_m, s.obstacle?.center_m, s.obstacle?.right_m].map((x) => fmt(x, 2)).join(" / ")],
    ]);

    const fsmMap = { 5: "EXEC", 8: "PROTECT" };
    kv(document.getElementById("exec-kv"), [
      ["fsm", s.fsm_state == null ? "—" : `${s.fsm_state} ${fsmMap[s.fsm_state] || ""}`],
      ["cmd hz", fmt(s.cmd_hz, 1)],
      ["vx / wz", `${fmt(s.cmd_vel?.vx, 3)} / ${fmt(s.cmd_vel?.wz, 3)}`],
    ]);

    renderZed(s.zed);
    document.getElementById("events").textContent = (s.events || []).join("\n") || "(no events)";
  } catch (e) {
    document.getElementById("meta").textContent = "API error: " + e;
  }
}

refreshTerminal();
tick();
setInterval(tick, 200);
setInterval(refreshTerminal, 800);

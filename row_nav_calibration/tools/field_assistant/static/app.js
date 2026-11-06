const $ = (selector) => document.querySelector(selector);

const state = {
  status: null,
  session: null,
  loaded: false,
};

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    headers: { "Content-Type": "application/json", ...(options.headers || {}) },
    ...options,
  });
  const payload = await response.json();
  if (!response.ok) throw new Error(payload.error || `请求失败 (${response.status})`);
  return payload;
}

function setMessage(message, kind = "") {
  const element = $("#message");
  element.textContent = message;
  element.className = `message ${kind ? `is-${kind}` : ""}`;
}

function renderStatus(status) {
  state.status = status;
  const isMock = status.mode === "mock";
  $("#mode-label").textContent = isMock ? "MyUbuntu 模拟模式" : "AWU202 硬件模式";
  $("#mode-dot").style.background = isMock ? "var(--water)" : "var(--rice)";
  $("#platform-label").textContent = `${status.platform.hostname} · ${status.platform.machine}`;
  $("#disk-label").textContent = `可用空间 ${status.disk.free_gb} GB`;
  $("#generated-at").textContent = new Date(status.generated_at).toLocaleString();

  $("#check-strip").innerHTML = status.checks.map((check) => `
    <article class="check-item" data-state="${escapeHtml(check.state)}">
      <div class="check-name">
        <span>${escapeHtml(check.name)}</span>
        <span class="check-state">${escapeHtml(check.label)}</span>
      </div>
      <p>${escapeHtml(check.message)}</p>
    </article>
  `).join("");

  const image = $("#preview-image");
  const empty = $("#preview-empty");
  if (status.camera.preview_available) {
    image.src = `/api/preview?t=${Date.now()}`;
    image.hidden = false;
    image.style.display = "block";
    empty.style.display = "none";
    $("#preview-badge").textContent = "文件预览（非实时）";
  } else {
    image.hidden = true;
    image.style.display = "none";
    empty.style.display = "block";
    $("#preview-badge").textContent = "无画面";
  }
  document.dispatchEvent(new CustomEvent("field-status", {detail: status}));
}

function renderSession(session) {
  const changedSession = state.session?.id !== session?.id;
  if (changedSession) resetCapture();
  state.session = session;
  document.dispatchEvent(new CustomEvent("field-session", { detail: session }));
  const panel = $("#current-session");
  $("#capture-panel").hidden = !session;
  if (!session) {
    panel.hidden = true;
    $("#intrinsic-runner").hidden = true;
    $("#session-tag").textContent = "尚未开始";
    return;
  }
  panel.hidden = false;
  $("#intrinsic-runner").hidden = false;
  $("#session-tag").textContent = "会话已建立";
  $("#session-name").textContent = session.id;
  $("#session-path").textContent = session.path;
  const download = $("#download-link");
  if (session.download_ready) {
    download.hidden = false;
    download.href = "/api/sessions/current/download";
  } else {
    download.hidden = true;
  }
  if (changedSession) restoreCapture();
  renderIntrinsicSummary(session.intrinsic_summary);
  renderIntrinsicAttempt(session.intrinsic_attempt);
  renderCapture();
}

const positions = ["左上", "上方", "右上", "左侧", "中央", "右侧", "左下", "下方", "右下"];
let captureSlot = 0;
let captureBusy = false;
let reviewedSlot = null;
let captureImageReady = false;
function captureKey() { return `row-nav-capture:${state.session?.id}`; }
function saveCaptureProgress() {
  try {
    const group = $("#capture-group").value;
    const previous = JSON.parse(localStorage.getItem(captureKey())) || {};
    localStorage.setItem(captureKey(), JSON.stringify({group, slot: captureSlot,
      slots: {...previous.slots, [group]: captureSlot}}));
  } catch (_) {}
}
function restoreCapture() {
  let saved;
  try { saved = JSON.parse(localStorage.getItem(captureKey())); } catch (_) {}
  const group = saved?.group === "validation" ? "validation" : "intrinsic";
  $("#capture-group").value = group;
  const limit = group === "intrinsic" ? 27 : 9;
  const slots = state.session?.capture_slots?.[group] || [];
  captureSlot = Number.isInteger(saved?.slot) && saved.slot >= 0 && saved.slot < limit
    ? saved.slot : Array.from({length: limit}, (_, i) => i).find(i => !slots.includes(i)) ?? limit - 1;
  showSavedCapture();
}
function renderIntrinsicAttempt(attempt) {
  const panel = $("#intrinsic-issues");
  panel.replaceChildren();
  if (!attempt || (!attempt.failed && !attempt.issues?.length)) return;
  const text = document.createElement("p");
  text.textContent = attempt.failed ? attempt.message : "照片检测结果（未采用的照片）：";
  panel.append(text);
  for (const issue of attempt.issues || []) {
    const row = document.createElement("p");
    row.textContent = `${issue.group === "intrinsic" ? "拟合" : "验证"} · ${issue.slot === null ? issue.image : `第 ${issue.slot + 1} 张`}：${issue.reason} `;
    if (issue.slot !== null && !state.session.metadata.intrinsic_run) {
      const button = document.createElement("button");
      button.type = "button"; button.textContent = "去重拍";
      button.addEventListener("click", () => {
        $("#capture-group").value = issue.group; captureSlot = issue.slot;
        reviewedSlot = null; $("#capture-review").hidden = true;
        document.dispatchEvent(new CustomEvent("field-navigate", {detail: "intrinsic"}));
        showSavedCapture(); saveCaptureProgress(); renderCapture();
      });
      row.append(button);
    }
    panel.append(row);
  }
}
function resetCapture() {
  captureSlot = 0;
  reviewedSlot = null;
  captureImageReady = false;
  $("#capture-review").hidden = true;
  $("#capture-feedback").textContent = "";
}
function captureImageUrl(group, slot) {
  return `/api/capture-image?session=${encodeURIComponent(state.session.id)}&group=${group}&slot=${slot}`;
}
function showSavedCapture() {
  const group = $("#capture-group").value;
  const saved = (state.session?.capture_slots?.[group] || []).includes(captureSlot);
  reviewedSlot = saved ? captureSlot : null;
  $("#capture-review").hidden = !saved;
  captureImageReady = false;
  $("#capture-show-corners").checked = false;
  if (saved) loadCapturePhoto();
  $("#capture-feedback").textContent = saved ? "这是已保存的照片。可检查后继续，或重拍当前这一张。" : "";
}
function renderCapture() {
  const group = $("#capture-group").value;
  saveCaptureProgress();
  const round = Math.floor(captureSlot / 9);
  $("#capture-round").value = String(round);
  const limit = group === "intrinsic" ? 27 : 9;
  const slots = state.session?.capture_slots?.[group] || [];
  const counts = state.session?.capture_counts || {};
  const completed = !!state.session?.metadata?.intrinsic_run;
  const locked = captureBusy || completed;
  $("#capture-round").disabled = locked || group !== "intrinsic";
  $("#capture-position-count").textContent = `第 ${captureSlot + 1} / ${limit} 张`;
  $("#capture-saved-count").textContent = `已保存 ${counts[group] || 0} 张`;
  $("#capture-progress").max = limit;
  $("#capture-progress").value = counts[group] || 0;
  $("#capture-counts").textContent = `已保存：拟合 ${counts.intrinsic || 0}/27 张；独立验证 ${counts.validation || 0}/9 张`;
  const pose = group === "intrinsic" ? ["近距离，板子较大，轻微左右倾斜", "中距离，改变上下倾斜", "远距离，确保方格仍清晰可辨"][round] : "独立验证，重新摆放并改变距离和角度";
  $("#capture-instruction").textContent = `第 ${captureSlot + 1}/${limit} 张：${pose}；放在画面${positions[captureSlot % 9]}，完整入镜。`;
  $("#capture-grid").innerHTML = positions.map((label, i) => {
    const saved = slots.includes(round * 9 + i);
    const current = i === captureSlot % 9;
    return `<button type="button" data-position="${i}" data-saved="${saved}" aria-current="${current}" ${captureBusy ? "disabled" : ""}>${label}${saved ? " ✓" : ""}<small>${current ? "当前选择" : saved ? "已拍摄" : "待拍摄"}</small></button>`;
  }).join("");
  $("#capture-round-buttons").hidden = group !== "intrinsic";
  $("#capture-round-buttons").innerHTML = ["近距离", "中距离", "远距离"].map((label, i) =>
    `<button type="button" data-round="${i}" aria-pressed="${round === i}" ${captureBusy ? "disabled" : ""}>${label}<small>${slots.filter(slot => Math.floor(slot / 9) === i).length} / 9 张</small></button>`
  ).join("");
  $("#capture-button").disabled = locked;
  $("#capture-group").disabled = captureBusy;
  $("#capture-button").textContent = completed ? "本次内参已计算" : captureBusy ? "正在拍照并检查照片…" : reviewedSlot === captureSlot ? "不满意，重拍这张" : "准备好了，拍照";
  $("#capture-next").disabled = locked || reviewedSlot !== captureSlot || !captureImageReady || captureSlot === limit - 1;
  $("#capture-next-help").textContent = completed ? "已有候选内参，重新标定请先建立新会话。" : captureSlot === limit - 1 ? "本组最后一张；拍好后切换采集组或计算内参。" : reviewedSlot !== captureSlot ? "拍好当前照片后，可前往下一个位置。" : "检查清晰度和完整性，满意后再去下一个位置。";
  $("#capture-save-state").textContent = captureBusy ? "正在采集…" : (counts.intrinsic || counts.validation) ? "✓ 已拍照片保存在设备上" : "等待第一张照片";
  $(".preview-frame").dataset.captured = String(reviewedSlot === captureSlot);
  $("#preview-badge").textContent = reviewedSlot === captureSlot ? "已拍照片（非实时）" : state.status?.camera.preview_available ? "参考图片（非实时）" : "等待拍照";
  $("#capture-target-label").textContent = `当前选择：${positions[captureSlot % 9]}`;
  $("#photo-source").textContent = reviewedSlot === captureSlot ? `已拍照片 · 第 ${captureSlot + 1} 张${state.status?.mode === "mock" ? "（模拟）" : ""}` : state.status?.camera.preview_available ? "参考图片（非实时），点击拍照获取当前画面" : "尚无预览，点击拍照获取当前画面";
  $("#capture-thumbnails").innerHTML = slots.filter(slot => Math.floor(slot / 9) === round).map(slot =>
    `<button type="button" data-capture-slot="${slot}" aria-current="${slot === captureSlot}" aria-label="查看第 ${slot + 1} 张已拍照片"><img src="${captureImageUrl(group, slot)}" alt="第 ${slot + 1} 张已拍照片">第 ${slot + 1} 张 ✓</button>`
  ).join("");
  renderCaptureQuality();
  document.dispatchEvent(new CustomEvent("field-capture", {detail: {group, slot: captureSlot, busy: captureBusy}}));
}
function loadCapturePhoto() {
  if (reviewedSlot !== captureSlot || !state.session) return;
  const group = $("#capture-group").value;
  const key = `${state.session.id}:${group}:${captureSlot}`;
  const image = $("#capture-review");
  captureImageReady = false;
  $("#capture-next").disabled = true;
  $("#capture-image-retry").hidden = true;
  image.hidden = true;
  image.onload = () => {
    if (`${state.session?.id}:${$("#capture-group").value}:${captureSlot}` !== key) return;
    captureImageReady = image.naturalWidth > 0;
    image.hidden = !captureImageReady;
    renderCapture();
  };
  image.onerror = () => {
    if (`${state.session?.id}:${$("#capture-group").value}:${captureSlot}` !== key) return;
    captureImageReady = false;
    $("#capture-next").disabled = true;
    $("#capture-image-retry").hidden = false;
    $("#capture-feedback").textContent = "照片已保存，但画面加载失败。请重试显示照片，确认后再继续。";
  };
  image.src = `${captureImageUrl(group, captureSlot)}&kind=${$("#capture-show-corners").checked ? "corners" : "raw"}&t=${Date.now()}`;
}
function renderCaptureQuality() {
  const visible = reviewedSlot === captureSlot;
  $("#capture-quality").hidden = !visible;
  if (!visible) { $("#capture-image-retry").hidden = true; return; }
  const quality = state.session?.capture_quality?.[$("#capture-group").value]?.[String(captureSlot)];
  $("#capture-show-corners").disabled = captureBusy || !quality?.overlay_available;
  $("#capture-quality-summary").textContent = quality && Number.isFinite(quality.corners)
    ? `检测到 ${quality.corners}/${quality.expected_corners || 54} 个角点${quality.corner_coverage !== undefined ? ` · 角点覆盖 ${(quality.corner_coverage*100).toFixed(1)}%` : ""} · ${quality.width || "—"} × ${quality.height || "—"}${quality.sharpness !== undefined ? ` · 清晰度指标 ${quality.sharpness.toFixed(1)}` : ""}`
    : "此照片暂无可用的即时检查结果，请人工检查或重拍。";
  $("#capture-quality-warnings").replaceChildren(...(quality?.warnings || []).map(message => {
    const item = document.createElement("li"); item.textContent = message; return item;
  }));
}
$("#capture-show-corners").addEventListener("change", loadCapturePhoto);
$("#capture-image-retry").addEventListener("click", loadCapturePhoto);
$("#capture-grid").addEventListener("click", (event) => {
  const button = event.target.closest("[data-position]");
  if (!button || captureBusy) return;
  captureSlot = Math.floor(captureSlot / 9) * 9 + Number(button.dataset.position);
  showSavedCapture();
  renderCapture();
});
$("#capture-round").addEventListener("change", () => {
  captureSlot = Number($("#capture-round").value) * 9;
  showSavedCapture(); renderCapture();
});
$("#capture-group").addEventListener("change", () => {
  resetCapture();
  try {
    const saved = JSON.parse(localStorage.getItem(captureKey()));
    const slot = saved?.slots?.[$("#capture-group").value];
    if (Number.isInteger(slot)) captureSlot = slot;
  } catch (_) {}
  showSavedCapture(); renderCapture();
});
$("#capture-round-buttons").addEventListener("click", event => {
  const button = event.target.closest("[data-round]");
  if (!button || captureBusy) return;
  captureSlot = Number(button.dataset.round) * 9;
  showSavedCapture(); renderCapture();
});
$("#capture-thumbnails").addEventListener("click", event => {
  const button = event.target.closest("[data-capture-slot]");
  if (!button || captureBusy) return;
  captureSlot = Number(button.dataset.captureSlot);
  showSavedCapture(); renderCapture();
});
$("#capture-next").addEventListener("click", () => {
  captureSlot += 1;
  showSavedCapture();
  if (reviewedSlot === null) $("#capture-feedback").textContent = "请移动标定板，准备好后再点击拍照。";
  renderCapture();
});
$("#capture-button").addEventListener("click", async () => {
  captureBusy = true;
  reviewedSlot = null;
  renderCapture();
  try {
    const result = await api("/api/sessions/current/capture", {
      method: "POST", body: JSON.stringify({ group: $("#capture-group").value, slot: captureSlot }),
    });
    renderSession(result.session);
    reviewedSlot = captureSlot;
    $("#capture-show-corners").checked = false;
    $("#capture-review").hidden = false;
    loadCapturePhoto();
    $("#capture-feedback").textContent = result.simulated ? "模拟回放图片，仅测试流程，不能用于真实标定。" : "请检查照片：完整、清晰、无明显反光。满意后点下一个位置；最后一张拍好后可切换采集组或计算。";
  } catch (error) {
    $("#capture-feedback").textContent = error.message;
  } finally {
    captureBusy = false;
    renderCapture();
  }
});
$("#calibrate-captured").addEventListener("click", async () => {
  const button = $("#calibrate-captured");
  button.disabled = true;
  setMessage("正在使用本次拍摄的两组照片计算内参…");
  try {
    const result = await api("/api/sessions/current/intrinsics/run", {
      method: "POST", body: JSON.stringify({ source: "captured" }),
    });
    renderSession(result.session);
    setMessage("候选内参已生成，等待人工复核。", "success");
  } catch (error) { setMessage(error.message, "error"); await refresh(); }
  finally { button.disabled = false; }
});

function renderIntrinsicSummary(summary) {
  const result = $("#intrinsic-result");
  if (!summary) {
    $("#intrinsic-state").textContent = "未运行";
    result.hidden = true;
    return;
  }
  $("#intrinsic-state").textContent = "候选结果";
  result.hidden = false;
  const train = summary.training || {};
  const validation = summary.validation || {};
  const number = (value) => Number.isFinite(Number(value)) ? Number(value).toFixed(3) : "—";
  result.innerHTML = `
    <dt>拟合可用图片</dt><dd>${escapeHtml(train.usable ?? "—")} / ${escapeHtml(train.matched ?? "—")}</dd>
    <dt>拟合 RMS</dt><dd>${number(train.calibration_rms_px)} px</dd>
    <dt>验证可用图片</dt><dd>${escapeHtml(validation.usable ?? "—")} / ${escapeHtml(validation.matched ?? "—")}</dd>
    <dt>验证平均 RMS</dt><dd>${number(validation.mean_reprojection_rms_px)} px</dd>
    <dt>最大验证 RMS</dt><dd>${number(validation.max_reprojection_rms_px)} px</dd>
    <dt>启用状态</dt><dd>等待人工复核</dd>
  `;
}

async function refresh() {
  const button = $("#refresh-button");
  button.disabled = true;
  try {
    const [status, session] = await Promise.all([
      api("/api/status"),
      api("/api/sessions/current"),
    ]);
    state.loaded = true;
    renderStatus(status);
    renderSession(session.session);
  } catch (error) {
    setMessage(error.message, "error");
  } finally {
    button.disabled = false;
  }
}

$("#session-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  if (typeof bevDirty !== "undefined" && bevDirty && !window.confirm("当前点位尚未提交到设备，确定开始新会话吗？建议先保存点位草稿。")) return;
  const form = event.currentTarget;
  const submit = form.querySelector('button[type="submit"]');
  const data = new FormData(form);
  const payload = {
    operator: data.get("operator"),
    square_width_mm: data.get("square_width_mm"),
    square_height_mm: data.get("square_height_mm"),
    marker_mm: data.get("marker_mm"),
    check_line_mm: data.get("check_line_mm"),
    board_flat: data.get("board_flat") === "on",
  };
  submit.disabled = true;
  setMessage("正在建立会话…");
  try {
    const result = await api("/api/sessions/start", {
      method: "POST",
      body: JSON.stringify(payload),
    });
    renderSession(result.session);
    setMessage("现场会话已建立，输入和当前状态已经保存。", "success");
  } catch (error) {
    setMessage(error.message, "error");
  } finally {
    submit.disabled = false;
  }
});

$("#export-button").addEventListener("click", async () => {
  const button = $("#export-button");
  button.disabled = true;
  setMessage("正在生成校验和与 ZIP…");
  try {
    const result = await api("/api/sessions/current/export", {
      method: "POST",
      body: "{}",
    });
    renderSession(result.session);
    $("#download-link").href = result.session.download_url;
    $("#download-link").hidden = false;
    setMessage("测试包已生成，可以下载。", "success");
  } catch (error) {
    setMessage(error.message, "error");
  } finally {
    button.disabled = false;
  }
});

$("#run-intrinsic-button").addEventListener("click", async () => {
  const button = $("#run-intrinsic-button");
  button.disabled = true;
  setMessage("正在导入图片并运行内参拟合与独立验证…");
  try {
    const result = await api("/api/sessions/current/intrinsics/run", {
      method: "POST",
      body: JSON.stringify({
        train_dir: $("#train-dir").value,
        validation_dir: $("#validation-dir").value,
        camera_name: "CMD108-P200A-H120",
      }),
    });
    renderSession(result.session);
    setMessage("候选内参已经生成。检查对比图后才能标记为有效。", "success");
  } catch (error) {
    setMessage(error.message, "error");
    await refresh();
  } finally {
    button.disabled = false;
  }
});

$("#refresh-button").addEventListener("click", refresh);
refresh();

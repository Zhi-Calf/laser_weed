const $ = (selector) => document.querySelector(selector);

const state = {
  status: null,
  session: null,
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
    image.style.display = "block";
    empty.style.display = "none";
    $("#preview-badge").textContent = "文件预览（非实时）";
  } else {
    image.style.display = "none";
    empty.style.display = "block";
    $("#preview-badge").textContent = "无画面";
  }
}

function renderSession(session) {
  if (state.session?.id !== session?.id) resetCapture();
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
  renderIntrinsicSummary(session.intrinsic_summary);
  renderCapture();
}

const positions = ["左上", "上方", "右上", "左侧", "中央", "右侧", "左下", "下方", "右下"];
let captureSlot = 0;
let captureBusy = false;
let reviewedSlot = null;
function resetCapture() {
  captureSlot = 0;
  reviewedSlot = null;
  $("#capture-review").hidden = true;
  $("#capture-feedback").textContent = "";
}
function renderCapture() {
  const group = $("#capture-group").value;
  const round = Math.floor(captureSlot / 9);
  const limit = group === "intrinsic" ? 27 : 9;
  const counts = state.session?.capture_counts || {};
  $("#capture-counts").textContent = `已保存：拟合 ${counts.intrinsic || 0}/27 张；独立验证 ${counts.validation || 0}/9 张`;
  const pose = group === "intrinsic" ? ["近距离，板子较大，轻微左右倾斜", "中距离，改变上下倾斜", "远距离，确保方格仍清晰可辨"][round] : "独立验证，重新摆放并改变距离和角度";
  $("#capture-instruction").textContent = `第 ${captureSlot + 1}/${limit} 张：${pose}；放在画面${positions[captureSlot % 9]}，完整入镜。`;
  $("#capture-grid").innerHTML = positions.map((label, i) => `<button type="button" data-position="${i}" aria-current="${i === captureSlot % 9}" ${captureBusy ? "disabled" : ""}>${label}</button>`).join("");
  $("#capture-button").disabled = captureBusy;
  $("#capture-group").disabled = captureBusy;
  $("#capture-button").textContent = captureBusy ? "正在拍照，请保持不动…" : reviewedSlot === captureSlot ? "不满意，重拍这张" : "准备好了，拍照";
  $("#capture-next").disabled = captureBusy || reviewedSlot !== captureSlot || captureSlot === limit - 1;
}
$("#capture-grid").addEventListener("click", (event) => {
  const button = event.target.closest("[data-position]");
  if (!button || captureBusy) return;
  captureSlot = Math.floor(captureSlot / 9) * 9 + Number(button.dataset.position);
  reviewedSlot = null;
  $("#capture-review").hidden = true;
  renderCapture();
});
$("#capture-group").addEventListener("change", () => { resetCapture(); renderCapture(); });
$("#capture-next").addEventListener("click", () => {
  captureSlot += 1;
  reviewedSlot = null;
  $("#capture-review").hidden = true;
  $("#capture-feedback").textContent = "请移动标定板，准备好后再点击拍照。";
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
    const image = $("#capture-review");
    image.src = `${result.image_url}&t=${Date.now()}`;
    image.hidden = false;
    reviewedSlot = captureSlot;
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
  } catch (error) { setMessage(error.message, "error"); }
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
  } finally {
    button.disabled = false;
  }
});

$("#refresh-button").addEventListener("click", refresh);
refresh();

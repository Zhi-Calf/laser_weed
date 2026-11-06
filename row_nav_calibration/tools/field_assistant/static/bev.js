/* BEV operates on a frozen undistorted photo, never on a resized pixel grid. */
let bevData = null;
let bevPoints = [];
let bevSelected = 0;
let bevKey = "";
let bevBusy = false;
let bevDirty = false;
let bevPhoto = new Image();
let bevImageReady = false;
let bevImageRequest = 0;
let bevZoom = 1;
let bevPan = false;
let bevDrag = null;
let bevSuppressClick = false;
let bevUndo = [];
const bevDraftBase = () => JSON.stringify({points: bevData?.points || null, draft: bevData?.draft || null});
const bevDraftKey = () => `row-nav-bev-draft:${bevKey}`;
function storeBevDraft() {
  try { localStorage.setItem(bevDraftKey(), JSON.stringify({base: bevDraftBase(), points: bevPoints})); }
  catch (_) { bevMessage("浏览器无法保存草稿，请点击保存点位草稿，离开前确认已保存。"); }
}
function clearBevDraft() { try { localStorage.removeItem(bevDraftKey()); } catch (_) {} }
window.addEventListener("beforeunload", event => {
  if (bevDirty) { event.preventDefault(); event.returnValue = ""; }
});
const blankPoint = (role = "fit") => ({role, x: "", y: "", u: null, v: null});
const bevMessage = (text) => { $("#bev-message").textContent = text; };
const bevUrl = (name) => `/api/bev-image?session=${encodeURIComponent(state.session.id)}&name=${name}&t=${Date.now()}`;

function renderBev(session) {
  $("#bev-panel").hidden = !session;
  if (!session) { bevKey = ""; bevData = null; bevImageRequest++; bevImageReady = false; return; }
  bevData = session.bev;
  $("#bev-editor").hidden = !bevData;
  const key = `${session.id}:${bevData?.snapshot?.image_id || ""}`;
  if (key !== bevKey) {
    bevKey = key;
    bevImageRequest++;
    bevSelected = 0;
    bevUndo = [];
    bevImageReady = false;
    bevDirty = false;
    $("#bev-publish-reviewed").checked = false;
    $("#bev-reference-confirmed").checked = !!bevData?.reference?.reference_confirmed;
    $("#bev-intrinsic-confirmed").checked = !!bevData?.reference?.intrinsic_confirmed;
    $("#bev-intrinsic").value = bevData?.reference?.intrinsic_source || "configured";
    $("#bev-dynamic").checked = !!bevData?.reference?.dynamic_enabled;
    $("#bev-origin-height").value = bevData?.reference?.origin_height_m || "";
    $("#bev-dynamic-reference").checked = !!bevData?.reference?.dynamic_enabled;
    bevPoints = bevData?.draft?.points || bevData?.points?.points || Array.from({length: 12}, (_, i) => blankPoint(i < 8 ? "fit" : "check"));
    if (bevData) {
      let local;
      try { local = JSON.parse(localStorage.getItem(bevDraftKey())); } catch (_) {}
      if (local?.base === bevDraftBase() && Array.isArray(local.points)) {
        bevPoints = local.points; bevDirty = true;
        bevMessage("已恢复本浏览器未提交的点位草稿，请保存或重新计算。");
      }
      loadBevPhoto();
      bevRows();
    }
  }
  bevResult();
  const runtime = session.runtime_calibration;
  if (runtime) $("#bev-runtime-path").textContent = `算法参数固定目录：${runtime.current_path}；${runtime.active ? `当前已复核版本 ${runtime.active.release_id}` : "尚无已复核版本"}。目录中的 sdk-calibration.json 和 observer.calibration.yaml 分别供 SDK 与 ROS 接入时读取。`;
  bevPublishState();
}
function bevPublishState() {
  $("#bev-publish").disabled = bevBusy || bevDirty || (!!bevData?.draft || !bevData?.summary?.checks_passed) || state.session?.metadata?.mode !== "hardware" || !$("#bev-publish-reviewed").checked;
}
function bevResult() {
  const summary = bevDirty || bevData?.draft ? null : bevData?.summary;
  $("#bev-state").textContent = summary ? (summary.checks_passed ? "几何检查通过 · 待复核" : "检查未通过") : bevData ? "填写与点选中" : "未开始";
  $("#bev-result").hidden = !summary;
  if (!summary) return;
  $("#bev-metrics").textContent = `${bevData.snapshot.mode === "mock" ? "模拟流程；" : ""}标定内点 ${summary.fit_inliers}/${summary.fit.count}；独立检查 ${summary.check.count} 点，RMSE ${(summary.check.rmse_m * 100).toFixed(2)} cm，最大误差 ${(summary.check.max_m * 100).toFixed(2)} cm。阈值：RMSE ≤ 5 cm，最大 ≤ 10 cm。候选参数尚未启用。`;
  const d = summary.dynamic;
  $("#bev-dynamic-result").textContent = d ? `动态外参已计算（姿态时间差 ${d.sync_age_ms.toFixed(1)} ms）：相机离地 ${d.camera_height_ground_m.toFixed(3)} m；相对后轴中心位置 [${d.camera_position_vehicle_m.map(v=>v.toFixed(3)).join(", ")}] m；光轴下俯 ${d.camera_down_angle_deg.toFixed(2)}°、向左偏 ${d.camera_left_angle_deg.toFixed(2)}°；重投影 RMSE ${d.reprojection_rmse_px.toFixed(2)} px；动态独立检查 ${(d.check_rmse_m*100).toFixed(2)} cm。输出原点：实体后轴中心。候选结果仍需复核。` : "本次为静态 BEV，不会启用姿态补偿。";
  $("#bev-errors").replaceChildren(...summary.point_errors_m.map((error, i) => {
    const item = document.createElement("li");
    item.textContent = `${bevPoints[i]?.role === "check" ? "独立检查" : "标定"}点 ${(error * 100).toFixed(2)} cm`;
    return item;
  }));
  $("#bev-overlay").src = bevUrl("overlay.jpg");
  $("#bev-output").src = bevUrl("bev.jpg");
}
function bevRows() {
  $("#bev-points").innerHTML = bevPoints.map((p, i) => `<tr class="${i === bevSelected ? "is-selected" : ""}">
    <td><button type="button" data-select="${i}" aria-pressed="${i === bevSelected}" aria-label="选择第 ${i+1} 点">${i + 1}</button></td>
    <td><select data-index="${i}" data-key="role" aria-label="第 ${i+1} 点用途"><option value="fit" ${p.role === "fit" ? "selected" : ""}>标定</option><option value="check" ${p.role === "check" ? "selected" : ""}>检查</option></select></td>
    ${["x", "y"].map(k => `<td><input type="number" step="0.001" data-index="${i}" data-key="${k}" value="${escapeHtml(p[k])}" aria-label="第 ${i+1} 点${k === "x" ? "前向" : "左向"}距离"></td>`).join("")}
    <td>${p.u === null ? "未点选" : `${Number(p.u).toFixed(1)}, ${Number(p.v).toFixed(1)}`}</td>
    <td><button type="button" data-remove="${i}" aria-label="删除第 ${i+1} 点">删除</button></td></tr>`).join("");
  const p = bevPoints[bevSelected];
  $("#bev-selected").textContent = `当前选择第 ${bevSelected + 1} 点，请在照片中点击对应的地面标记中心。`;
  $("#bev-u").value = p?.u ?? "";
  $("#bev-v").value = p?.v ?? "";
  bevDraw();
  updateBevImageTools();
}
function bevDraw() {
  if (!bevImageReady || !bevPhoto.complete || !bevPhoto.naturalWidth) return;
  const canvas = $("#bev-canvas");
  canvas.width = bevPhoto.naturalWidth;
  canvas.height = bevPhoto.naturalHeight;
  const ctx = canvas.getContext("2d");
  ctx.drawImage(bevPhoto, 0, 0);
  const scale = canvas.width / 800;
  bevPoints.forEach((p, i) => {
    if (p.u === null || p.v === null) return;
    ctx.strokeStyle = p.role === "fit" ? "#36dd63" : "#ffaf38";
    ctx.lineWidth = (i === bevSelected ? 4 : 2) * scale;
    ctx.beginPath(); ctx.arc(p.u, p.v, 7 * scale, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = ctx.strokeStyle; ctx.font = `bold ${18 * scale}px sans-serif`;
    ctx.fillText(String(i + 1), p.u + 9 * scale, p.v - 9 * scale);
  });
  drawBevLoupe();
}

function updateBevImageTools() {
  const disabled = !bevImageReady || bevBusy;
  $("#bev-image-controls").querySelectorAll("button,input").forEach(el => { el.disabled = disabled; });
  $("#bev-set-pixel").disabled = disabled;
  $("#bev-run").disabled = disabled;
  $("#bev-undo").disabled = disabled || !bevUndo.length;
  $("#bev-zoom-out").disabled = disabled || bevZoom <= 1;
  $("#bev-zoom-in").disabled = disabled || bevZoom >= 8;
  $("#bev-canvas").setAttribute("aria-disabled", String(disabled));
  $("#bev-image-scroll").dataset.ready = String(bevImageReady);
}
function loadBevPhoto() {
  if (!bevData) return;
  const request = ++bevImageRequest;
  bevImageReady = false;
  $("#bev-image-retry").hidden = true;
  $("#bev-image-status").textContent = "正在加载地面照片，加载完成后才能选点。";
  $("#bev-canvas").style.visibility = "hidden";
  $("#bev-loupe").hidden = true;
  updateBevImageTools();
  const image = new Image();
  const failed = message => {
    if (request !== bevImageRequest) return;
    bevImageReady = false;
    $("#bev-image-status").textContent = message;
    $("#bev-image-retry").hidden = false;
    updateBevImageTools();
  };
  image.onload = () => {
    if (request !== bevImageRequest) return;
    if (image.naturalWidth !== bevData.snapshot.width || image.naturalHeight !== bevData.snapshot.height) {
      failed("照片尺寸与本次冻结记录不一致，不能选点。请重试或重新拍摄地面照片。"); return;
    }
    bevPhoto = image;
    bevImageReady = true;
    bevZoom = 1;
    bevPan = false;
    $("#bev-pan").setAttribute("aria-pressed", "false");
    $("#bev-canvas").style.width = "100%";
    $("#bev-canvas").style.visibility = "visible";
    $("#bev-image-scroll").dataset.pan = "false";
    $("#bev-image-scroll").scrollLeft = $("#bev-image-scroll").scrollTop = 0;
    $("#bev-zoom-label").textContent = "1×";
    $("#bev-image-status").textContent = `${image.naturalWidth} × ${image.naturalHeight} · 点位按原图像素保存`;
    updateBevImageTools(); bevDraw();
  };
  image.onerror = () => failed("照片加载失败，已有点位已保留；点击重试加载照片。");
  image.src = bevUrl("snapshot.jpg");
}
function rememberBevEdit() {
  bevUndo.push({points: JSON.parse(JSON.stringify(bevPoints)), selected: bevSelected});
  if (bevUndo.length > 50) bevUndo.shift();
}
function undoBevEdit() {
  if (bevBusy || !bevImageReady || !bevUndo.length) return;
  const previous = bevUndo.pop();
  bevPoints = previous.points; bevSelected = previous.selected;
  bevEdited(); bevRows();
}
function zoomBev(factor) {
  if (!bevImageReady || bevBusy) return;
  const pane = $("#bev-image-scroll"), canvas = $("#bev-canvas");
  const x = (pane.scrollLeft + pane.clientWidth / 2) / canvas.clientWidth;
  const y = (pane.scrollTop + pane.clientHeight / 2) / canvas.clientHeight;
  bevZoom = Math.max(1, Math.min(8, factor));
  canvas.style.width = `${bevZoom * 100}%`;
  pane.scrollLeft = x * canvas.clientWidth - pane.clientWidth / 2;
  pane.scrollTop = y * canvas.clientHeight - pane.clientHeight / 2;
  $("#bev-zoom-label").textContent = `${Number(bevZoom.toFixed(2))}×`;
  updateBevImageTools();
}
function drawBevLoupe(u, v) {
  const visible = bevImageReady && $("#bev-show-loupe").checked;
  $("#bev-loupe").hidden = !visible;
  if (!visible) return;
  const p = bevPoints[bevSelected];
  u = Number(u ?? p?.u ?? bevPhoto.naturalWidth / 2);
  v = Number(v ?? p?.v ?? bevPhoto.naturalHeight / 2);
  const canvas = $("#bev-loupe-canvas"), ctx = canvas.getContext("2d");
  ctx.fillStyle = "#e1e8e5"; ctx.fillRect(0, 0, 160, 160);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(bevPhoto, u - 20, v - 20, 40, 40, 0, 0, 160, 160);
  ctx.strokeStyle = "#b74912"; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(65, 80); ctx.lineTo(95, 80); ctx.moveTo(80, 65); ctx.lineTo(80, 95); ctx.stroke();
  $("#bev-loupe-label").textContent = `${u.toFixed(1)}, ${v.toFixed(1)} · 原图 40 × 40 px`;
}
$("#bev-image-retry").addEventListener("click", loadBevPhoto);
$("#bev-zoom-in").addEventListener("click", () => zoomBev(bevZoom * 1.5));
$("#bev-zoom-out").addEventListener("click", () => zoomBev(bevZoom / 1.5));
$("#bev-fit").addEventListener("click", () => {
  zoomBev(1); $("#bev-image-scroll").scrollLeft = $("#bev-image-scroll").scrollTop = 0;
});
$("#bev-pan").addEventListener("click", () => {
  bevPan = !bevPan;
  $("#bev-pan").setAttribute("aria-pressed", String(bevPan));
  $("#bev-image-scroll").dataset.pan = String(bevPan);
});
$("#bev-undo").addEventListener("click", undoBevEdit);
$("#bev-show-loupe").addEventListener("change", () => drawBevLoupe());
const bevCanvas = $("#bev-canvas");
bevCanvas.addEventListener("keydown", event => {
  if (!bevImageReady || bevBusy) return;
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "z") {
    event.preventDefault(); undoBevEdit(); return;
  }
  const deltas = {ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1]};
  const delta = deltas[event.key], p = bevPoints[bevSelected];
  if (!delta) return;
  event.preventDefault();
  if (p?.u === null || p?.v === null) { bevMessage("请先在照片上选择当前点。"); return; }
  const step = event.shiftKey ? .1 : 1;
  bevSetPixel(Number(p.u) + delta[0] * step, Number(p.v) + delta[1] * step);
});
bevCanvas.addEventListener("pointerdown", event => {
  if (!bevImageReady || bevBusy || (!bevPan && event.button !== 1)) return;
  const pane = $("#bev-image-scroll");
  bevDrag = {x: event.clientX, y: event.clientY, left: pane.scrollLeft, top: pane.scrollTop};
  bevCanvas.setPointerCapture(event.pointerId); event.preventDefault();
});
bevCanvas.addEventListener("pointermove", event => {
  if (!bevImageReady || bevBusy) return;
  if (bevDrag) {
    $("#bev-image-scroll").scrollLeft = bevDrag.left - (event.clientX - bevDrag.x);
    $("#bev-image-scroll").scrollTop = bevDrag.top - (event.clientY - bevDrag.y);
    bevSuppressClick = true; return;
  }
  const box = bevCanvas.getBoundingClientRect();
  drawBevLoupe((event.clientX - box.left) * bevPhoto.naturalWidth / box.width,
    (event.clientY - box.top) * bevPhoto.naturalHeight / box.height);
});
for (const event of ["pointerup", "pointercancel"]) bevCanvas.addEventListener(event, () => {
  bevDrag = null; setTimeout(() => { bevSuppressClick = false; }, 0);
});

function bevEdited() { bevDirty = true; storeBevDraft(); $("#bev-publish-reviewed").checked = false; bevPublishState(); bevResult(); bevMessage("点位已修改，请保存点位草稿或重新计算。"); document.dispatchEvent(new CustomEvent("field-bev-edit")); }
$("#bev-points").addEventListener("input", event => {
  const el = event.target;
  if (!el.dataset.key) return;
  if (bevBusy) return;
  rememberBevEdit();
  bevPoints[Number(el.dataset.index)][el.dataset.key] = el.value;
  bevEdited(); bevDraw();
});
$("#bev-points").addEventListener("click", event => {
  if (bevBusy) return;
  const el = event.target;
  if (el.dataset.select !== undefined) bevSelected = Number(el.dataset.select);
  else if (el.dataset.remove !== undefined && bevPoints.length > 1) {
    rememberBevEdit();
    bevPoints.splice(Number(el.dataset.remove), 1);
    bevSelected = Math.min(bevSelected, bevPoints.length - 1); bevEdited();
  } else return;
  bevRows();
});
function bevSetPixel(u, v) {
  if (!bevImageReady) { bevMessage("请等待地面照片加载完成后再选点。"); return; }
  if (bevBusy || !bevData || !Number.isFinite(u) || !Number.isFinite(v) || u < 0 || v < 0 || u >= bevData.snapshot.width || v >= bevData.snapshot.height) {
    bevMessage("点选位置必须在原始照片范围内。"); return;
  }
  rememberBevEdit();
  Object.assign(bevPoints[bevSelected], {u, v}); bevEdited(); bevRows();
}
$("#bev-canvas").addEventListener("click", event => {
  if (!bevImageReady || bevPan || bevSuppressClick || bevDrag) return;
  const box = event.currentTarget.getBoundingClientRect();
  bevSetPixel((event.clientX - box.left) * bevPhoto.naturalWidth / box.width,
    (event.clientY - box.top) * bevPhoto.naturalHeight / box.height);
});
$("#bev-set-pixel").addEventListener("click", () => {
  if ($("#bev-u").value === "" || $("#bev-v").value === "") { bevMessage("请填写水平和垂直像素坐标。"); return; }
  bevSetPixel(Number($("#bev-u").value), Number($("#bev-v").value));
});
$("#bev-add").addEventListener("click", () => {
  if (bevPoints.length >= 64) return;
  if (bevBusy) return;
  rememberBevEdit();
  bevPoints.push(blankPoint()); bevSelected = bevPoints.length - 1; bevEdited(); bevRows();
});
async function bevAction(action, payload) {
  if (bevBusy) return;
  if (action === "run" && !bevImageReady) { bevMessage("请先加载地面照片，再计算 BEV。"); return; }
  bevBusy = true;
  $("#bev-panel").querySelectorAll("button,input,select").forEach(el => { el.disabled = true; });
  bevMessage(action === "prepare" ? "正在拍照和去畸变，请保持相机与地面标记不动…" : "正在保存并处理地面点…");
  try {
    const result = await api(`/api/sessions/current/bev/${action}`, {method: "POST", body: JSON.stringify(payload)});
    // A newly prepared frame clears the local editor even when replay bytes match.
    if (action === "prepare") bevKey = "";
    clearBevDraft();
    bevDirty = false;
    renderSession(result.session);
    if (action === "run") $("#bev-publish-reviewed").checked = false;
    document.dispatchEvent(new CustomEvent("field-bev-operation", {detail: action}));
    bevMessage(action === "publish" ? "已保存为算法参数并保留旧版本。正在运行的算法仍使用原参数，下次初始化时加载新版本。" : action === "prepare" ? "照片已冻结并去畸变。请填写距离并点选地面标记。" : action === "run" ? "BEV 已计算，请检查误差和鸟瞰图，再导出记录。" : "点位草稿已保存到设备，刷新页面后可继续；计算时会检查所有点是否完整。");
  } catch (error) {
    // A failed request may already have invalidated the server's old result.
    // Keep local edits, hide stale results, and require a new review.
    $("#bev-publish-reviewed").checked = false;
    if (action !== "publish") bevDirty = true;
    bevResult();
    bevPublishState();
    let message = error.message;
    try {
      const current = await api("/api/sessions/current");
      renderSession(current.session);
    } catch (_) {
      bevDirty = true;
      bevResult();
      message += "；无法刷新最新状态，请恢复连接后刷新页面。";
    }
    bevMessage(message);
  }
  finally {
    bevBusy = false;
    $("#bev-panel").querySelectorAll("button,input,select").forEach(el => { el.disabled = false; });
    bevPublishState();
    updateBevImageTools();
  }
}
$("#bev-prepare").addEventListener("click", () => {
  if (bevData && !window.confirm("重新拍摄将清空本会话已有地面点和 BEV 结果。确定继续吗？")) return;
  bevAction("prepare", {reference_confirmed: $("#bev-reference-confirmed").checked,
    intrinsic_confirmed: $("#bev-intrinsic-confirmed").checked, intrinsic_source: $("#bev-intrinsic").value,
    dynamic_enabled: $("#bev-dynamic").checked, origin_height_m: $("#bev-origin-height").value,
    dynamic_reference_confirmed: $("#bev-dynamic-reference").checked});
});
for (const action of ["save", "run"]) $("#bev-" + action).addEventListener("click", () => {
  if (!bevData) return;
  bevAction(action === "save" ? "points" : "run", {image_id: bevData.snapshot.image_id, points: bevPoints});
});
$("#bev-publish-reviewed").addEventListener("change", bevPublishState);
$("#bev-publish").addEventListener("click", () => {
  if (!bevData || bevDirty) return;
  bevAction("publish", {image_id: bevData.snapshot.image_id, review_confirmed: $("#bev-publish-reviewed").checked});
});
document.addEventListener("field-session", event => renderBev(event.detail));
renderBev(state.session);

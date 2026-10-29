/* BEV operates on a frozen undistorted photo, never on a resized pixel grid. */
let bevData = null;
let bevPoints = [];
let bevSelected = 0;
let bevKey = "";
let bevBusy = false;
let bevDirty = false;
const bevPhoto = new Image();
const blankPoint = (role = "fit") => ({role, x: "", y: "", u: null, v: null});
const bevMessage = (text) => { $("#bev-message").textContent = text; };
const bevUrl = (name) => `/api/bev-image?session=${encodeURIComponent(state.session.id)}&name=${name}&t=${Date.now()}`;

function renderBev(session) {
  $("#bev-panel").hidden = !session;
  if (!session) { bevKey = ""; return; }
  bevData = session.bev;
  $("#bev-editor").hidden = !bevData;
  const key = `${session.id}:${bevData?.snapshot?.image_id || ""}`;
  if (key !== bevKey) {
    bevKey = key;
    bevSelected = 0;
    bevDirty = false;
    $("#bev-publish-reviewed").checked = false;
    $("#bev-reference-confirmed").checked = !!bevData?.reference?.reference_confirmed;
    $("#bev-intrinsic-confirmed").checked = !!bevData?.reference?.intrinsic_confirmed;
    $("#bev-intrinsic").value = bevData?.reference?.intrinsic_source || "configured";
    bevPoints = bevData?.points?.points || Array.from({length: 12}, (_, i) => blankPoint(i < 8 ? "fit" : "check"));
    if (bevData) {
      bevPhoto.src = bevUrl("snapshot.jpg");
      bevRows();
    }
  }
  bevResult();
  const runtime = session.runtime_calibration;
  if (runtime) $("#bev-runtime-path").textContent = `算法参数固定目录：${runtime.current_path}；${runtime.active ? `当前已复核版本 ${runtime.active.release_id}` : "尚无已复核版本"}。目录中的 sdk-calibration.json 和 observer.calibration.yaml 分别供 SDK 与 ROS 接入时读取。`;
  bevPublishState();
}
function bevPublishState() {
  $("#bev-publish").disabled = bevBusy || bevDirty || !bevData?.summary?.checks_passed || state.session?.metadata?.mode !== "hardware" || !$("#bev-publish-reviewed").checked;
}
function bevResult() {
  const summary = bevDirty ? null : bevData?.summary;
  $("#bev-state").textContent = summary ? (summary.checks_passed ? "几何检查通过 · 待复核" : "检查未通过") : bevData ? "填写与点选中" : "未开始";
  $("#bev-result").hidden = !summary;
  if (!summary) return;
  $("#bev-metrics").textContent = `${bevData.snapshot.mode === "mock" ? "模拟流程；" : ""}标定内点 ${summary.fit_inliers}/${summary.fit.count}；独立检查 ${summary.check.count} 点，RMSE ${(summary.check.rmse_m * 100).toFixed(2)} cm，最大误差 ${(summary.check.max_m * 100).toFixed(2)} cm。阈值：RMSE ≤ 5 cm，最大 ≤ 10 cm。候选参数尚未启用。`;
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
}
function bevDraw() {
  if (!bevPhoto.complete || !bevPhoto.naturalWidth) return;
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
}
bevPhoto.onload = bevDraw;
bevPhoto.onerror = () => bevMessage("照片加载失败，请刷新页面后重试。");
function bevEdited() { bevDirty = true; $("#bev-publish-reviewed").checked = false; bevPublishState(); bevResult(); bevMessage("点位已修改，请保存点位或重新计算。"); }
$("#bev-points").addEventListener("input", event => {
  const el = event.target;
  if (!el.dataset.key) return;
  bevPoints[Number(el.dataset.index)][el.dataset.key] = el.value;
  bevEdited(); bevDraw();
});
$("#bev-points").addEventListener("click", event => {
  if (bevBusy) return;
  const el = event.target;
  if (el.dataset.select !== undefined) bevSelected = Number(el.dataset.select);
  else if (el.dataset.remove !== undefined && bevPoints.length > 1) {
    bevPoints.splice(Number(el.dataset.remove), 1);
    bevSelected = Math.min(bevSelected, bevPoints.length - 1); bevEdited();
  } else return;
  bevRows();
});
function bevSetPixel(u, v) {
  if (bevBusy || !bevData || !Number.isFinite(u) || !Number.isFinite(v) || u < 0 || v < 0 || u >= bevData.snapshot.width || v >= bevData.snapshot.height) {
    bevMessage("点选位置必须在原始照片范围内。"); return;
  }
  Object.assign(bevPoints[bevSelected], {u, v}); bevEdited(); bevRows();
}
$("#bev-canvas").addEventListener("click", event => {
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
  bevPoints.push(blankPoint()); bevSelected = bevPoints.length - 1; bevEdited(); bevRows();
});
async function bevAction(action, payload) {
  if (bevBusy) return;
  bevBusy = true;
  $("#bev-panel").querySelectorAll("button,input,select").forEach(el => { el.disabled = true; });
  bevMessage(action === "prepare" ? "正在拍照和去畸变，请保持相机与地面标记不动…" : "正在保存并处理地面点…");
  try {
    const result = await api(`/api/sessions/current/bev/${action}`, {method: "POST", body: JSON.stringify(payload)});
    // A newly prepared frame clears the local editor even when replay bytes match.
    if (action === "prepare") bevKey = "";
    bevDirty = false;
    renderSession(result.session);
    if (action === "run") $("#bev-publish-reviewed").checked = false;
    bevMessage(action === "publish" ? "已保存为算法参数并保留旧版本。正在运行的算法仍使用原参数，下次初始化时加载新版本。" : action === "prepare" ? "照片已冻结并去畸变。请填写距离并点选地面标记。" : action === "run" ? "BEV 已计算，请检查误差和鸟瞰图，再导出记录。" : "地面点已保存，刷新页面后可继续。");
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
  }
}
$("#bev-prepare").addEventListener("click", () => {
  if (bevData && !window.confirm("重新拍摄将清空本会话已有地面点和 BEV 结果。确定继续吗？")) return;
  bevAction("prepare", {reference_confirmed: $("#bev-reference-confirmed").checked,
    intrinsic_confirmed: $("#bev-intrinsic-confirmed").checked, intrinsic_source: $("#bev-intrinsic").value});
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

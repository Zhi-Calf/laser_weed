(() => {
  let busy = false;
  let current = null;
  const el = id => document.getElementById(id);
  function render(data) {
    current = data;
    const live = data.live;
    el('attitude-live').textContent = live.available
      ? `横滚 ${live.roll_deg.toFixed(2)}° · 俯仰 ${live.pitch_deg.toFixed(2)}° · 接收距今 ${live.age_ms.toFixed(0)} ms`
      : `姿态不可用：${live.reason}`;
    el('attitude-source').textContent = live.available
      ? `安装标识：${live.installation_id} · 来源：${live.topic} · 坐标帧：${live.frame_id}` : '';
    const baseline = data.baseline ? '已记录基准。' : '下一动作前请记录基准。';
    el('attitude-results').textContent = baseline + ['nose_up', 'left_up'].map((key, i) => {
      const e = data.evidence[key];
      return `${i === 0 ? '车头抬高' : '左侧抬高'}：${e ? `${e.passed ? '通过' : '不通过，请检查动作、单位及轴向'}（横滚变化 ${e.roll_delta_deg.toFixed(2)}°，俯仰变化 ${e.pitch_delta_deg.toFixed(2)}°）` : '未检查'}`;
    }).join('；');
    el('attitude-saved').textContent = data.saved_matches_installation
      ? `当前安装已保存方向核查，核查人：${data.saved.operator}。这不会自动开启动态 BEV。`
      : data.saved ? '存在历史核查记录，但当前数据源不可用或安装标识不匹配，不能认定已确认。' : '当前没有正式方向核查记录。';
    el('attitude-baseline').disabled = busy || !live.available;
    for (const id of ['attitude-nose', 'attitude-left']) el(id).disabled = busy || !live.available || !data.baseline;
    el('attitude-save').disabled = busy || !live.available ||
      !['nose_up', 'left_up'].every(key => data.evidence[key]?.passed);
  }
  async function refresh() {
    if (busy) return;
    try { render(await api('/api/attitude')); }
    catch (e) {
      current = null;
      el('attitude-live').textContent = `连接中断：${e.message}`;
      el('attitude-saved').textContent = '无法核验当前安装状态';
      for (const id of ['attitude-baseline', 'attitude-nose', 'attitude-left', 'attitude-save']) el(id).disabled = true;
    }
  }
  async function act(action) {
    if (busy) return;
    busy = true;
    if (current) render(current);
    try {
      const data = await api('/api/attitude/action', {method: 'POST', body: JSON.stringify({action,
        performed: el('attitude-performed').checked, review_confirmed: el('attitude-reviewed').checked,
        operator: el('attitude-operator').value})});
      el('attitude-performed').checked = false;
      el('attitude-reviewed').checked = false;
      el('attitude-message').textContent = action === 'save' ? '方向核查已保存。' : '操作已记录，请查看核查结果。';
      current = data;
    } catch (e) { el('attitude-message').textContent = e.message; }
    finally { busy = false; await refresh(); }
  }
  for (const [id, action] of Object.entries({'attitude-baseline':'baseline', 'attitude-nose':'nose_up',
    'attitude-left':'left_up', 'attitude-save':'save', 'attitude-reset':'reset'})) {
    el(id).addEventListener('click', () => {
      if (action === 'reset' && !window.confirm('清除当前安装的方向核查记录？')) return;
      act(action);
    });
  }
  refresh();
  setInterval(refresh, 1000);
})();

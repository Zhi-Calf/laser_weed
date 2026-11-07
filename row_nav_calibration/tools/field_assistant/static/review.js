/* Review images are read-only; fitting and independent validation stay separate. */
(() => {
  let session = null;
  let reportKey = '';
  let requestId = 0;
  function renderPhotos() {
    const filter = $('#intrinsic-review-group').value;
    const rows = (session?.intrinsic_review || []).filter(row => filter === 'all' || row.group === filter)
      .slice().sort((a,b) => (b.rms_px ?? -1) - (a.rms_px ?? -1));
    $('#intrinsic-review-rows').replaceChildren(...rows.map(row => {
      const tr = document.createElement('tr');
      for (const text of [row.group === 'intrinsic' ? '拟合' : '独立验证',
        row.slot == null ? row.image : `第 ${row.slot+1} 张 · ${row.image}`,
        row.rms_px == null ? '—' : row.rms_px.toFixed(3)]) {
        const cell = document.createElement('td'); cell.textContent = text; tr.append(cell);
      }
      const actions = document.createElement('td');
      for (const [kind,label,available] of [['original','原图',row.original_available],
        ['comparison','校正前后对比',row.comparison_available]]) {
        if (!available) continue;
        const button = document.createElement('button'); button.type = 'button';
        button.className = 'button button-quiet'; button.textContent = label;
        button.addEventListener('click', () => showPhoto(row,kind)); actions.append(button);
      }
      tr.append(actions); return tr;
    }));
    if (rows.length && !$('#intrinsic-review-viewer').children.length) $('#intrinsic-review-status').textContent = `共 ${rows.length} 张，选择原图或校正前后对比进行检查。`;
    if (!rows.length) $('#intrinsic-review-status').textContent = '这份报告没有可对应到照片的逐图记录；旧报告不会猜测照片与误差的对应关系。';
  }
  function showPhoto(row,kind) {
    const token = ++requestId;
    const url = `/api/intrinsic-image?session=${encodeURIComponent(session.id)}&group=${row.group}&index=${row.index}&kind=${kind}`;
    $('#intrinsic-review-status').textContent = '正在加载检查图片…';
    const image = document.createElement('img'); image.hidden = true;
    image.alt = kind === 'comparison' ? '左侧原图，右侧去畸变图' : '内参标定原始照片';
    image.onload = () => {
      if (token !== requestId) return;
      image.hidden = false;
      $('#intrinsic-review-status').textContent = kind === 'comparison'
        ? '左侧原图，右侧去畸变。检查边缘直线、黑边和有效视野，不能只看误差数字。' : '原始照片：检查清晰度、反光和标定板是否完整。';
    };
    image.onerror = () => {
      if (token !== requestId) return;
      $('#intrinsic-review-status').textContent = '检查图片加载失败，请重新点击该照片重试。';
    };
    $('#intrinsic-review-viewer').replaceChildren(image);
    const link = $('#intrinsic-review-open'); link.href = url; link.hidden = false;
    image.src = url;
  }
  function render(data) {
    session = data;
    $('#intrinsic-photo-report').hidden = !session?.intrinsic_summary;
    const key = `${session?.id}:${session?.metadata?.intrinsic_run?.completed_at || ''}`;
    if (key !== reportKey) {
      reportKey = key; requestId++;
      $('#intrinsic-review-viewer').replaceChildren(); $('#intrinsic-review-open').hidden = true;
      $('#intrinsic-review-status').textContent = '';
    }
    renderPhotos();
  }
  $('#intrinsic-review-group').addEventListener('change', () => {
    requestId++; $('#intrinsic-review-viewer').replaceChildren(); $('#intrinsic-review-open').hidden = true; renderPhotos();
  });
  document.addEventListener('field-session', event => render(event.detail));
  document.addEventListener('field-intrinsics-complete', () => {
    if (!$('#intrinsic-view').hidden) $('#intrinsic-photo-report').scrollIntoView({block:'start'});
  });
  render(state.session);
})();

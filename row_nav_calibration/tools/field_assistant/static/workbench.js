/* Presentation and navigation only. Calibration and publication stay in their existing services. */
(() => {
  const steps = ['setup', 'intrinsic', 'bev', 'review'];
  let sessionId = null;
  let preparedImage;
  let pendingTarget = null;
  const key = () => `row-nav-view:${state.session?.id || 'new'}`;

  function navigate(target, focus = false) {
    if (!steps.includes(target) || (target !== 'setup' && !state.session)) return;
    if (focus && state.loaded === false) pendingTarget = target;
    for (const name of steps) {
      $(`#${name}-view`).hidden = name !== target;
      const button = $(`#nav-${name}`);
      if (name === target) button.setAttribute('aria-current', 'step');
      else button.removeAttribute('aria-current');
    }
    try { localStorage.setItem(key(), target); } catch (_) {}
    if (focus) {
      $(`#${target}-view h2`).focus({preventScroll: true});
      window.scrollTo({top: 0, behavior: 'instant'});
    }
  }

  function dynamicFields() {
    $('#dynamic-options').hidden = !$('#bev-dynamic').checked;
  }

  function render() {
    const session = state.session;
    for (const name of steps) {
      $(`#nav-${name}`).disabled = name !== 'setup' && !session;
      $(`#nav-${name}`).title = name !== 'setup' && !session ? '请先建立本次会话' : '';
    }
    $('#operator-label').textContent = session ? `操作人员：${session.metadata.operator}` : '尚未建立会话';
    $('#nav-session').textContent = session?.id || '';
    $('#step-state-setup').textContent = session ? '会话已建立' : '待建立会话';
    const counts = session?.capture_counts || {};
    $('#step-state-intrinsic').textContent = session?.intrinsic_summary ? '候选内参' : session ? `拟合 ${counts.intrinsic || 0} · 验证 ${counts.validation || 0}` : '';
    const bev = session?.bev;
    const dirty = typeof bevDirty !== 'undefined' && bevDirty;
    const draft = !!bev?.draft || dirty;
    $('#step-state-bev').textContent = draft ? '点位草稿' : bev?.summary ? '已计算' : bev ? '点选中' : '';
    $('#step-state-review').textContent = draft ? '待重新计算' : session?.runtime_calibration?.published ? '已保存版本' : bev?.summary ? '待复核' : '';
    $('#review-summary').textContent = draft ? '点位仍为草稿。请补齐并重新计算，再检查误差与候选参数。' : bev?.summary
      ? bev.summary.checks_passed ? '几何检查通过，仍需人工复核图像和参考坐标；尚未因此自动启用新参数。' : '几何检查未通过。查看下方误差，返回外参标定修改点位后重新计算。'
      : session?.intrinsic_summary ? '已生成候选内参。可返回内参采集查看拟合与独立验证误差；完成外参计算后再复核整套参数。'
      : '尚未生成外参候选结果。可以先采集内参，或使用已复核内参进行外参标定。';
    const publication = session?.runtime_calibration?.state || {};
    const candidateLabels = {not_started:'尚未计算',draft:'点位草稿',checks_failed:'几何检查未通过',
      pending_review:'候选结果 · 待人工复核',published_default:'本次候选已发布为默认参数',published_replaced:'本次候选已发布过，当前默认版本已改变'};
    $('#candidate-state-label').textContent = draft ? '点位草稿 · 需重新计算' : candidateLabels[publication.candidate] || '候选状态未知';
    $('#default-release-label').textContent = publication.default_release || '尚无可确认的默认版本';
    $('#runtime-load-label').textContent = '未知：未接入算法进程加载回执';
    if (!draft && publication.candidate === 'published_default') {
      $('#review-summary').textContent = '本次候选已发布为默认参数。运行程序是否加载仍未知；需重新初始化并确认。';
      $('#step-state-review').textContent = '已发布 · 加载未知';
    }
    dynamicFields();
    const imageKey = session?.bev ? `${session.id}:${session.bev.snapshot.image_id}` : '';
    if (preparedImage !== imageKey) {
      preparedImage = imageKey;
      $('#bev-preparation').open = !session?.bev;
    }
    $('#bev-preparation-label').textContent = session?.bev ? '地面照片已冻结 · 展开查看设置或重新拍摄' : '地面拍照与参考设置';
    if (sessionId !== session?.id || (pendingTarget && state.loaded)) {
      sessionId = session?.id;
      let saved;
      try { saved = localStorage.getItem(key()); } catch (_) {}
      navigate(pendingTarget || (session ? (steps.includes(saved) ? saved : 'intrinsic') : 'setup'));
      pendingTarget = null;
    }
  }

  document.addEventListener('click', event => {
    const button = event.target.closest('[data-step-target]');
    if (button) navigate(button.dataset.stepTarget, true);
  });
  document.addEventListener('field-session', render);
  document.addEventListener('field-capture', render);
  document.addEventListener('field-bev-edit', render);
  document.addEventListener('field-navigate', event => navigate(event.detail, true));
  document.addEventListener('field-bev-operation', event => {
    render();
    if (event.detail === 'prepare') $('#bev-preparation').open = false;
    if (event.detail === 'run' || event.detail === 'publish') navigate('review', true);
  });
  $('#bev-dynamic').addEventListener('change', dynamicFields);
  $('#open-attitude-settings').addEventListener('click', () => {
    navigate('setup', true);
    $('#attitude-settings').open = true;
    $('#attitude-settings').scrollIntoView({block: 'start'});
  });
  $('#help-button').addEventListener('click', () => {
    navigate('setup', true);
    $('#help-details').open = true;
    $('#help-details').scrollIntoView({block: 'start'});
  });
  render();
})();

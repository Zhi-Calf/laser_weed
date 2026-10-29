// Run with node --test tests/js/test_bev_retry.cjs; no browser dependencies.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../../tools/field_assistant/static/bev.js'), 'utf8');
function setup(refreshFails = false) {
  const nodes = new Map();
  function $(id) {
    if (!nodes.has(id)) nodes.set(id, {checked: false, disabled: false, hidden: false,
      addEventListener() {}, querySelectorAll() {return [];}, replaceChildren() {}});
    return nodes.get(id);
  }
  const session = {id: 's', metadata: {mode: 'hardware'}, bev: {
    snapshot: {image_id: 'photo', mode: 'hardware'}, points: {points: [{role:'fit', x:1, y:0, u:10, v:20}]},
    summary: {checks_passed:true, fit_inliers:8, fit:{count:8}, check:{count:4, rmse_m:0, max_m:0}, point_errors_m:[]}}};
  const calls = [];
  const ctx = vm.createContext({$, state:{session}, Image:class {}, escapeHtml:String,
    document:{addEventListener(){}, createElement(){return {};}}, window:{confirm(){return true;}},
    api:async (url) => {
      calls.push(url);
      if (url !== '/api/sessions/current') throw Error('calculation failed');
      if (refreshFails) throw Error('offline');
      return {session:{...session, bev:{...session.bev, summary:null}}};
    }});
  vm.runInContext('function renderSession(s) { state.session=s; renderBev(s); }', ctx);
  vm.runInContext(source, ctx);
  $('#bev-publish-reviewed').checked = true;
  vm.runInContext('bevPoints[0].x=2; bevPublishState()', ctx);
  return {ctx, $, calls};
}
for (const refreshFails of [false, true]) {
  test(`failed calculation hides old result and preserves edits, refreshFails=${refreshFails}`, async () => {
    const {ctx,$,calls} = setup(refreshFails);
    await vm.runInContext("bevAction('run', {})", ctx);
    assert.equal($('#bev-result').hidden, true);
    assert.equal($('#bev-publish').disabled, true);
    assert.equal($('#bev-publish-reviewed').checked, false);
    assert.equal(vm.runInContext('bevPoints[0].x',ctx), 2);
    assert.equal(vm.runInContext('bevBusy',ctx), false);
    assert.match($('#bev-message').textContent, /calculation failed/);
    if (refreshFails) assert.match($('#bev-message').textContent, /无法刷新/);
    assert.equal(calls.length,2);
    // A successful retry clears the dirty state and restores the result for review.
    ctx.api = async () => ({session:{id:'s',metadata:{mode:'hardware'},bev:{snapshot:{image_id:'photo'},
      summary:{checks_passed:true,fit_inliers:8,fit:{count:8},check:{count:4,rmse_m:0,max_m:0},point_errors_m:[]}}}});
    await vm.runInContext("bevAction('run', {})",ctx);
    assert.equal($('#bev-result').hidden,false);
    assert.equal($('#bev-publish').disabled,true);
    $('#bev-publish-reviewed').checked=true;
    vm.runInContext('bevPublishState()',ctx);
    assert.equal($('#bev-publish').disabled,false);
  });
}

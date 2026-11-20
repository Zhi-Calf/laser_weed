// Run with node --test tests/js/test_bev_retry.cjs; no browser dependencies.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../../tools/field_assistant/static/bev.js'), 'utf8');
function setup(refreshFails = false) {
  const nodes = new Map();
  const images = [];
  const context2d = new Proxy({}, {get(target, key) {return target[key] || (()=>{});}});
  function $(id) {
    if (!nodes.has(id)) nodes.set(id, {checked: false, disabled: false, hidden: false,
      style:{}, dataset:{}, clientWidth:800, clientHeight:600,
      setAttribute(){}, getContext(){return context2d;},
      addEventListener() {}, querySelectorAll() {return [];}, replaceChildren() {}});
    return nodes.get(id);
  }
  const session = {id: 's', metadata: {mode: 'hardware'}, bev: {
    snapshot: {image_id: 'photo', mode: 'hardware', width:800, height:600}, points: {points: [{role:'fit', x:1, y:0, u:10, v:20}]},
    summary: {checks_passed:true, fit_inliers:8, fit:{count:8}, check:{count:4, rmse_m:0, max_m:0}, point_errors_m:[]}}};
  const calls = [];
  const storage = new Map();
  const handlers = {};
  const ctx = vm.createContext({$, state:{session}, Image:class { constructor(){this.complete=true;this.naturalWidth=800;this.naturalHeight=600;images.push(this);} }, escapeHtml:String,
    document:{addEventListener(){}, dispatchEvent(){}, createElement(){return {};}},
    CustomEvent: class { constructor(type, options) { this.type=type; this.detail=options?.detail; } }, window:{confirm(){return true;}, addEventListener(name, callback){handlers[name]=callback;}},
    localStorage:{getItem(key){return storage.get(key) || null;}, setItem(key,value){storage.set(key,value);}, removeItem(key){storage.delete(key);}},
    api:async (url) => {
      calls.push(url);
      if (url !== '/api/sessions/current') throw Error('calculation failed');
      if (refreshFails) throw Error('offline');
      return {session:{...session, bev:{...session.bev, summary:null}}};
    }});
  vm.runInContext('function renderSession(s) { state.session=s; renderBev(s); }', ctx);
  vm.runInContext(source, ctx);
  $('#bev-publish-reviewed').checked = true;
  vm.runInContext('bevImageReady=true; bevPoints[0].x=2; bevPublishState()', ctx);
  return {ctx, $, calls, storage, handlers, images};
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

test('local draft survives editor reload and blocks publication', () => {
  const {ctx, $, storage, handlers} = setup();
  vm.runInContext('bevEdited()', ctx);
  assert.equal(storage.size, 1);
  let prevented = false;
  handlers.beforeunload({preventDefault(){prevented=true;}});
  assert.equal(prevented, true);
  vm.runInContext('bevKey=""; renderBev(state.session)', ctx);
  assert.equal(vm.runInContext('bevPoints[0].x', ctx), 2);
  assert.equal(vm.runInContext('bevDirty', ctx), true);
  assert.equal($('#bev-result').hidden, true);
  assert.equal($('#bev-publish').disabled, true);
});
test('device draft restores partial points and never exposes old result', () => {
  const {ctx, $} = setup();
  vm.runInContext('state.session.bev.draft={points:[{role:"fit",x:"",y:"",u:null,v:null}]}; bevKey=""; renderBev(state.session)', ctx);
  assert.equal(vm.runInContext('bevPoints[0].x', ctx), '');
  assert.equal($('#bev-result').hidden, true);
  assert.equal($('#bev-publish').disabled, true);
});

test('loading, failed and dimension-mismatched images cannot change pixel coordinates', () => {
  const {ctx, $, images} = setup();
  vm.runInContext('loadBevPhoto(); bevSetPixel(0,0)', ctx);
  assert.equal(vm.runInContext('bevPoints[0].u',ctx),10);
  assert.equal($('#bev-run').disabled,true);
  images.at(-1).onerror();
  assert.equal($('#bev-image-retry').hidden,false);
  assert.equal(vm.runInContext('bevImageReady',ctx),false);
  vm.runInContext('loadBevPhoto()',ctx);
  images.at(-1).naturalWidth=123;
  images.at(-1).onload();
  assert.match($('#bev-image-status').textContent,/尺寸/);
  assert.equal(vm.runInContext('bevPoints[0].u',ctx),10);
});
test('late image responses cannot unlock a newer request, retry keeps edits', () => {
  const {ctx,$,images}=setup();
  const old=images.at(-1);
  vm.runInContext('loadBevPhoto()',ctx);
  old.onload();
  assert.equal(vm.runInContext('bevImageReady',ctx),false);
  images.at(-1).onload();
  assert.equal(vm.runInContext('bevImageReady',ctx),true);
  assert.equal(vm.runInContext('bevPoints[0].x',ctx),2);
  assert.equal($('#bev-run').disabled,false);
});
test('pixel undo restores both original coordinates and leaves result requiring recalculation', () => {
  const {ctx,$}=setup();
  vm.runInContext('bevSetPixel(123.4,234.5)',ctx);
  assert.equal(vm.runInContext('bevPoints[0].u',ctx),123.4);
  vm.runInContext('undoBevEdit()',ctx);
  assert.equal(vm.runInContext('bevPoints[0].u',ctx),10);
  assert.equal(vm.runInContext('bevPoints[0].v',ctx),20);
  assert.equal(vm.runInContext('bevDirty',ctx),true);
  assert.equal($('#bev-publish').disabled,true);
});

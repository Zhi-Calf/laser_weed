// Isolated mock service; generate data/field-import/synthetic with generate_charuco_fixture.py first.
const {chromium}=require('playwright');
const assert=require('node:assert/strict');
const base=process.env.CALIBRATION_TEST_URL;
if(!base)throw Error('Set CALIBRATION_TEST_URL to an isolated mock service with synthetic imports.');
(async()=>{const browser=await chromium.launch({headless:true,...(process.env.CHROME_PATH?{executablePath:process.env.CHROME_PATH}:{})});
try{
  const page=await browser.newPage({viewport:{width:1360,height:1000}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  assert.equal((await(await page.request.get(base+'/api/status')).json()).mode,'mock');
  const created=await page.request.post(base+'/api/sessions/start',{data:{operator:'报告与小图缓存测试',square_width_mm:56.5,square_height_mm:56.5,marker_mm:42.2,check_line_mm:100,board_flat:true}});
  assert.equal(created.status(),201);
  for(const slot of [0,1])assert.equal((await page.request.post(base+'/api/sessions/current/capture',{data:{group:'intrinsic',slot}})).status(),200);
  await page.goto(base);await page.waitForFunction(()=>state.loaded);
  await page.click('#nav-intrinsic');
  await page.waitForFunction(()=>[...document.querySelectorAll('#capture-thumbnails img')].length===2 && [...document.querySelectorAll('#capture-thumbnails img')].every(i=>i.naturalWidth>0));
  await page.evaluate(()=>{window.testThumbs=[...document.querySelectorAll('#capture-thumbnails button')];});
  const old=await page.locator('#capture-thumbnails img').evaluateAll(nodes=>nodes.map(i=>i.src));
  for(const src of old){
    const response=await page.request.get(src);assert.match(response.headers()['cache-control'],/immutable/);
    const etag=response.headers()['etag'];assert.equal((await page.request.get(src,{headers:{'If-None-Match':etag}})).status(),304);
  }
  for(const i of [0,1,0,1]){
    await page.locator(`#capture-grid [data-position="${i}"]`).click();
    await page.waitForFunction(()=>captureImageReady);
  }
  assert.equal(await page.evaluate(()=>window.testThumbs.every((n,i)=>n===document.querySelectorAll('#capture-thumbnails button')[i])),true);
  await page.request.post(base+'/api/sessions/current/capture',{data:{group:'intrinsic',slot:1}});
  await page.evaluate(()=>refresh());
  const fresh=await page.locator('#capture-thumbnails img').evaluateAll(nodes=>nodes.map(i=>i.src));
  assert.equal(fresh[0],old[0]);assert.notEqual(fresh[1],old[1]);
  assert.equal(await page.evaluate(()=>window.testThumbs.every((n,i)=>n===document.querySelectorAll('#capture-thumbnails button')[i])),true);
  assert.ok(await page.locator('#capture-thumbnails img').first().evaluate(i=>i.naturalWidth<=192));
  await page.locator('.import-settings summary').click();
  await page.click('#run-intrinsic-button');
  await page.waitForFunction(()=>state.session?.intrinsic_summary,{},{timeout:60000});
  const data=(await(await page.request.get(base+'/api/sessions/current')).json()).session;
  assert.equal(data.intrinsic_review.length,data.intrinsic_summary.training.usable+data.intrinsic_summary.validation.usable);
  assert.equal(await page.locator('#intrinsic-review-rows tr').count(),data.intrinsic_review.length);
  const errorsInOrder=await page.locator('#intrinsic-review-rows tr').evaluateAll(nodes=>nodes.map(n=>Number(n.cells[2].textContent)));
  assert.ok(errorsInOrder.every((n,i)=>!i || n<=errorsInOrder[i-1]));
  await page.selectOption('#intrinsic-review-group','validation');
  assert.equal(await page.locator('#intrinsic-review-rows tr').count(),data.intrinsic_summary.validation.usable);
  await page.locator('#intrinsic-review-rows button').filter({hasText:'校正前后对比'}).first().click();
  await page.waitForFunction(()=>document.querySelector('#intrinsic-review-viewer img')?.naturalWidth===3840);
  await page.locator('#intrinsic-review-rows button').filter({hasText:/^原图$/}).first().click();
  await page.waitForFunction(()=>document.querySelector('#intrinsic-review-viewer img')?.naturalWidth===1920);
  await page.click('#nav-review');
  assert.match(await page.locator('#runtime-load-label').textContent(),/未知/);
  assert.match(await page.locator('#default-release-label').textContent(),/尚无/);
  await page.setViewportSize({width:390,height:844});
  assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
  assert.deepEqual(errors,[]);
  console.log(JSON.stringify({smallVersionedThumbnails:true,conditional304:true,stableThumbnailNodes:true,onlyRetakenImageInvalidated:true,
    realIntrinsicReport:true,sortedPerPhotoErrors:true,independentComparisonImages:true,runtimeLoadUnknown:true,mobileOverflow:false,javascriptErrors:errors}));
}finally{await browser.close();}})().catch(e=>{console.error(e);process.exit(1)});

// Requires an isolated mock service with a decodable preview image and calibration Python.
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const base = process.env.CALIBRATION_TEST_URL;
if (!base) throw Error('Set CALIBRATION_TEST_URL to an isolated mock service. This test creates a session.');
const closeTo = (a,b,tolerance=.02) => assert.ok(Math.abs(a-b)<tolerance, `${a} != ${b}`);
(async () => {
  const browser = await chromium.launch({headless:true,
    ...(process.env.CHROME_PATH ? {executablePath:process.env.CHROME_PATH} : {})});
  try {
    const page = await browser.newPage({viewport:{width:1360,height:1000}});
    const errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    assert.equal((await (await page.request.get(base+'/api/status')).json()).mode,'mock');
    const response=await page.request.post(base+'/api/sessions/start',{data:{operator:'精细选点与质量检查回归',
      square_width_mm:56.5,square_height_mm:56.5,marker_mm:42.2,check_line_mm:100,board_flat:true}});
    assert.equal(response.status(),201);
    let failCapture=true;
    await page.route('**/api/capture-image?**',route=>{
      if(failCapture && new URL(route.request().url()).searchParams.get('kind')==='raw') {
        failCapture=false; return route.abort();
      }
      return route.continue();
    });
    let initialSession;
    let releaseSession;
    const sessionRequested=new Promise(resolve=>{releaseSession=resolve;});
    await page.route('**/api/sessions/current',route=>{initialSession=route;releaseSession();});
    await page.goto(base);
    await sessionRequested;
    await page.click('#nav-setup');
    await initialSession.continue();
    await page.waitForFunction(()=>state.loaded);
    assert.equal(await page.locator('#nav-setup').getAttribute('aria-current'),'step');
    await page.unroute('**/api/sessions/current');
    await page.click('#nav-intrinsic');
    await page.locator('#capture-button').waitFor({state:'visible'});
    await page.click('#capture-button');
    await page.locator('#capture-image-retry').waitFor({state:'visible'});
    assert.equal(await page.locator('#capture-next').isDisabled(),true);
    await page.click('#capture-image-retry');
    await page.waitForFunction(()=>!document.querySelector('#capture-next').disabled);
    assert.match(await page.locator('#capture-quality-summary').textContent(),/角点/);
    const session=(await (await page.request.get(base+'/api/sessions/current')).json()).session;
    const quality=session.capture_quality.intrinsic['0'];
    assert.equal(quality.advisory_only,true);
    assert.equal(quality.overlay_available,true);
    assert.equal(session.capture_counts.intrinsic,1);
    await page.check('#capture-show-corners');
    await page.waitForFunction(()=>document.querySelector('#capture-review').src.includes('kind=corners') && !document.querySelector('#capture-next').disabled);

    let pending;
    let markPending;
    const pendingImage=new Promise(resolve=>{markPending=resolve;});
    await page.route('**/api/bev-image?**',route=>{
      if(new URL(route.request().url()).searchParams.get('name')==='snapshot.jpg') {
        pending=route; markPending(); return;
      }
      return route.continue();
    });
    await page.click('#nav-bev');
    await page.check('#bev-reference-confirmed');
    await page.check('#bev-intrinsic-confirmed');
    await page.click('#bev-prepare');
    await pendingImage;
    assert.equal(await page.locator('#bev-run').isDisabled(),true);
    assert.equal(await page.locator('#bev-set-pixel').isDisabled(),true);
    await page.evaluate(()=>document.querySelector('#bev-canvas').dispatchEvent(new MouseEvent('click',{clientX:10,clientY:10,bubbles:true})));
    assert.equal(await page.evaluate(()=>bevPoints[0].u),null);
    await page.locator('#bev-points input[data-key="x"]').first().fill('2.5');
    await pending.abort();
    await page.locator('#bev-image-retry').waitFor({state:'visible'});
    assert.equal(await page.evaluate(()=>bevPoints[0].x),'2.5');
    await page.unroute('**/api/bev-image?**');
    await page.click('#bev-image-retry');
    await page.waitForFunction(()=>document.querySelector('#bev-canvas').getAttribute('aria-disabled')==='false');
    const canvas=page.locator('#bev-canvas');
    const snap=(await (await page.request.get(base+'/api/sessions/current')).json()).session.bev.snapshot;
    const chooseCenter=async()=>{
      const box=await canvas.boundingBox();
      await canvas.click({position:{x:box.width/2,y:box.height/2}});
      // Mouse events are quantized to display pixels; subpixel edits are checked separately.
      closeTo(await page.evaluate(()=>bevPoints[0].u),snap.width/2,snap.width/box.width+1);
      closeTo(await page.evaluate(()=>bevPoints[0].v),snap.height/2,snap.height/box.height+1);
      assert.equal(await canvas.evaluate(node=>node.width),snap.width);
    };
    await chooseCenter();
    const initial=await page.evaluate(()=>({...bevPoints[0]}));
    await canvas.press('ArrowRight');
    await canvas.press('Shift+ArrowDown');
    closeTo(await page.evaluate(()=>bevPoints[0].u),initial.u+1);
    closeTo(await page.evaluate(()=>bevPoints[0].v),initial.v+.1);
    await canvas.press('Control+z');
    closeTo(await page.evaluate(()=>bevPoints[0].v),initial.v);
    const beforeZoom=await page.evaluate(()=>JSON.stringify(bevPoints));
    await page.click('#bev-zoom-in');
    await page.click('#bev-zoom-in');
    assert.equal(await page.evaluate(()=>JSON.stringify(bevPoints)),beforeZoom);
    await chooseCenter();
    const before=await page.evaluate(()=>JSON.stringify(bevPoints));
    const scrollBefore=await page.locator('#bev-image-scroll').evaluate(node=>node.scrollLeft);
    await page.click('#bev-pan');
    const rect=await page.locator('#bev-image-scroll').boundingBox();
    await page.mouse.move(rect.x+rect.width*.5,rect.y+rect.height*.5);
    await page.mouse.down();
    await page.mouse.move(rect.x+rect.width*.5-60,rect.y+rect.height*.5-20,{steps:5});
    await page.mouse.up();
    assert.equal(await page.evaluate(()=>JSON.stringify(bevPoints)),before);
    assert.ok((await page.locator('#bev-image-scroll').evaluate(node=>node.scrollLeft))>scrollBefore);
    await page.click('#bev-pan');
    await page.click('#bev-fit');
    await chooseCenter();
    assert.equal(await page.locator('#bev-loupe').isVisible(),true);
    await page.setViewportSize({width:390,height:844});
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    const lens=await page.locator('#bev-loupe').boundingBox();
    const frame=await page.locator('.bev-image-frame').boundingBox();
    assert.ok(lens.y>=frame.y && lens.x+ lens.width<=frame.x+frame.width+1);
    assert.deepEqual(errors,[]);
    console.log(JSON.stringify({initialNavigationPreserved:true,captureImageRetry:true,qualityGuidance:true,cornerOverlay:true,
      imageLoadGuard:true,imageFailureRecovery:true,originalPixelsAfterZoom:true,
      keyboardSubpixel:true,undo:true,panDoesNotSelect:true,mobileOverflow:false,javascriptErrors:errors}));
  } finally {await browser.close();}
})().catch(error=>{console.error(error);process.exit(1);});

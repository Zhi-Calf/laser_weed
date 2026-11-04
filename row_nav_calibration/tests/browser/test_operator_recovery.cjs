// Optional Playwright smoke test. Creates sessions on an isolated mock service.
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const base = process.env.CALIBRATION_TEST_URL;
if (!base) throw Error('Set CALIBRATION_TEST_URL to an isolated mock server. This test creates sessions.');

(async () => {
  const browser = await chromium.launch({
    headless: true,
    ...(process.env.CHROME_PATH ? {executablePath: process.env.CHROME_PATH} : {}),
  });
  try {
    const page = await browser.newPage({viewport: {width: 1360, height: 1000}});
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('dialog', dialog => dialog.accept());
    const status = await (await page.request.get(base + '/api/status')).json();
    assert.equal(status.mode, 'mock');
    await page.goto(base);
    await page.locator('input[name=operator]').fill('流程回归测试');
    await page.locator('input[name=board_flat]').check();
    await Promise.all([
      page.waitForResponse(response => response.url().endsWith('/api/sessions/start') && response.status() === 201),
      page.locator('#session-form button[type=submit]').click(),
    ]);
    await page.waitForFunction(() => document.querySelector('#message').textContent.includes('现场会话已建立'));

    await page.selectOption('#capture-round', '2');
    await page.locator('[data-position="4"]').click();
    await page.reload();
    await page.waitForFunction(() => document.querySelector('#capture-instruction').textContent.includes('第 23/27'));
    await page.click('#capture-button');
    await page.waitForFunction(() => !document.querySelector('#capture-review').hidden && document.querySelector('#capture-review').naturalWidth > 0);
    assert.match(await page.locator('[data-position="4"]').textContent(), /✓/);
    await page.reload();
    await page.waitForFunction(() => !document.querySelector('#capture-review').hidden);
    assert.equal(await page.locator('#capture-next').isDisabled(), false);

    await page.check('#bev-reference-confirmed');
    await page.check('#bev-intrinsic-confirmed');
    await page.click('#bev-prepare');
    await page.locator('#bev-editor').waitFor({state: 'visible'});
    await page.locator('#bev-points input[data-key="x"]').first().fill('2.345');
    await page.reload();
    await page.waitForFunction(() => document.querySelector('#bev-points input[data-key="x"]')?.value === '2.345');
    assert.equal(await page.locator('#bev-publish').isDisabled(), true);
    await page.click('#bev-save');
    await page.waitForFunction(() => document.querySelector('#bev-message').textContent.includes('草稿已保存到设备'));

    // Clear browser data to prove that the partial draft also exists on the server.
    await page.evaluate(() => localStorage.clear());
    await page.reload();
    await page.waitForFunction(() => document.querySelector('#bev-points input[data-key="x"]')?.value === '2.345');
    await page.click('#bev-run');
    await page.waitForFunction(() => document.querySelector('#bev-message').textContent.includes('未完成'));
    assert.equal(await page.locator('#bev-publish').isDisabled(), true);
    assert.equal(await page.locator('#bev-result').isHidden(), true);
    await page.waitForFunction(() => document.querySelector('#capture-instruction').textContent.includes('第 1/27'));
    assert.match(await page.locator('#capture-counts').textContent(), /拟合 1\/27/);
    if (process.env.CALIBRATION_SCREENSHOT) {
      await page.screenshot({path: process.env.CALIBRATION_SCREENSHOT, fullPage: true});
    }
    assert.deepEqual(errors, []);
    console.log(JSON.stringify({captureRoundResume: true, captureImageRestore: true,
      localDraftRestore: true, devicePartialDraftRestore: true,
      incompleteCalculationBlocked: true, javascriptErrors: errors}));
  } finally {
    await browser.close();
  }
})().catch(error => {console.error(error); process.exit(1);});

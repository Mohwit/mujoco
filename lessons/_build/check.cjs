// Headless check of one lesson in a built course page.
// Usage: PW=<path to node_modules/playwright> node check.cjs <page.html> <lessonN> <screenshotDir>
// Loads the page in dark, light and 390px-wide views, switches to lesson N, clicks every
// button inside #lessonN and #sandboxN, moves every range slider, and reports JS errors.
const { chromium } = require(process.env.PW || 'playwright');
const path = require('path'); const fs = require('fs');
(async () => {
  const [file, nStr, outDir = '.'] = process.argv.slice(2); const n = Number(nStr);
  fs.mkdirSync(outDir, { recursive: true });
  const url = 'file://' + path.resolve(file);
  const browser = await chromium.launch();
  const errors = [];
  for (const v of [{ name: 'dark', scheme: 'dark', w: 1440, h: 900 }, { name: 'light', scheme: 'light', w: 1440, h: 900 }, { name: 'mobile', scheme: 'dark', w: 390, h: 844 }]) {
    const page = await browser.newPage({ viewport: { width: v.w, height: v.h }, colorScheme: v.scheme });
    page.on('pageerror', e => errors.push(`[${v.name}] pageerror: ${e.message}`));
    page.on('console', m => { if (m.type() === 'error') errors.push(`[${v.name}] console: ${m.text()}`); });
    await page.goto(url); await page.waitForTimeout(300);
    await page.evaluate(k => switchLesson(k), n); await page.waitForTimeout(300);
    if (v.name === 'light') { await page.evaluate(() => { const t = document.getElementById('themeToggle'); if (document.documentElement.dataset.theme !== 'light') t.click(); }); await page.waitForTimeout(300); }
    await page.screenshot({ path: `${outDir}/lesson${n}-${v.name}-top.png` });
    await page.screenshot({ path: `${outDir}/lesson${n}-${v.name}-full.png`, fullPage: true });
    if (v.name === 'dark') {
      // Reveal predict answers, open maths boxes, move sliders, click every button.
      await page.evaluate(k => document.querySelectorAll(`#lesson${k} details`).forEach(d => d.open = true), n);
      for (const sel of [`#lesson${n} input[type=range]`, `#sandbox${n} input[type=range]`]) {
        for (const h of await page.$$(sel)) {
          await h.evaluate(el => { const lo = +el.min || 0, hi = +el.max || 100; for (const f of [0, 1, 0.5]) { el.value = lo + f * (hi - lo); el.dispatchEvent(new Event('input', { bubbles: true })); el.dispatchEvent(new Event('change', { bubbles: true })); } });
        }
      }
      const btns = await page.$$(`#lesson${n} button, #sandbox${n} button`);
      let clicked = 0;
      for (const b of btns) {
        const label = (await b.innerText()).trim();
        if (/^Next/.test(label)) continue;
        try { await b.evaluate(el => el.click()); clicked++; await page.waitForTimeout(120); } catch (e) { errors.push(`[click] ${label}: ${e.message}`); }
      }
      await page.waitForTimeout(1500);
      await page.screenshot({ path: `${outDir}/lesson${n}-dark-after-clicks-full.png`, fullPage: true });
      console.log(`clicked ${clicked} buttons`);
      // Theme toggle while on this lesson.
      await page.click('#themeToggle').catch(() => {}); await page.waitForTimeout(200);
      await page.click('#sbToggle').catch(() => {}); await page.waitForTimeout(200);
      await page.click('#sbToggle').catch(() => {});
    }
    if (v.name === 'mobile') {
      const overflow = await page.evaluate(() => document.documentElement.scrollWidth - window.innerWidth);
      if (overflow > 2) errors.push(`[mobile] horizontal overflow ${overflow}px`);
    }
    await page.close();
  }
  await browser.close();
  console.log(errors.length ? 'ERRORS:\n' + errors.join('\n') : 'OK: no JavaScript errors');
  process.exit(errors.length ? 1 : 0);
})();

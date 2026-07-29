import { existsSync, mkdirSync } from "node:fs";
import { resolve } from "node:path";
import puppeteer from "puppeteer-core";

const candidates = [
  process.env.CHROME_PATH,
  "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe",
  "C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe",
  "/usr/bin/google-chrome",
  "/usr/bin/chromium",
  "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
].filter(Boolean);
const executablePath = candidates.find((path) => existsSync(path));

if (!executablePath) {
  throw new Error("Chrome or Edge was not found. Set CHROME_PATH and retry.");
}

const url = process.env.SIMULATOR_URL ?? "http://127.0.0.1:4173/";
const artifacts = resolve("artifacts");
mkdirSync(artifacts, { recursive: true });
const consoleErrors = [];
const pageErrors = [];
const httpErrors = [];

const browser = await puppeteer.launch({
  executablePath,
  headless: true,
  args: [
    "--no-first-run",
    "--disable-background-networking",
    "--disable-extensions",
    "--disable-sync",
    "--enable-unsafe-swiftshader",
    "--use-angle=swiftshader",
  ],
});

try {
  const page = await browser.newPage();
  page.on("console", (message) => {
    if (message.type() === "error") consoleErrors.push(message.text());
  });
  page.on("pageerror", (error) => pageErrors.push(error.message));
  page.on("response", (response) => {
    if (response.status() >= 400) {
      httpErrors.push(`${response.status()} ${response.url()}`);
    }
  });
  await page.setViewport({ width: 1440, height: 900, deviceScaleFactor: 1 });
  const response = await page.goto(url, {
    waitUntil: "networkidle0",
    timeout: 60_000,
  });
  await page.waitForSelector(".model-loader", { hidden: true, timeout: 60_000 });
  await page.waitForSelector(".viewport-canvas canvas", { visible: true });
  await page.screenshot({ path: resolve(artifacts, "desktop-smoke.png") });

  const phaseBefore = await page.$eval(
    ".phase-strip .control-label b",
    (node) => node.textContent,
  );
  const distanceBefore = Number(
    await page.$eval(
      ".odometry-block",
      (node) => node.getAttribute("data-odometry-distance"),
    ),
  );
  await page.$eval(".run-button", (button) => button.click());
  await page
    .waitForFunction(
      (previous) =>
        document.querySelector(".phase-strip .control-label b")?.textContent !==
        previous,
      { timeout: 15_000 },
      phaseBefore,
    )
    .catch(() => undefined);
  const phaseAfter = await page.$eval(
    ".phase-strip .control-label b",
    (node) => node.textContent,
  );
  await page
    .waitForFunction(
      (previous) =>
        Number(
          document
            .querySelector(".odometry-block")
            ?.getAttribute("data-odometry-distance"),
        ) >
        previous + 0.002,
      { timeout: 15_000 },
      distanceBefore,
    )
    .catch(() => undefined);
  const distanceAfter = Number(
    await page.$eval(
      ".odometry-block",
      (node) => node.getAttribute("data-odometry-distance"),
    ),
  );
  const runningLabel = await page.$eval(
    ".run-button",
    (node) => node.textContent?.trim(),
  );
  await page.screenshot({ path: resolve(artifacts, "desktop-walking.png") });
  await page.$eval(".run-button", (button) => button.click());

  const readOdometry = async (field) =>
    Number(
      await page.$eval(
        ".odometry-block",
        (node, key) => node.getAttribute(`data-odometry-${key}`),
        field,
      ),
    );

  const gaitChecks = { tripod: distanceAfter > distanceBefore + 0.002 };
  for (const gaitName of ["ripple", "wave", "tetrapod"]) {
    await page.$$eval(
      ".gait-grid button",
      (buttons, label) =>
        buttons
          .find((button) => button.textContent?.toLowerCase().includes(label))
          ?.click(),
      gaitName,
    );
    const before = await readOdometry("distance");
    await page.$eval(".run-button", (button) => button.click());
    await page.waitForFunction(
      (previous) =>
        Number(
          document
            .querySelector(".odometry-block")
            ?.getAttribute("data-odometry-distance"),
        ) >
        previous + 0.001,
      { timeout: 15_000 },
      before,
    );
    const after = await readOdometry("distance");
    gaitChecks[gaitName] = after > before + 0.001;
    await page.$eval(".run-button", (button) => button.click());
  }

  const checkCommand = async (selector, field, direction, threshold = 0.001) => {
    await page.$eval(selector, (button) => button.click());
    const before = await readOdometry(field);
    await page.$eval(".run-button", (button) => button.click());
    await page.waitForFunction(
      (key, previous, sign, delta) => {
        const value = Number(
          document
            .querySelector(".odometry-block")
            ?.getAttribute(`data-odometry-${key}`),
        );
        return sign * (value - previous) > delta;
      },
      { timeout: 15_000 },
      field,
      before,
      direction,
      threshold,
    );
    const after = await readOdometry(field);
    await page.$eval(".run-button", (button) => button.click());
    return direction * (after - before) > threshold;
  };

  const commandChecks = {
    backward: await checkCommand(
      ".direction-layout button:nth-child(5)",
      "x",
      1,
    ),
    strafeLeft: await checkCommand(
      ".direction-layout button:nth-child(2)",
      "y",
      -1,
    ),
    strafeRight: await checkCommand(
      ".direction-layout button:nth-child(4)",
      "y",
      1,
    ),
    turnLeft: await checkCommand(".turn-row button:first-child", "yaw", -1, 0.005),
    turnRight: await checkCommand(".turn-row button:last-child", "yaw", 1, 0.005),
  };

  await page.$eval(".mode-tabs > button:nth-child(2)", (button) => button.click());
  await page.waitForSelector(".joint-stack", { visible: true });
  await page.$eval(".joint-stack input", (input) => {
    const slider = input;
    slider.value = "0.35";
    slider.dispatchEvent(new Event("input", { bubbles: true }));
    slider.dispatchEvent(new Event("change", { bubbles: true }));
  });
  const jointMode = Boolean(await page.$(".joint-values"));

  await page.$eval(".mode-tabs > button:nth-child(3)", (button) => button.click());
  await page.waitForSelector(".solver-result", { visible: true });
  await page.$eval(".parameter-stack input", (input) => {
    const slider = input;
    slider.value = "-0.1";
    slider.dispatchEvent(new Event("input", { bubbles: true }));
    slider.dispatchEvent(new Event("change", { bubbles: true }));
  });
  const ikMode = Boolean(await page.$(".matrix-readout"));

  await page.$eval(".mode-tabs > button:nth-child(4)", (button) => button.click());
  await page.waitForSelector(".pose-visual", { visible: true });
  const bodyMode = Boolean(await page.$(".pose-presets"));

  const desktop = await page.evaluate(() => ({
    title: document.title,
    canvas: Boolean(document.querySelector("canvas")),
    tabs: document.querySelectorAll(".mode-tabs > button").length,
    contactLegs: document.querySelectorAll(".contact-map button").length,
    status: document.querySelector(".status-online")?.textContent?.trim(),
    width: document.documentElement.scrollWidth,
    viewport: window.innerWidth,
  }));

  await page.setViewport({
    width: 412,
    height: 915,
    deviceScaleFactor: 1,
    isMobile: true,
    hasTouch: true,
  });
  await page.reload({ waitUntil: "networkidle0", timeout: 60_000 });
  await page.waitForSelector(".model-loader", { hidden: true, timeout: 60_000 });
  await page.screenshot({
    path: resolve(artifacts, "mobile-smoke.png"),
    fullPage: true,
  });
  const mobile = await page.evaluate(() => ({
    width: document.documentElement.scrollWidth,
    viewport: window.innerWidth,
    navScrollable:
      document.querySelector(".mode-tabs")?.scrollWidth >
      document.querySelector(".mode-tabs")?.clientWidth,
    canvasHeight: document.querySelector(".viewport-panel")?.clientHeight,
  }));

  const result = {
    httpStatus: response?.status(),
    desktop,
    mobile,
    gaitAdvanced: phaseBefore !== phaseAfter,
    worldLocomotionAdvanced: distanceAfter > distanceBefore + 0.002,
    gaitChecks,
    commandChecks,
    odometry: { distanceBefore, distanceAfter },
    runningLabel,
    modeChecks: { jointMode, ikMode, bodyMode },
    consoleErrors,
    pageErrors,
    httpErrors,
  };
  console.log(JSON.stringify(result, null, 2));

  if (
    response?.status() !== 200 ||
    !desktop.canvas ||
    desktop.tabs !== 4 ||
    desktop.contactLegs !== 6 ||
    desktop.width > desktop.viewport + 1 ||
    mobile.width > mobile.viewport + 1 ||
    phaseBefore === phaseAfter ||
    distanceAfter <= distanceBefore + 0.002 ||
    Object.values(gaitChecks).some((passed) => !passed) ||
    Object.values(commandChecks).some((passed) => !passed) ||
    !runningLabel?.includes("PAUSE") ||
    !jointMode ||
    !ikMode ||
    !bodyMode ||
    pageErrors.length ||
    httpErrors.length
  ) {
    process.exitCode = 1;
  }
} finally {
  await browser.close();
}

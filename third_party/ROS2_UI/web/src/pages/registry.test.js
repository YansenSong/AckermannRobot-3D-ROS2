import { afterEach, expect, test, vi } from "vitest";

afterEach(() => {
  vi.unstubAllEnvs();
  vi.resetModules();
});

test("inspection profile exposes configured missions and omits unconfigured controls", async () => {
  vi.stubEnv("REACT_APP_UI_PROFILE", "inspection_demo");
  vi.resetModules();
  const { PAGE_REGISTRY, NAV_REGISTRY } = await import("./registry");
  expect(PAGE_REGISTRY.map(({ path }) => path)).toEqual([
    "/", "/route", "/maps", "/missions", "/info", "/health", "/events", "/config",
  ]);
  expect(NAV_REGISTRY.find(({ path }) => path === "/missions").label).toBe("Missions");
});

test("legacy profile omits removed optional pages and keeps device management", async () => {
  vi.stubEnv("REACT_APP_UI_PROFILE", "");
  vi.resetModules();
  const { PAGE_REGISTRY } = await import("./registry");
  expect(PAGE_REGISTRY.some(({ path }) => path === "/blocks")).toBe(false);
  expect(PAGE_REGISTRY.some(({ path }) => path === "/robot")).toBe(false);
  expect(PAGE_REGISTRY.some(({ path }) => path === "/devices")).toBe(true);
});

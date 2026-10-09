import { afterEach, describe, expect, it, vi } from "vitest";
import { startOneOffMission } from "./taskApi";

const response = (payload, status = 200) =>
  new Response(JSON.stringify(payload), { status });

describe("startOneOffMission", () => {
  afterEach(() => vi.unstubAllGlobals());

  it("saves a map-bound mission before starting it through the task API", async () => {
    const calls = [];
    vi.stubGlobal(
      "fetch",
      vi.fn(async (url, options = {}) => {
        if (String(url).endsWith("/auth/csrf"))
          return response({ csrf_token: "csrf" });
        calls.push({ url: String(url), options });
        if (String(url).endsWith("/missions"))
          return response({ command_id: "save-1", status: "accepted" }, 202);
        return response(
          { command_id: "start-1", task_id: "task-1", status: "accepted" },
          202,
        );
      }),
    );

    const result = await startOneOffMission({
      robotId: "robot-001",
      mapId: "grid-a",
      mapVersionId: "grid-a",
      name: "Manual route",
      steps: [{ type: "waypoint", pose: { x: 1, y: 2, z: 0, w: 1 } }],
    });

    expect(result.task_id).toBe("task-1");
    expect(calls).toHaveLength(2);
    expect(calls[0].url).toContain("/missions");
    expect(calls[1].url).toContain("/tasks");
    const savedMission = JSON.parse(calls[0].options.body);
    expect(savedMission.map_version_id).toBe("grid-a");
    expect(savedMission.steps).toHaveLength(1);
    expect(
      calls.every(({ options }) =>
        new Headers(options.headers).get("Idempotency-Key"),
      ),
    ).toBe(true);
  });
});

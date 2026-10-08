import React, { useEffect, useState } from "react";

import useSavedWaypoints from "../shared/hooks/useSavedWaypoints";
import useMissionRun from "../shared/hooks/useMissionRun";
import {
  getMissions,
  subscribeMissions,
  addMission,
  removeMission,
  addStep,
  removeStep,
  moveStep,
  importMission,
  loadLegacyMissions,
  isMissionOnline,
} from "../shared/missions/missions";
import {
  getHistory,
  requestStart,
  requestStop,
  requestPause,
  requestResume,
  requestRetry,
  requestSkip,
  requestReleaseHold,
} from "../shared/missions/missionClient";
import {
  DashboardCard,
  EmptyState,
  SectionHeader,
} from "../shared/ui/Dashboard";
import { useT, T } from "../shared/i18n/i18n";

const inputClass =
  "rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

const STEP_TYPES = [
  { value: "waypoint", label: "Go to waypoint" },
  { value: "home", label: "Go home" },
  { value: "wait", label: "Wait" },
  { value: "dock", label: "Dock" },
  { value: "undock", label: "Undock" },
];

const STATUS_STYLE = {
  running: "text-statusBlue",
  paused: "text-statusYellow",
  succeeded: "text-statusGreen",
  failed: "text-statusRed",
  cancelled: "text-statusYellow",
};

const describeStep = (step, waypoints) => {
  switch (step.type) {
    case "waypoint": {
      const wp = waypoints.find((w) => w.id === step.waypointId);
      return `Go to "${step.waypointName || wp?.name || "(missing waypoint)"}"`;
    }
    case "home":
      return "Go home";
    case "wait":
      return `Wait ${step.seconds}s`;
    case "dock":
      return "Dock";
    case "undock":
      return "Undock";
    default:
      return step.type;
  }
};

/** Author and control missions persisted and executed on the robot. */
const MissionsPage = () => {
  const { t } = useT();
  const { waypoints } = useSavedWaypoints();
  const run = useMissionRun();
  const [missions, setMissions] = useState(getMissions);
  const [online, setOnline] = useState(isMissionOnline);
  const [selectedId, setSelectedId] = useState(null);
  const [selectedTaskId, setSelectedTaskId] = useState(null);
  const [newName, setNewName] = useState("");
  const [stepForm, setStepForm] = useState({
    type: "waypoint",
    waypointId: "",
    seconds: "5",
  });

  useEffect(
    () =>
      subscribeMissions((next) => {
        setMissions(next);
        setOnline(isMissionOnline());
      }),
    [],
  );

  const selected = missions.find((m) => m.id === selectedId) || null;

  const createMission = () => {
    const trimmed = newName.trim();
    if (!trimmed) return;
    const mission = addMission(trimmed);
    if (!mission) return;
    setNewName("");
    setSelectedId(mission.id);
  };

  const addStepToSelected = () => {
    if (!selected) return;
    if (stepForm.type === "waypoint") {
      if (!stepForm.waypointId) return;
      const waypoint = waypoints.find(
        (w) => String(w.id) === stepForm.waypointId,
      );
      if (!waypoint) return;
      addStep(selected.id, {
        type: "waypoint",
        waypointId: waypoint.id,
        waypointName: waypoint.name,
        pose: {
          x: waypoint.x,
          y: waypoint.y,
          z: waypoint.z ?? 0,
          w: waypoint.w ?? 1,
        },
      });
    } else if (stepForm.type === "wait") {
      const seconds = Math.max(1, parseInt(stepForm.seconds, 10) || 5);
      addStep(selected.id, { type: "wait", seconds });
    } else {
      addStep(selected.id, { type: stepForm.type });
    }
  };

  const importLegacy = () => {
    const drafts = loadLegacyMissions();
    let imported = 0;
    for (const draft of drafts) {
      if (missions.some((m) => m.id === String(draft.id))) continue;
      const steps = draft.steps?.map((step) => {
        if (step.type !== "waypoint" || step.pose) return step;
        const waypoint = waypoints.find((w) => w.id === step.waypointId);
        if (!waypoint) return null;
        return {
          ...step,
          waypointName: waypoint.name,
          pose: {
            x: waypoint.x,
            y: waypoint.y,
            z: waypoint.z ?? 0,
            w: waypoint.w ?? 1,
          },
        };
      });
      if (steps?.includes(null)) continue;
      if (importMission({ ...draft, steps: steps || [] })) imported += 1;
    }
    window.alert(
      `${imported} mission drafts imported. Drafts with missing waypoints were skipped.`,
    );
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:py-6">
      <SectionHeader
        eyebrow={t("Autonomy")}
        title={t("Missions")}
        description={t(
          "Mission definitions and execution records live on the robot. Running tasks continue when this browser closes.",
        )}
      />
      <div className="flex items-center gap-3 text-xs text-themeTextGray">
        <span>
          {online ? t("Mission manager online") : t("Mission manager offline")}
        </span>
        {online &&
          loadLegacyMissions().some(
            (draft) =>
              !missions.some((mission) => mission.id === String(draft.id)),
          ) && (
            <button
              onClick={importLegacy}
              className="text-themeBlue hover:underline"
            >
              {t("Import browser mission drafts")}
            </button>
          )}
      </div>
      {run?.hold_active &&
        ["succeeded", "failed", "cancelled"].includes(run.status) && (
          <div className="rounded-lg border border-statusYellow p-3 text-xs text-statusYellow">
            {t("Mission drive hold is active after the task ended.")}{" "}
            <button
              disabled={!online}
              onClick={() => {
                if (
                  window.confirm(
                    t(
                      "Release mission drive hold? Confirm the vehicle is safe to move.",
                    ),
                  )
                ) {
                  requestReleaseHold();
                }
              }}
              className="underline disabled:opacity-40"
            >
              {t("Release drive hold")}
            </button>
          </div>
        )}

      <DashboardCard className="p-0">
        {missions.length === 0 ? (
          <EmptyState
            title={t("No missions yet")}
            description={t(
              "Create one below, then add steps — e.g. go to the loading bay, wait 10s, then dock.",
            )}
          />
        ) : (
          <ul className="divide-y divide-borderSubtle/30 font-[RobotoMono]">
            {missions.map((m) => {
              const isSelected = selectedId === m.id;
              const isRunning =
                run?.missionId === m.id && run.status === "running";
              const activeRun = run?.missionId === m.id ? run : null;
              return (
                <li key={m.id} className="px-4 py-3">
                  <div className="flex flex-wrap items-center gap-3">
                    <button
                      onClick={() => setSelectedId(isSelected ? null : m.id)}
                      className="min-w-0 flex-1 text-left"
                    >
                      <p className="text-sm font-semibold text-textWhiteHover">
                        {m.name}
                        {isSelected && (
                          <span className="ml-2 text-themeBlue">▾</span>
                        )}
                      </p>
                      <p className="text-xs text-themeTextGray">
                        {m.steps.length} {t("steps")}
                        {activeRun && (
                          <span className="ml-2">
                            task_id: {activeRun.taskId}
                          </span>
                        )}
                        {activeRun && (
                          <span
                            className={`ml-2 ${
                              STATUS_STYLE[activeRun.status] || ""
                            }`}
                          >
                            · {t(activeRun.status)}
                            {activeRun.status === "running" &&
                              ` (step ${activeRun.stepIndex + 1}/${
                                activeRun.steps.length
                              })`}
                          </span>
                        )}
                      </p>
                      {activeRun && (
                        <p className="text-[11px] text-themeTextGray">
                          {activeRun.reason} ·{" "}
                          {Math.max(
                            0,
                            activeRun.steps.length - activeRun.stepIndex - 1,
                          )}{" "}
                          {t("remaining steps")}
                        </p>
                      )}
                      {activeRun && activeRun.steps[activeRun.stepIndex] && (
                        <p className="text-[11px] text-themeTextGray">
                          {t("Current step")}: #{activeRun.stepIndex + 1}{" "}
                          {t(
                            describeStep(
                              activeRun.steps[activeRun.stepIndex],
                              waypoints,
                            ),
                          )}
                        </p>
                      )}
                    </button>
                    {isRunning && (
                      <>
                        <button
                          onClick={requestPause}
                          disabled={!online}
                          className="rounded-lg border border-statusYellow px-3 py-1 text-xs text-statusYellow disabled:opacity-40"
                        >
                          {t("Pause")}
                        </button>
                        <button
                          onClick={requestSkip}
                          disabled={!online}
                          className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue disabled:opacity-40"
                        >
                          {t("Skip")}
                        </button>
                        <button
                          onClick={requestStop}
                          disabled={!online}
                          className="rounded-lg border border-statusRed px-3 py-1 text-xs text-statusRed disabled:opacity-40"
                        >
                          <T>{"Stop"}</T>
                        </button>
                      </>
                    )}
                    {activeRun?.status === "paused" && (
                      <>
                        <button
                          onClick={requestResume}
                          disabled={!online}
                          className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue disabled:opacity-40"
                        >
                          {t("Resume")}
                        </button>
                        <button
                          onClick={requestSkip}
                          disabled={!online}
                          className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue disabled:opacity-40"
                        >
                          {t("Skip")}
                        </button>
                        <button
                          onClick={requestStop}
                          disabled={!online}
                          className="rounded-lg border border-statusRed px-3 py-1 text-xs text-statusRed disabled:opacity-40"
                        >
                          <T>{"Stop"}</T>
                        </button>
                      </>
                    )}
                    {activeRun?.status === "failed" && (
                      <>
                        <button
                          onClick={requestRetry}
                          disabled={!online}
                          className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue disabled:opacity-40"
                        >
                          {t("Retry")}
                        </button>
                        <button
                          onClick={requestSkip}
                          disabled={!online}
                          className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue disabled:opacity-40"
                        >
                          {t("Skip")}
                        </button>
                      </>
                    )}
                    {!isRunning && activeRun?.status !== "paused" && (
                      <button
                        onClick={() => {
                          if (
                            !window.confirm(
                              t(
                                "This will start moving the robot through this mission's steps. Continue?",
                              ),
                            )
                          )
                            return;
                          requestStart(m.id);
                        }}
                        disabled={
                          !online ||
                          !m.steps.length ||
                          run?.status === "running" ||
                          run?.status === "paused"
                        }
                        className="rounded-lg border border-themeBlue px-3 py-1 text-xs text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:opacity-40"
                      >
                        {t("Run (moves robot)")}
                      </button>
                    )}
                    <button
                      onClick={() => {
                        removeMission(m.id);
                        if (selectedId === m.id) setSelectedId(null);
                      }}
                      aria-label={`Delete ${m.name}`}
                      disabled={
                        !online ||
                        (activeRun &&
                          ["running", "paused"].includes(activeRun.status))
                      }
                      className="px-1 text-themeTextGray hover:text-statusRed"
                    >
                      ×
                    </button>
                  </div>

                  {isSelected && (
                    <div className="mt-3 space-y-2 rounded-lg bg-bgSurface p-3">
                      {activeRun?.events?.length > 0 && (
                        <div className="max-h-36 overflow-auto rounded border border-borderSubtle p-2 text-[11px] text-themeTextGray">
                          {activeRun.events.map((event, index) => (
                            <p key={index}>
                              {event.timestamp} · #{event.step_index + 1} ·{" "}
                              {event.status} · {event.reason}
                            </p>
                          ))}
                        </div>
                      )}
                      {m.steps.length === 0 ? (
                        <p className="text-xs text-themeTextGray opacity-70">
                          {t("No steps yet — add one below.")}
                        </p>
                      ) : (
                        <ol className="space-y-1.5">
                          {m.steps.map((step, index) => (
                            <li
                              key={step.id}
                              className="flex items-center gap-2 rounded-lg border border-borderSubtle bg-bgCard px-2.5 py-1.5 text-xs"
                            >
                              <span className="w-5 shrink-0 text-themeTextGray">
                                {index + 1}.
                              </span>
                              <span className="flex-1 text-textWhiteHover">
                                {t(describeStep(step, waypoints))}
                              </span>
                              {activeRun?.events?.some(
                                (event) =>
                                  event.step_index === index &&
                                  event.reason.includes("completed"),
                              ) && <span className="text-statusGreen">✓</span>}
                              <button
                                onClick={() => moveStep(m.id, step.id, -1)}
                                disabled={!online || index === 0}
                                className="text-themeTextGray hover:text-themeBlue disabled:opacity-30"
                                aria-label="Move up"
                              >
                                ↑
                              </button>
                              <button
                                onClick={() => moveStep(m.id, step.id, 1)}
                                disabled={
                                  !online || index === m.steps.length - 1
                                }
                                className="text-themeTextGray hover:text-themeBlue disabled:opacity-30"
                                aria-label="Move down"
                              >
                                ↓
                              </button>
                              <button
                                onClick={() => removeStep(m.id, step.id)}
                                disabled={!online}
                                className="text-themeTextGray hover:text-statusRed"
                                aria-label="Remove step"
                              >
                                ×
                              </button>
                            </li>
                          ))}
                        </ol>
                      )}

                      <div className="flex flex-wrap items-center gap-2 border-t border-borderSubtle pt-2">
                        <select
                          className={inputClass}
                          value={stepForm.type}
                          onChange={(e) =>
                            setStepForm((p) => ({ ...p, type: e.target.value }))
                          }
                        >
                          {STEP_TYPES.map((stepType) => (
                            <option key={stepType.value} value={stepType.value}>
                              {t(stepType.label)}
                            </option>
                          ))}
                        </select>
                        {stepForm.type === "waypoint" && (
                          <select
                            className={inputClass}
                            value={stepForm.waypointId}
                            onChange={(e) =>
                              setStepForm((p) => ({
                                ...p,
                                waypointId: e.target.value,
                              }))
                            }
                          >
                            <option value="">{t("Choose a waypoint…")}</option>
                            {waypoints.map((w) => (
                              <option key={w.id} value={w.id}>
                                {w.name}
                              </option>
                            ))}
                          </select>
                        )}
                        {stepForm.type === "wait" && (
                          <input
                            type="number"
                            min="1"
                            className={`${inputClass} w-24`}
                            value={stepForm.seconds}
                            onChange={(e) =>
                              setStepForm((p) => ({
                                ...p,
                                seconds: e.target.value,
                              }))
                            }
                            placeholder="seconds"
                          />
                        )}
                        <button
                          onClick={addStepToSelected}
                          disabled={!online}
                          className="rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeTextGray transition-colors hover:border-themeBlue hover:text-themeBlue"
                        >
                          {t("+ Add step")}
                        </button>
                      </div>
                      {stepForm.type === "waypoint" &&
                        waypoints.length === 0 && (
                          <p className="text-[11px] text-themeTextGray/70">
                            {t(
                              "No saved waypoints yet — add one from the Map page first.",
                            )}
                          </p>
                        )}
                    </div>
                  )}
                </li>
              );
            })}
          </ul>
        )}
      </DashboardCard>

      {getHistory().length > 0 && (
        <DashboardCard className="p-4 font-[RobotoMono]">
          <p className="mb-2 text-xs font-bold text-themeBlue">
            {t("Recent task executions")}
          </p>
          <div className="max-h-44 space-y-1 overflow-auto text-[11px] text-themeTextGray">
            {getHistory().map((task) => (
              <div key={task.task_id}>
                <button
                  onClick={() =>
                    setSelectedTaskId(
                      selectedTaskId === task.task_id ? null : task.task_id,
                    )
                  }
                  className="text-left hover:text-themeBlue"
                >
                  {task.started_at} · {task.mission_name} ·{" "}
                  {t(task.status.toLowerCase())} · task_id: {task.task_id}
                </button>
                {selectedTaskId === task.task_id && (
                  <div className="ml-3 border-l border-borderSubtle pl-2">
                    {task.events?.map((event, index) => (
                      <p key={index}>
                        {event.timestamp} · #{event.step_index + 1} ·{" "}
                        {event.status} · {event.reason}
                      </p>
                    ))}
                  </div>
                )}
              </div>
            ))}
          </div>
        </DashboardCard>
      )}

      <DashboardCard className="p-4 font-[RobotoMono]">
        <p className="mb-3 text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          {t("New mission")}
        </p>
        <div className="flex gap-2">
          <input
            className={`${inputClass} flex-1`}
            value={newName}
            onChange={(e) => setNewName(e.target.value)}
            onKeyDown={(e) => e.key === "Enter" && createMission()}
            placeholder={t("e.g. Evening patrol")}
          />
          <button
            onClick={createMission}
            disabled={!online}
            className="shrink-0 rounded-lg border border-themeBlue bg-themeBlue/10 px-4 py-2 text-sm font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white"
          >
            {t("Create")}
          </button>
        </div>
      </DashboardCard>
    </div>
  );
};

export default MissionsPage;

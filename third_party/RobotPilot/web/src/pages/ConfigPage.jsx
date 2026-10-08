import React, { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRuntimeConfig } from "../app/App";
import {
  DEFAULT_RUNTIME_CONFIG,
  resolveRosbridgeHost,
} from "../shared/constants/runtimeConfig";
import { SectionHeader, DashboardCard } from "../shared/ui/Dashboard";
import Button from "../shared/ui/Button";
import Switcher from "../shared/ui/Switcher";
import SystemHealth from "../components/SystemHealth";
import LifecycleStatus from "../components/LifecycleStatus";
import useSystemDiagnostics, {
  OVERALL_LABELS,
} from "../shared/hooks/useSystemDiagnostics";
import { useT } from "../shared/i18n/i18n";

const OVERALL_DOT = {
  0: "bg-statusGreen",
  1: "bg-statusYellow",
  2: "bg-statusRed",
  3: "bg-statusRed",
};

const OVERALL_TEXT = {
  0: "text-statusGreen",
  1: "text-statusYellow",
  2: "text-statusRed",
  3: "text-statusRed",
};

const Field = ({ label, hint, children }) => {
  const { t } = useT();
  return (
    <label className="block">
      <span className="text-xs font-semibold uppercase tracking-wider text-themeTextGray">
        {t(label)}
      </span>
      <div className="mt-1.5">{children}</div>
      {hint ? (
        <p className="mt-1 text-[11px] text-themeTextGray/70">{t(hint)}</p>
      ) : null}
    </label>
  );
};

const inputClass =
  "w-full rounded-lg border border-borderSubtle bg-bgCard px-3 py-2 text-sm text-textWhiteHover outline-none focus:border-themeBlue";

const ConfigPage = () => {
  const { t } = useT();
  const { config, updateConfig } = useRuntimeConfig();
  const [form, setForm] = useState(config);
  const [notifPermission, setNotifPermission] = useState(
    typeof Notification !== "undefined"
      ? Notification.permission
      : "unsupported",
  );
  const { reportHealth, reportLifecycle, issues, overall, overallLabel } =
    useSystemDiagnostics();

  // 如果设置在其他位置发生变化（例如另一标签页重置了相同 localStorage 键），同步更新表单。
  useEffect(() => setForm(config), [config]);

  const requestNotificationPermission = async () => {
    if (typeof Notification === "undefined") return;
    const result = await Notification.requestPermission();
    setNotifPermission(result);
    if (result === "granted") toast.success(t("Browser notifications enabled"));
    else toast.warn(t("Notification permission was not granted"));
  };

  const resolvedHost = resolveRosbridgeHost(form);

  const setField = (key) => (e) => {
    const { value } = e.target;
    setForm((prev) => ({ ...prev, [key]: value }));
  };

  const handleSave = () => {
    updateConfig({
      rosbridgeHost: form.rosbridgeHost.trim(),
      rosbridgePort: String(
        form.rosbridgePort || DEFAULT_RUNTIME_CONFIG.rosbridgePort,
      ),
      cameraPort: String(form.cameraPort || DEFAULT_RUNTIME_CONFIG.cameraPort),
      maxLinearSpeed:
        parseFloat(form.maxLinearSpeed) ||
        DEFAULT_RUNTIME_CONFIG.maxLinearSpeed,
      maxAngularSpeed:
        parseFloat(form.maxAngularSpeed) ||
        DEFAULT_RUNTIME_CONFIG.maxAngularSpeed,
      notificationsEnabled: Boolean(form.notificationsEnabled),
      lowBatteryThreshold:
        parseFloat(form.lowBatteryThreshold) ||
        DEFAULT_RUNTIME_CONFIG.lowBatteryThreshold,
    });
    toast.success(t("Settings saved"));
  };

  const handleReset = () => {
    if (
      !window.confirm(
        t(
          "Reset connection address/port, camera port, speed limits, and the low-battery alert back to defaults? If you're currently connected to a robot at a custom address, this will disconnect you.",
        ),
      )
    )
      return;
    setForm(DEFAULT_RUNTIME_CONFIG);
    updateConfig(DEFAULT_RUNTIME_CONFIG);
    toast.info(t("Settings reset to defaults"));
  };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
      <SectionHeader
        eyebrow="Settings"
        title="Configuration"
        description="Connection and safety defaults for this browser, saved locally — nothing here is shared with other operators or persisted on the robot."
      />

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          {t("Connection")}
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          {t("Currently resolving to")}{" "}
          <code className="rounded bg-bgSurface px-1.5 py-0.5 text-textWhiteHover">
            ws://{resolvedHost}:{form.rosbridgePort}
          </code>
          {t(". Changing these fields reconnects to the robot.")}
        </p>

        <div className="mt-4 grid gap-4 sm:grid-cols-2">
          <Field
            label="Robot address override"
            hint="This is the network address the app uses to talk to the robot. Leave blank to auto-use this page’s own host (the normal case once deployed on the robot)."
          >
            <input
              type="text"
              value={form.rosbridgeHost}
              onChange={setField("rosbridgeHost")}
              placeholder={`${t("Auto")} — ${resolveRosbridgeHost({
                rosbridgeHost: "",
              })}`}
              className={inputClass}
            />
          </Field>

          <Field label="Robot connection port">
            <input
              type="number"
              min="1"
              max="65535"
              value={form.rosbridgePort}
              onChange={setField("rosbridgePort")}
              className={inputClass}
            />
          </Field>

          <Field
            label="Camera stream port"
            hint="Port used to stream the camera feed."
          >
            <input
              type="number"
              min="1"
              max="65535"
              value={form.cameraPort}
              onChange={setField("cameraPort")}
              className={inputClass}
            />
          </Field>
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
            {t("Connection diagnostics")}
          </p>
          <div className="flex items-center gap-1.5">
            <span className={`h-2 w-2 rounded-full ${OVERALL_DOT[overall]}`} />
            <span className={`text-xs font-semibold ${OVERALL_TEXT[overall]}`}>
              {t(overallLabel || OVERALL_LABELS[0])}
            </span>
          </div>
        </div>
        <p className="mt-1 text-sm text-themeTextGray">
          {t(
            "The same health summary as the Health Centre; connection problems appear here too.",
          )}
        </p>

        {issues.length > 0 && (
          <div className="mt-3 flex flex-wrap gap-1.5">
            {issues.map((issue) =>
              issue.linkTo ? (
                <Link
                  key={issue.id}
                  to={issue.linkTo}
                  className="rounded-lg border border-borderSubtle bg-bgSurface px-2.5 py-1.5 text-xs text-textWhiteHover hover:border-themeBlue hover:text-themeBlue"
                >
                  {t(issue.message)}
                </Link>
              ) : (
                <span
                  key={issue.id}
                  className="rounded-lg border border-borderSubtle bg-bgSurface px-2.5 py-1.5 text-xs text-themeTextGray"
                >
                  {t(issue.message)}
                </span>
              ),
            )}
          </div>
        )}

        <div className="mt-4 grid gap-3 sm:grid-cols-2">
          <div className="dashboard-card p-3">
            <SystemHealth compact onHealthChange={reportHealth} />
          </div>
          <div className="dashboard-card p-3">
            <LifecycleStatus compact onStatesChange={reportLifecycle} />
          </div>
        </div>

        <Link
          to="/health"
          className="mt-3 inline-block text-xs text-themeBlue hover:underline"
        >
          {t("Open full Health Centre →")}
        </Link>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          {t("Manual-drive safety limits")}
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          {t(
            "Maximum joystick and map speed settings. Changes apply to new manual commands.",
          )}
        </p>

        <div className="mt-4 grid gap-4 sm:grid-cols-2">
          <Field label="Max linear speed" hint="m/s">
            <input
              type="number"
              min="0.05"
              max="2"
              step="0.05"
              value={form.maxLinearSpeed}
              onChange={setField("maxLinearSpeed")}
              className={inputClass}
            />
          </Field>

          <Field label="Max angular speed" hint="rad/s">
            <input
              type="number"
              min="0.1"
              max="6"
              step="0.1"
              value={form.maxAngularSpeed}
              onChange={setField("maxAngularSpeed")}
              className={inputClass}
            />
          </Field>
        </div>
      </DashboardCard>

      <DashboardCard className="p-4">
        <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
          {t("Notifications")}
        </p>
        <p className="mt-1 text-sm text-themeTextGray">
          {t(
            "Browser notifications for navigation, docking, and low battery require browser permission.",
          )}
        </p>

        <div className="mt-4 flex flex-wrap items-center gap-3">
          <Switcher
            switcherValue={Boolean(form.notificationsEnabled)}
            onChange={(next) =>
              setForm((prev) => ({ ...prev, notificationsEnabled: next }))
            }
          />
          <span className="text-sm text-textWhiteHover">
            {t(form.notificationsEnabled ? "Enabled" : "Disabled")}
          </span>

          <button
            type="button"
            onClick={requestNotificationPermission}
            className="ml-auto rounded-lg border border-borderSubtle px-3 py-1.5 text-xs text-themeBlue hover:border-themeBlue"
          >
            {t(
              notifPermission === "granted"
                ? "Permission granted"
                : notifPermission === "denied"
                ? "Permission blocked — check browser settings"
                : "Request permission",
            )}
          </button>
        </div>

        <div className="mt-4 max-w-xs">
          <Field
            label="Low battery threshold"
            hint="Notify once battery drops to or below this level, %"
          >
            <input
              type="number"
              min="0"
              max="100"
              step="1"
              value={form.lowBatteryThreshold}
              onChange={setField("lowBatteryThreshold")}
              className={inputClass}
            />
          </Field>
        </div>
      </DashboardCard>

      <div className="grid grid-cols-2 gap-3 sm:max-w-sm">
        <Button type="orange" onBtnClick={handleSave}>
          {t("Save settings")}
        </Button>
        <Button onBtnClick={handleReset}>{t("Reset to defaults")}</Button>
      </div>

      <ToastContainer theme="dark" position="bottom-right" />
    </div>
  );
};

export default ConfigPage;

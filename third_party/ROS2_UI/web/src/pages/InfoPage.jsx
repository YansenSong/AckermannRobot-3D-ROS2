import React, { useRef, useState, useEffect } from "react";

import Camera from "../components/Camera";
import RobotState from "../components/RobotState";
import SystemHealth from "../components/SystemHealth";
import { useRos } from "../app/App";
import { AppConfig } from "../shared/constants/index";
import { useT } from "../shared/i18n/i18n";
import { INSPECTION_PROFILE } from "../shared/robot/robotContract";
import {
  ChartCard,
  DashboardCard,
  EmptyState,
  SectionHeader,
  StatusBadge,
} from "../shared/ui/Dashboard";

const BATTERY_HISTORY_LENGTH = 40;

// 简易内联迷你趋势图：仓库没有图表库，且只有此处需要趋势线，因此使用小型本地 SVG 即可。
const Sparkline = ({ values, className = "" }) => {
  if (values.length < 2) return null;
  const width = 100;
  const height = 28;
  const points = values
    .map((v, i) => {
      const x = (i / (values.length - 1)) * width;
      const y = height - (Math.max(0, Math.min(100, v)) / 100) * height;
      return `${x},${y}`;
    })
    .join(" ");

  return (
    <svg
      viewBox={`0 0 ${width} ${height}`}
      preserveAspectRatio="none"
      className={className}
      aria-hidden="true"
    >
      <polyline
        points={points}
        fill="none"
        stroke="currentColor"
        strokeWidth="1.5"
        vectorEffect="non-scaling-stroke"
      />
    </svg>
  );
};

const InfoPage = () => {
  const { t } = useT();
  const ros = useRos();
  const [batteryPct, setBatteryPct] = useState(null);
  const [batteryHistory, setBatteryHistory] = useState([]);
  const [charging, setCharging] = useState(false);

  const batteryTopic = useRef(null);
  const chargeTopic = useRef(null);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    batteryTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });

    chargeTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CHARGE_STATION_CONNECTED,
      messageType: "std_msgs/Bool",
    });

    // battery.py 将电量百分比作为普通浮点数（0–100）发布。
    batteryTopic.current.subscribe(({ data }) => {
      setBatteryPct(Math.round(data));
      setBatteryHistory((prev) => [...prev.slice(-(BATTERY_HISTORY_LENGTH - 1)), data]);
    });

    chargeTopic.current.subscribe(({ data }) => {
      setCharging(data);
    });

    return () => {
      batteryTopic.current?.unsubscribe();
      chargeTopic.current?.unsubscribe();
    };
  }, [ros]);

  const batteryStatus =
    batteryPct === null
      ? { status: "unknown", label: "No data" }
      : batteryPct > 60
      ? { status: "success", label: "Good" }
      : batteryPct > 25
      ? { status: "warning", label: "Low" }
      : { status: "error", label: "Critical" };

  return (
    <div className="sectionHeight space-y-5 py-4 sm:space-y-6 sm:py-6">
      <SectionHeader
        eyebrow="System overview"
        title="Robot status"
        description="Live camera, telemetry, power, and ROS health in one operational view."
      />

      <div className="grid min-w-0 gap-4 xl:grid-cols-[minmax(0,1.45fr)_minmax(360px,0.8fr)] xl:gap-5">
        <section className="grid min-w-0 gap-4">
          <div className="h-[320px] min-w-0 sm:h-[420px] xl:h-[500px]">
            <Camera />
          </div>
          {INSPECTION_PROFILE ? <p className="dashboard-card p-4 text-sm text-statusYellow">{t("Project localization interface is unconfigured. Status: UNKNOWN.")}</p> : <RobotState />}
        </section>

        <section className="grid min-w-0 content-start gap-4">
          <ChartCard
            title="Battery level"
            value={batteryPct !== null ? `${batteryPct}%` : "—"}
            status={
              <StatusBadge
                status={batteryStatus.status}
                label={batteryStatus.label}
              />
            }
          >
            {batteryPct !== null ? (
              <div className="mt-4 space-y-2">
                <div
                  className="premium-progress"
                  role="progressbar"
                    aria-label={t("Battery charge")}
                  aria-valuemin="0"
                  aria-valuemax="100"
                  aria-valuenow={batteryPct}
                >
                  <div
                    className="premium-progress__value"
                    style={{
                      width: `${Math.max(0, Math.min(100, batteryPct))}%`,
                    }}
                  />
                </div>
                <div className="flex justify-between font-[RobotoMono] text-[10px] text-themeTextGray">
                  <span>0%</span>
                  <span>100%</span>
                </div>

                {batteryHistory.length > 1 && (
                  <div>
                    <p className="mb-1 text-[10px] uppercase tracking-wider text-themeTextGray">
                      {t("Recent trend")}{" "}
                      <span className="normal-case tracking-normal text-themeTextGray/60">
                        {t("(last ~10 min)")}
                      </span>
                    </p>
                    <Sparkline
                      values={batteryHistory}
                      className={`h-7 w-full ${
                        batteryStatus.status === "success"
                          ? "text-statusGreen"
                          : batteryStatus.status === "warning"
                          ? "text-statusYellow"
                          : "text-statusRed"
                      }`}
                    />
                  </div>
                )}
              </div>
            ) : (
              <EmptyState
                className="min-h-[92px] px-0 pb-0"
                title="No battery telemetry"
                description="Waiting for the configured battery topic to publish."
              />
            )}
          </ChartCard>

          <DashboardCard className="flex items-center justify-between gap-4 p-4">
            <div>
              <p className="font-[RobotoMono] text-[11px] font-bold uppercase tracking-[0.14em] text-themeBlue">
                {t("Charging station")}
              </p>
              <p className="mt-1 text-sm text-themeTextGray">
                {t(charging
                  ? "Robot is connected to external power."
                  : "Robot is not connected to the dock.")}
              </p>
            </div>
            <StatusBadge
              status={charging ? "connected" : "idle"}
              label={charging ? "Connected" : "Not connected"}
              pulse={charging}
            />
          </DashboardCard>

          <div className="min-w-0">
            <SystemHealth />
          </div>
        </section>
      </div>
    </div>
  );
};

export default InfoPage;

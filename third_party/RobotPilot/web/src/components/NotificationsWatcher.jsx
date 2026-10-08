import { useEffect, useRef } from "react";

import { useRos, useRuntimeConfig } from "../app/App";
import { AppConfig } from "../shared/constants";

const NAV_TERMINAL_LABELS = {
  4: "Navigation succeeded",
  5: "Navigation canceled",
  6: "Navigation failed",
};

const notify = (title, body) => {
  if (typeof Notification === "undefined" || Notification.permission !== "granted") return;
  try {
    // 即发即弃；无需保留此实例的引用。
    const notification = new Notification(title, { body });
    return notification;
  } catch {
    // 部分平台（例如 Android Chrome）只允许通过 service worker 显示通知，直接构造时会抛出异常。
    // 这不是致命错误，跳过通知即可。
  }
};

/**
 * 无界面、始终挂载于 AppLayout 的监听器，在导航目标完成、对接结束/失败或电量低于阈值等关键事件发生时，
 * 通过浏览器 Notification API 提醒操作员，使其无需一直将此标签页停留在特定页面。由 Config 页的
 * config.notificationsEnabled 控制，默认关闭。
 */
const NotificationsWatcher = () => {
  const ros = useRos();
  const { config } = useRuntimeConfig();
  const lastNavTerminalRef = useRef(null);
  const lowBatteryNotifiedRef = useRef(false);

  useEffect(() => {
    if (!config.notificationsEnabled || !ros || !window.ROSLIB) return;

    const navStatusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAV_STATUS_TOPIC,
      messageType: "action_msgs/GoalStatusArray",
    });
    navStatusTopic.subscribe((msg) => {
      if (!msg.status_list?.length) return;
      const latest = msg.status_list[msg.status_list.length - 1];
      if (![4, 5, 6].includes(latest.status)) return;
      const key = latest.goal_info?.goal_id?.uuid?.join?.("-") || latest.status;
      if (lastNavTerminalRef.current === key) return;
      lastNavTerminalRef.current = key;
      notify("RobotPilot", NAV_TERMINAL_LABELS[latest.status]);
    });

    const dockStatusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.DOCK_TRIGGER_STATUS_TOPIC,
      messageType: "std_msgs/String",
    });
    dockStatusTopic.subscribe((msg) => {
      if (msg.data === "docked") notify("RobotPilot", "Docking complete");
      else if (msg.data === "failed") notify("RobotPilot", "Docking failed — check the dock tag and logs");
    });

    const batteryTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });
    batteryTopic.subscribe(({ data }) => {
      const threshold = config.lowBatteryThreshold;
      if (data <= threshold) {
        if (!lowBatteryNotifiedRef.current) {
          lowBatteryNotifiedRef.current = true;
          notify("RobotPilot", `Battery at ${Math.round(data)}% — below ${threshold}%`);
        }
      } else if (data > threshold + 5) {
        // 使用滞回：电量恢复到高于阈值数个百分点后才重新启用提醒，避免电量在阈值附近波动时每条消息都触发通知。
        lowBatteryNotifiedRef.current = false;
      }
    });

    return () => {
      navStatusTopic.unsubscribe();
      dockStatusTopic.unsubscribe();
      batteryTopic.unsubscribe();
    };
  }, [ros, config.notificationsEnabled, config.lowBatteryThreshold]);

  return null;
};

export default NotificationsWatcher;

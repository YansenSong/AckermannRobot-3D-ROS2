import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useRos, useRosStatus, useRuntimeConfig } from "../../app/App";
import { AppConfig, LIFECYCLE_NODES } from "../constants";
import { FRIENDLY_NAMES as LIFECYCLE_FRIENDLY_NAMES } from "../../components/LifecycleStatus";
import useDevices from "./useDevices";
import useDeviceStatuses from "./useDeviceStatuses";

// Plain-language names for the topic-health keys SystemHealth already
// computes, so an issue pill reads "Localization has gone silent" instead
// of "amcl topic has gone silent".
const TOPIC_FRIENDLY_NAMES = {
  odom: "Odometry",
  scan: "Laser scan",
  map: "Map",
  localization: "Localization",
  nav2: "Navigation",
  costmap: "Global costmap",
  plan: "Path plan",
};

// Topics this app's own pages already depend on being alive — a reasonable,
// honest definition of "expected", since it's exactly what SystemHealth,
// and InfoPage already assume is publishing.
const EXPECTED_TOPICS = [
  { topic: AppConfig.SCAN_TOPIC, label: "Laser scan" },
  { topic: AppConfig.ROBOT_POSE_TOPIC, label: "Odometry" },
  { topic: AppConfig.MAP_TOPIC, label: "Map" },
  { topic: AppConfig.LOCALIZATION_POSE_TOPIC, label: "Localization" },
  { topic: AppConfig.NAV_STATUS_TOPIC, label: "Navigation" },
  { topic: AppConfig.BATTERY_TOPIC, label: "Battery" },
  { topic: AppConfig.JOINT_STATES_TOPIC, label: "Joint states" },
];

const CRITICAL_TOPIC_KEYS = new Set(["odom", "scan", "map", "localization"]);

const DIAGNOSTIC_LEVEL_LABEL = { 1: "WARN", 2: "ERROR", 3: "STALE" };

export const OVERALL_LABELS = {
  0: "Ready",
  1: "Ready with warnings",
  2: "Needs attention",
  3: "Not ready",
};

const RECONNECT_TOPICS_INTERVAL_MS = 20000;
const FAULT_LOG_LIMIT = 30;
const DIAGNOSTIC_STALE_MS = 5000;
const FAULT_STARTUP_GRACE_MS = 10000;
const FAULT_CONFIRM_MS = 3000;

/**
 * Aggregates every health signal this app already computes elsewhere
 * (SystemHealth's topic/TF checks, LifecycleStatus's nav2 states, battery,
 * registered-device status, /diagnostics, and a
 * best-effort rosapi topic-graph check) into one overall Ready / Ready with
 * warnings / Partially connected / Not ready rollup, plus a list of the
 * specific issues driving that rollup so the Health Centre page can link
 * each one to where it can actually be fixed.
 *
 * SystemHealth/LifecycleStatus own their own ROS subscriptions already —
 * this hook doesn't re-subscribe to the same topics/services a second time,
 * it just receives their computed state via the reportHealth/reportLifecycle
 * callbacks the Health Centre page wires up.
 */
export default function useSystemDiagnostics() {
  const ros = useRos();
  const rosbridgeStatus = useRosStatus();
  const { config } = useRuntimeConfig();
  const { devices } = useDevices();
  const deviceStatuses = useDeviceStatuses(ros, devices);

  const [health, setHealth] = useState({});
  const [tfLinks, setTfLinks] = useState({});
  const [lifecycle, setLifecycle] = useState({});
  const [battery, setBattery] = useState({ pct: null, charging: false });
  const [diagnosticsByName, setDiagnosticsByName] = useState({});
  const [missingTopics, setMissingTopics] = useState([]);
  const [faultLog, setFaultLog] = useState([]);

  const reportHealth = useCallback(({ health: h, tfLinks: tl }) => {
    setHealth(h);
    setTfLinks(tl);
  }, []);

  const reportLifecycle = useCallback((states) => {
    setLifecycle(states);
  }, []);

  // Battery + charging — same topics InfoPage already subscribes to.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const batteryTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "std_msgs/Float32",
    });
    const chargeTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CHARGE_STATION_CONNECTED,
      messageType: "std_msgs/Bool",
    });

    batteryTopic.subscribe(({ data }) =>
      setBattery((prev) => ({ ...prev, pct: Math.round(data) })),
    );
    chargeTopic.subscribe(({ data }) =>
      setBattery((prev) => ({ ...prev, charging: data })),
    );

    return () => {
      batteryTopic.unsubscribe();
      chargeTopic.unsubscribe();
    };
  }, [ros]);

  // Standard ROS2 diagnostics aggregator, if anything in the stack publishes
  // to it — best-effort, absence just means "no diagnostic_updater sources",
  // not an error.
  useEffect(() => {
    setDiagnosticsByName({});
    if (!ros || !window.ROSLIB) return;

    const topic = new window.ROSLIB.Topic({
      ros,
      name: "/diagnostics",
      messageType: "diagnostic_msgs/DiagnosticArray",
      queue_length: 1,
    });
    topic.subscribe((msg) => {
      const now = Date.now();
      setDiagnosticsByName((previous) => {
        const next = { ...previous };
        for (const entry of msg?.status || []) {
          if (entry.level >= 1) {
            next[entry.name] = {
              name: entry.name,
              message: entry.message,
              level: entry.level,
              lastSeen: now,
            };
          } else {
            delete next[entry.name];
          }
        }
        return next;
      });
    });

    const expiry = setInterval(() => {
      const now = Date.now();
      setDiagnosticsByName((previous) => {
        if (
          Object.values(previous).every(
            (entry) => now - entry.lastSeen < DIAGNOSTIC_STALE_MS,
          )
        ) {
          return previous;
        }
        return Object.fromEntries(
          Object.entries(previous).filter(
            ([, entry]) => now - entry.lastSeen < DIAGNOSTIC_STALE_MS,
          ),
        );
      });
    }, 1000);

    return () => {
      topic.unsubscribe();
      clearInterval(expiry);
    };
  }, [ros]);

  const diagnosticsMsgs = useMemo(
    () =>
      Object.values(diagnosticsByName).filter(
        (entry) =>
          // The EKF's frequency diagnostic can report zero events while its
          // odometry publisher is demonstrably streaming through rosbridge.
          !(
            health.odom === "online" &&
            entry.name === "ekf_filter_node: odometry/filtered topic status" &&
            entry.message === "No events recorded."
          ),
      ),
    [diagnosticsByName, health.odom],
  );

  // Best-effort rosapi check for whether this app's expected topics are
  // currently in the ROS graph at all. If rosapi itself isn't reachable,
  // this just quietly reports nothing missing rather than treating that as
  // a fault of its own — it's a bonus check, not a required one.
  useEffect(() => {
    if (!ros || rosbridgeStatus !== "connected") {
      setMissingTopics([]);
      return;
    }

    let cancelled = false;
    const check = () => {
      ros.getTopics(
        (result) => {
          if (cancelled) return;
          const present = new Set(result?.topics || []);
          setMissingTopics(
            EXPECTED_TOPICS.filter(({ topic }) => !present.has(topic)),
          );
        },
        () => {
          if (!cancelled) setMissingTopics([]);
        },
      );
    };

    check();
    const id = setInterval(check, RECONNECT_TOPICS_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(id);
    };
  }, [ros, rosbridgeStatus]);

  const issues = useMemo(() => {
    const list = [];

    if (rosbridgeStatus !== "connected") {
      list.push({
        id: "rosbridge",
        severity: 3,
        message:
          "Robot connection is offline — nothing else here can be verified.",
        linkTo: "/config",
      });
    }

    if (
      tfLinks &&
      Object.keys(tfLinks).length &&
      health.tfChain === "offline"
    ) {
      list.push({
        id: "tf-chain",
        severity: 2,
        message:
          "Position tracking is broken — the robot doesn't know where it is. Navigation won't work until this is fixed.",
        linkTo: "/info",
      });
    }

    Object.entries(health).forEach(([key, state]) => {
      if (key === "tfChain" || state !== "offline") return;
      const name = TOPIC_FRIENDLY_NAMES[key] || key;
      list.push({
        id: `topic-${key}`,
        severity: CRITICAL_TOPIC_KEYS.has(key) ? 2 : 1,
        message: `${name} has gone silent.`,
        linkTo: "/info",
      });
    });

    const LIFECYCLE_STATE_LABELS = {
      inactive: "paused",
      unconfigured: "not set up yet",
      unknown: "not responding",
    };
    Object.entries(lifecycle).forEach(([name, state]) => {
      if (state === "active") return;
      const friendlyName = LIFECYCLE_FRIENDLY_NAMES[name] || name;
      const friendlyState = LIFECYCLE_STATE_LABELS[state] || state;
      list.push({
        id: `lifecycle-${name}`,
        severity: state === "inactive" ? 1 : 2,
        message: `${friendlyName} is ${friendlyState}.`,
        linkTo: "/health",
      });
    });

    if (battery.pct !== null && !battery.charging) {
      if (battery.pct <= 10) {
        list.push({
          id: "battery",
          severity: 2,
          message: `Battery critically low (${battery.pct}%).`,
          linkTo: "/info",
        });
      } else if (battery.pct <= (config.lowBatteryThreshold ?? 20)) {
        list.push({
          id: "battery",
          severity: 1,
          message: `Battery low (${battery.pct}%).`,
          linkTo: "/info",
        });
      }
    }

    devices.forEach((device) => {
      if (deviceStatuses[device.id] === "offline") {
        list.push({
          id: `device-${device.id}`,
          severity: 1,
          message: `${device.name} is offline.`,
          linkTo: "/devices",
        });
      }
    });

    diagnosticsMsgs.forEach((entry) => {
      list.push({
        id: `diagnostic-${entry.name}`,
        severity: entry.level >= 2 ? 2 : 1,
        message: `${entry.name}: ${entry.message} (${
          DIAGNOSTIC_LEVEL_LABEL[entry.level] || entry.level
        })`,
        linkTo: null,
      });
    });

    missingTopics.forEach(({ topic, label }) => {
      list.push({
        id: `missing-${topic}`,
        severity: 1,
        message: `${label} isn't sending data right now.`,
        linkTo: "/info",
      });
    });

    return list.sort((a, b) => b.severity - a.severity);
  }, [
    rosbridgeStatus,
    health,
    tfLinks,
    lifecycle,
    battery,
    config.lowBatteryThreshold,
    devices,
    deviceStatuses,
    diagnosticsMsgs,
    missingTopics,
  ]);

  const overall = issues.length
    ? Math.max(...issues.map((i) => i.severity))
    : 0;

  // Build a baseline while ROS subscriptions and lifecycle polls settle. Only
  // record new faults that persist, so startup ordering and one failed poll do
  // not become permanent entries in this session's history.
  const faultLogStartedAt = useRef(Date.now());
  const knownIssueIds = useRef(new Set());
  const pendingIssueSince = useRef(new Map());
  useEffect(() => {
    const now = Date.now();
    const currentIds = new Set(issues.map((i) => i.id));
    if (now - faultLogStartedAt.current < FAULT_STARTUP_GRACE_MS) {
      knownIssueIds.current = currentIds;
      pendingIssueSince.current.clear();
      return;
    }

    for (const id of knownIssueIds.current) {
      if (!currentIds.has(id)) knownIssueIds.current.delete(id);
    }
    for (const id of pendingIssueSince.current.keys()) {
      if (!currentIds.has(id)) pendingIssueSince.current.delete(id);
    }

    const confirmed = issues.filter((issue) => {
      if (knownIssueIds.current.has(issue.id)) return false;
      const firstSeen = pendingIssueSince.current.get(issue.id) ?? now;
      pendingIssueSince.current.set(issue.id, firstSeen);
      if (now - firstSeen < FAULT_CONFIRM_MS) return false;
      pendingIssueSince.current.delete(issue.id);
      knownIssueIds.current.add(issue.id);
      return true;
    });
    if (confirmed.length === 0) return;

    setFaultLog((prev) =>
      [
        ...confirmed.map((issue) => ({
          id: issue.id,
          time: now,
          message: issue.message,
          severity: issue.severity,
        })),
        ...prev,
      ].slice(0, FAULT_LOG_LIMIT),
    );
  }, [issues]);

  return {
    reportHealth,
    reportLifecycle,
    battery,
    diagnosticsMsgs,
    missingTopics,
    devices,
    deviceStatuses,
    issues,
    overall,
    overallLabel: OVERALL_LABELS[overall],
    faultLog,
  };
}

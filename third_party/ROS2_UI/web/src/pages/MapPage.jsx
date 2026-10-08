import React, { useState, useRef, useEffect, useCallback } from "react";
import { ToastContainer, toast } from "react-toastify";
import "react-toastify/dist/ReactToastify.css";

import { useRos, useRuntimeConfig } from "../app/App";
import { AppConfig } from "../shared/constants";

import Map from "../components/Map";
import Camera from "../components/Camera";
import Joystick from "../components/Joystick";
import RobotState from "../components/RobotState";
import { MetricCard } from "../shared/ui/Dashboard";
import NavStatus from "../components/NavStatus";
import MapLayers from "../components/MapLayers";
import SystemAlerts from "../components/SystemAlerts";
import WaypointLibrary from "../components/WaypointLibrary";
import useSavedWaypoints from "../shared/hooks/useSavedWaypoints";
import useKeepoutZones from "../shared/hooks/useKeepoutZones";
import { addEvent } from "../shared/events/eventLog";
import { INSPECTION_PROFILE } from "../shared/robot/robotContract";
import { useT, T } from "../shared/i18n/i18n";

const INITIAL_POSE_COV = [
  0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.0685,
];

const NAVIGATION_STATE = {
  PLANNING: 1,
  MOVING: 2,
  ARRIVED: 3,
  FAILED: 4,
};

const MapPage = () => {
  const ros = useRos();
  const { config } = useRuntimeConfig();
  const { t } = useT();
  const mapRef = useRef(null);

  // mode: null | 'goal' | 'pose' | 'waypoint'
  const [mode, setModeState] = useState(null);
  const modeRef = useRef(null);
  const setMode = (m) => {
    modeRef.current = m;
    setModeState(m);
  };

  const [waypointQueue, setWaypointQueueState] = useState([]);
  const waypointQueueRef = useRef([]);
  const setWaypointQueue = (updater) => {
    const next =
      typeof updater === "function"
        ? updater(waypointQueueRef.current)
        : updater;
    waypointQueueRef.current = next;
    setWaypointQueueState(next);
  };

  const [queueExecuting, setQueueExecuting] = useState(false);
  const [activeWaypointIndex, setActiveWaypointIndex] = useState(-1);
  const [completedWaypointCount, setCompletedWaypointCount] = useState(0);
  const queueExecutingRef = useRef(false);
  const queueIdxRef = useRef(0);
  const queueGoalActiveRef = useRef(false);
  const queueGoalSentAtRef = useRef(0);
  const queueNearGoalSinceRef = useRef(0);
  const queueAdvanceTimerRef = useRef(null);
  const queueVelocityRef = useRef(Number.POSITIVE_INFINITY);
  const [savedRoutes, setSavedRoutes] = useState([]);
  const [selectedRoute, setSelectedRoute] = useState("");
  const [routeLoading, setRouteLoading] = useState(false);
  const routeContextRef = useRef({ group: "", map: "" });
  const selectedRouteRef = useRef(selectedRoute);
  selectedRouteRef.current = selectedRoute;
  const routeOperationTopicRef = useRef(null);
  const pendingRouteLoadRef = useRef(false);
  const pendingRouteTimerRef = useRef(null);
  const previewedRouteRef = useRef("");

  const { waypoints, addWaypoint, removeWaypoint } = useSavedWaypoints();
  useKeepoutZones();
  const waypointsRef = useRef(waypoints);
  useEffect(() => {
    waypointsRef.current = waypoints;
  }, [waypoints]);

  // Draw every queued waypoint on the map, not just the single in-flight goal.
  useEffect(() => {
    window.NAV2D?.setQueuedWaypoints?.(waypointQueue, {
      activeIndex: activeWaypointIndex,
      completedCount: completedWaypointCount,
    });
  }, [waypointQueue, activeWaypointIndex, completedWaypointCount]);

  const goalPoseTopic = useRef(null);
  const initialPoseTopic = useRef(null);
  const stopTopic = useRef(null);
  const cmdVelTopic = useRef(null);
  const pendingInitialPoseRef = useRef(false);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    goalPoseTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.GOAL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseStamped",
    });

    initialPoseTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.INITIAL_POSE_TOPIC,
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    stopTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.STOP_TOPIC,
      messageType: "std_msgs/Bool",
    });

    cmdVelTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });

    const localizationTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.LOCALIZATION_POSE_TOPIC,
      messageType: AppConfig.LOCALIZATION_POSE_TYPE,
    });
    localizationTopic.subscribe(() => {
      if (!pendingInitialPoseRef.current) return;
      pendingInitialPoseRef.current = false;
      toast.success(t("Robot's position estimate updated"));
    });

    return () => localizationTopic.unsubscribe();
  }, [ros]);

  // Route files are authored on the Routes page, then loaded here into the
  // same queue used by hand-placed waypoints. The route store scopes its
  // catalog to the occupancy map currently loaded in the simulation.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const requestTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROUTE_DATA_REQ_TOPIC,
      messageType: "std_msgs/Empty",
    });
    const responseTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROUTE_DATA_RESP_TOPIC,
      messageType: "std_msgs/String",
    });
    const operationTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.UI_OPERATION_TOPIC,
      messageType: "std_msgs/String",
    });
    routeOperationTopicRef.current = operationTopic;
    const waypointsTopic = new window.ROSLIB.Topic({
      ros,
      name: "/WayPoints_topic",
      messageType: "robotpilot_ui_msgs/ArrayPoseStampedWithCovariance",
    });

    responseTopic.subscribe((message) => {
      try {
        const response = JSON.parse(message.data || "{}");
        const active = response.active_files || {};
        const mapChanged =
          routeContextRef.current.group !== (active.group || "") ||
          routeContextRef.current.map !== (active.map || "");
        routeContextRef.current = {
          group: active.group || "",
          map: active.map || "",
        };
        const options = [];
        for (const groupEntry of response.structure || []) {
          for (const [group, maps] of Object.entries(groupEntry || {})) {
            for (const mapEntry of maps || []) {
              for (const [map, routes] of Object.entries(mapEntry || {})) {
                if (group !== active.group || map !== active.map) continue;
                for (const file of routes || []) {
                  const name = String(file).replace(/\.csv$/i, "");
                  options.push({ value: name, label: name.replace(/_/g, " ") });
                }
              }
            }
          }
        }
        setSavedRoutes(options);
        if (mapChanged) {
          previewedRouteRef.current = "";
          setWaypointQueue([]);
          setActiveWaypointIndex(-1);
          setCompletedWaypointCount(0);
        }
        setSelectedRoute((current) =>
          !mapChanged && options.some((route) => route.value === current)
            ? current
            : "",
        );
      } catch (error) {
        console.error("Invalid route catalog response:", error);
      }
    });

    waypointsTopic.subscribe((message) => {
      if (!pendingRouteLoadRef.current) return;
      const shouldExecute = pendingRouteLoadRef.current === "execute";
      pendingRouteLoadRef.current = false;
      window.clearTimeout(pendingRouteTimerRef.current);
      pendingRouteTimerRef.current = null;
      setRouteLoading(false);

      const route = (message?.poses || [])
        .map((entry) => {
          const pose = entry?.pose?.pose;
          const covariance = entry?.pose?.covariance || [];
          if (!pose) return null;
          return {
            ...pose,
            dwellSeconds:
              Math.max(0, Number(covariance[1]) || 0) * 3600 +
              Math.max(0, Number(covariance[2]) || 0) * 60,
          };
        })
        .filter((pose) => pose?.position && pose?.orientation);
      if (!route.length) {
        previewedRouteRef.current = "";
        setWaypointQueue([]);
        setActiveWaypointIndex(-1);
        setCompletedWaypointCount(0);
        toast.warn(t("The selected route has no waypoints."));
        return;
      }

      setWaypointQueue(route);
      previewedRouteRef.current = selectedRouteRef.current;
      queueIdxRef.current = 0;
      queueExecutingRef.current = shouldExecute;
      queueGoalActiveRef.current = false;
      queueNearGoalSinceRef.current = 0;
      setActiveWaypointIndex(shouldExecute ? 0 : -1);
      setCompletedWaypointCount(0);
      setQueueExecuting(shouldExecute);
      if (shouldExecute) {
        publishGoal(route[0]);
        toast.info(`${t("Executing route")}: ${selectedRouteRef.current} (${route.length})`);
      }
    });

    requestTopic.publish();
    return () => {
      responseTopic.unsubscribe();
      waypointsTopic.unsubscribe();
      if (pendingRouteTimerRef.current) {
        window.clearTimeout(pendingRouteTimerRef.current);
      }
      pendingRouteLoadRef.current = false;
      pendingRouteTimerRef.current = null;
      if (routeOperationTopicRef.current === operationTopic) {
        routeOperationTopicRef.current = null;
      }
      setRouteLoading(false);
    };
  }, [ros, t]);

  const finishQueueWaypoint = () => {
    if (!queueExecutingRef.current || queueGoalActiveRef.current === false)
      return;
    queueGoalActiveRef.current = false;
    const current = waypointQueueRef.current[queueIdxRef.current];
    setCompletedWaypointCount(queueIdxRef.current + 1);
    setActiveWaypointIndex(-1);
    const dwellMs = Math.max(0, Number(current?.dwellSeconds) || 0) * 1000;
    if (queueAdvanceTimerRef.current)
      window.clearTimeout(queueAdvanceTimerRef.current);
    queueAdvanceTimerRef.current = window.setTimeout(() => {
      queueAdvanceTimerRef.current = null;
      if (!queueExecutingRef.current) return;
      const next = queueIdxRef.current + 1;
      if (next < waypointQueueRef.current.length) {
        queueIdxRef.current = next;
        queueNearGoalSinceRef.current = 0;
        setActiveWaypointIndex(next);
        publishGoal(waypointQueueRef.current[next]);
        toast.info(
          `${t("Waypoint")} ${next + 1} / ${waypointQueueRef.current.length}`,
        );
      } else {
        queueExecutingRef.current = false;
        setQueueExecuting(false);
        setActiveWaypointIndex(-1);
        setCompletedWaypointCount(waypointQueueRef.current.length);
        toast.success(t("All waypoints complete!"));
      }
    }, dwellMs);
  };

  // Advance the queue from this project's navigation-state topic. Goals from
  // this page use /goal_pose and are tracked by nav_status, not by the Nav2
  // action status relay.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;

    const statusTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.NAVIGATION_STATE_TOPIC,
      messageType: AppConfig.NAVIGATION_STATE_TYPE,
    });

    statusTopic.subscribe((msg) => {
      if (!queueExecutingRef.current) return;

      if (
        msg.state === NAVIGATION_STATE.PLANNING ||
        msg.state === NAVIGATION_STATE.MOVING
      ) {
        queueGoalActiveRef.current = true;
        return;
      }

      // Ignore stale ARRIVED heartbeats until the current goal has entered
      // PLANNING or MOVING.
      if (
        msg.state === NAVIGATION_STATE.ARRIVED &&
        queueGoalActiveRef.current
      ) {
        finishQueueWaypoint();
      } else if (
        msg.state === NAVIGATION_STATE.FAILED &&
        queueGoalActiveRef.current
      ) {
        queueGoalActiveRef.current = false;
        queueExecutingRef.current = false;
        setQueueExecuting(false);
        setActiveWaypointIndex(-1);
        toast.warn(t("Queue stopped: goal was canceled or failed"));
      }
    });

    return () => statusTopic.unsubscribe();
  }, [ros]);

  // The status topic is the primary completion signal. This odometry fallback
  // advances a route if the UI bridge misses ARRIVED, but only after the robot
  // remains stopped close to the active waypoint.
  useEffect(() => {
    if (!ros || !window.ROSLIB) return;
    const odomTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.ROBOT_POSE_TOPIC,
      messageType: "nav_msgs/Odometry",
    });
    odomTopic.subscribe((msg) => {
      const twist = msg?.twist?.twist;
      if (!twist) return;
      queueVelocityRef.current = Math.hypot(
        twist.linear.x || 0,
        twist.linear.y || 0,
      );
    });
    const timer = window.setInterval(() => {
      if (
        !queueExecutingRef.current ||
        queueAdvanceTimerRef.current ||
        !queueGoalSentAtRef.current
      )
        return;
      const goal = waypointQueueRef.current[queueIdxRef.current];
      const pose = window.NAV2D?.currentPose;
      if (!goal?.position || !pose?.position) return;
      const distance = Math.hypot(
        goal.position.x - pose.position.x,
        goal.position.y - pose.position.y,
      );
      if (distance <= 0.35 && queueVelocityRef.current <= 0.08) {
        if (!queueNearGoalSinceRef.current)
          queueNearGoalSinceRef.current = Date.now();
        const goalSentLongEnoughAgo =
          Date.now() - queueGoalSentAtRef.current >= 5000;
        if (
          Date.now() - queueNearGoalSinceRef.current >= 1000 &&
          goalSentLongEnoughAgo
        ) {
          queueGoalActiveRef.current = true;
          finishQueueWaypoint();
        }
      } else {
        queueNearGoalSinceRef.current = 0;
      }
    }, 100);
    return () => {
      odomTopic.unsubscribe();
      window.clearInterval(timer);
    };
  }, [ros]);

  const publishGoal = (pose) => {
    if (INSPECTION_PROFILE) return;
    if (!goalPoseTopic.current) return;
    stopTopic.current?.publish(new window.ROSLIB.Message({ data: false }));
    queueGoalSentAtRef.current = Date.now();
    queueNearGoalSinceRef.current = 0;
    goalPoseTopic.current.publish(
      new window.ROSLIB.Message({
        header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x: pose.position.x, y: pose.position.y, z: 0 },
          orientation: {
            x: 0,
            y: 0,
            z: pose.orientation.z,
            w: pose.orientation.w,
          },
        },
      }),
    );
    window.NAV2D?.setGoalPose?.(pose);
  };

  const goToWaypoint = useCallback((wp) => {
    publishGoal({
      position: { x: wp.x, y: wp.y, z: 0 },
      orientation: { x: 0, y: 0, z: wp.z, w: wp.w },
    });
    toast.success(`${t("Navigating to")} "${wp.name}"`);
  }, []);

  // The three map right-click context-menu actions (Map.jsx's onContext*
  // props) — each reuses the exact same publish/topic logic as the
  // corresponding mode-button flow above, just without requiring a mode to
  // be active first or a heading drag (orientation defaults to identity).
  const sendGoalAt = (pose) => {
    publishGoal(pose);
    toast.success(
      `${t("Goal")}: (${pose.position.x.toFixed(2)}, ${pose.position.y.toFixed(
        2,
      )}) m`,
    );
  };

  const saveWaypointAt = (name, pose) => {
    addWaypoint(name, {
      x: pose.position.x,
      y: pose.position.y,
      z: pose.orientation.z,
      w: pose.orientation.w,
    });
    toast.success(`${t("Saved")} "${name}"`);
  };

  const setInitialPoseAt = (pose) => {
    if (INSPECTION_PROFILE) return;
    if (!initialPoseTopic.current) return;
    initialPoseTopic.current.publish(
      new window.ROSLIB.Message({
        header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
        pose: {
          pose: {
            position: { x: pose.position.x, y: pose.position.y, z: 0 },
            orientation: {
              x: 0,
              y: 0,
              z: pose.orientation.z,
              w: pose.orientation.w,
            },
          },
          covariance: INITIAL_POSE_COV,
        },
      }),
    );
    pendingInitialPoseRef.current = true;
    window.NAV2D?.clearTrail?.();
    window.NAV2D?.clearGoalPose?.();
    toast.success(
      `${t("Initial pose set")}: (${pose.position.x.toFixed(
        2,
      )}, ${pose.position.y.toFixed(2)}) m`,
    );
  };

  // Clicking a saved-waypoint pin on the map fires this — set once (not
  // re-registered every render) and reading the latest list via a ref, the
  // same pattern modeRef/waypointQueueRef already use in this file.
  useEffect(() => {
    if (!window.NAV2D) return undefined;
    window.NAV2D._savedWaypointClickCallback = (id) => {
      const wp = waypointsRef.current.find((w) => w.id === id);
      if (wp) goToWaypoint(wp);
    };
    return () => {
      window.NAV2D._savedWaypointClickCallback = null;
    };
  }, [goToWaypoint]);

  // Install the direct NAV2D callback — fires synchronously from stagemouseup,
  // no DOM bubbling or setTimeout needed.
  const installCallback = useCallback(() => {
    if (INSPECTION_PROFILE) return;
    if (!window.NAV2D) return;
    window.NAV2D._poseCallback = (pose) => {
      const m = modeRef.current;
      if (m === "goal") {
        publishGoal(pose);
        toast.success(
          `${t("Goal")}: (${pose.position.x.toFixed(
            2,
          )}, ${pose.position.y.toFixed(2)}) m`,
        );
      } else if (m === "pose") {
        if (!initialPoseTopic.current) return;
        initialPoseTopic.current.publish(
          new window.ROSLIB.Message({
            header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
            pose: {
              pose: {
                position: { x: pose.position.x, y: pose.position.y, z: 0 },
                orientation: {
                  x: 0,
                  y: 0,
                  z: pose.orientation.z,
                  w: pose.orientation.w,
                },
              },
              covariance: INITIAL_POSE_COV,
            },
          }),
        );
        pendingInitialPoseRef.current = true;
        if (window.NAV2D?.clearTrail) window.NAV2D.clearTrail();
        window.NAV2D?.clearGoalPose?.();
        toast.success(
          `${t("Initial pose set")}: (${pose.position.x.toFixed(
            2,
          )}, ${pose.position.y.toFixed(2)}) m`,
        );
        deactivateMode();
      } else if (m === "waypoint") {
        window.NAV2D?.clearGoalPose?.();
        const idx = waypointQueueRef.current.length;
        setWaypointQueue((prev) => [...prev, pose]);
        toast.info(`${t("Waypoint")} ${idx + 1} ${t("added")}`);
      }
    };
  }, []);

  const activateMode = useCallback(
    (m) => {
      if (INSPECTION_PROFILE) return;
      if (!window.NAV2D) return;
      window.NAV2D.arePointsSettable = true;
      installCallback();
      setMode(m);
    },
    [installCallback],
  );

  const deactivateMode = useCallback(() => {
    if (window.NAV2D) {
      window.NAV2D.arePointsSettable = false;
      window.NAV2D._poseCallback = null;
    }
    setMode(null);
  }, []);

  // Re-install callback whenever mode changes so the closure always has the right mode
  useEffect(() => {
    if (["goal", "pose", "waypoint"].includes(modeRef.current)) installCallback();
  }, [mode, installCallback]);

  const cancelGoal = useCallback(() => {
    stopTopic.current?.publish(new window.ROSLIB.Message({ data: true }));
    const currentPose = window.NAV2D?.currentPose;
    if (currentPose && goalPoseTopic.current) {
      goalPoseTopic.current.publish(
        new window.ROSLIB.Message({
          header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
          pose: {
            position: {
              x: currentPose.position.x,
              y: currentPose.position.y,
              z: currentPose.position.z || 0,
            },
            orientation: {
              x: currentPose.orientation.x || 0,
              y: currentPose.orientation.y || 0,
              z: currentPose.orientation.z,
              w: currentPose.orientation.w,
            },
          },
        }),
      );
    }
    window.NAV2D?.clearGoalPose?.();
  }, []);

  const emergencyStop = useCallback(() => {
    queueExecutingRef.current = false;
    queueGoalActiveRef.current = false;
    if (queueAdvanceTimerRef.current)
      window.clearTimeout(queueAdvanceTimerRef.current);
    setQueueExecuting(false);
    setWaypointQueue([]);
    setActiveWaypointIndex(-1);
    setCompletedWaypointCount(0);
    if (cmdVelTopic.current) {
      cmdVelTopic.current.publish(
        new window.ROSLIB.Message({
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        }),
      );
    }
    cancelGoal();
    addEvent({
      type: "safety",
      severity: "error",
      message: "Emergency stop — robot halted",
    });
    toast.warn("Emergency stop — robot halted");
  }, [cancelGoal]);

  const sendHome = useCallback(() => {
    if (INSPECTION_PROFILE) return;
    if (!goalPoseTopic.current) return;
    stopTopic.current?.publish(new window.ROSLIB.Message({ data: false }));
    goalPoseTopic.current.publish(
      new window.ROSLIB.Message({
        header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
        pose: {
          position: { x: 0, y: 0, z: 0 },
          orientation: { x: 0, y: 0, z: 0, w: 1 },
        },
      }),
    );
    window.NAV2D?.setGoalPose?.({
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    });
    toast.info(t("Navigating to home position (0, 0) m"));
  }, []);

  const executeQueue = useCallback(() => {
    if (INSPECTION_PROFILE) return;
    if (!waypointQueueRef.current.length) return;
    queueIdxRef.current = 0;
    queueExecutingRef.current = true;
    queueGoalActiveRef.current = false;
    setActiveWaypointIndex(0);
    setCompletedWaypointCount(0);
    if (queueAdvanceTimerRef.current)
      window.clearTimeout(queueAdvanceTimerRef.current);
    setQueueExecuting(true);
    publishGoal(waypointQueueRef.current[0]);
    toast.info(
      `${t("Executing")} ${waypointQueueRef.current.length} ${t("waypoints")}`,
    );
  }, []);

  const loadSelectedRoute = (route, execute = false) => {
    if (INSPECTION_PROFILE || queueExecutingRef.current || routeLoading) return;
    if (!route) {
      toast.warn(t("Select a saved route first."));
      return;
    }
    const { group, map } = routeContextRef.current;
    if (!group || !map || group === "Null" || map === "Null") {
      toast.warn(t("Waiting for the active simulation map."));
      return;
    }
    if (!routeOperationTopicRef.current) {
      toast.error(t("Route service is not connected."));
      return;
    }

    pendingRouteLoadRef.current = execute ? "execute" : "preview";
    setRouteLoading(true);
    pendingRouteTimerRef.current = window.setTimeout(() => {
      pendingRouteLoadRef.current = false;
      pendingRouteTimerRef.current = null;
      setRouteLoading(false);
      toast.error(t("Timed out while loading the selected route."));
    }, 10000);

    routeOperationTopicRef.current.publish(
      new window.ROSLIB.Message({
        data: `change_route/${JSON.stringify({
          group,
          map,
          route,
        })}`,
      }),
    );
  };

  const executeSelectedRoute = () => {
    if (previewedRouteRef.current === selectedRoute && waypointQueueRef.current.length) {
      executeQueue();
      return;
    }
    loadSelectedRoute(selectedRoute, true);
  };

  const stopQueue = useCallback(() => {
    queueExecutingRef.current = false;
    queueGoalActiveRef.current = false;
    if (queueAdvanceTimerRef.current)
      window.clearTimeout(queueAdvanceTimerRef.current);
    queueAdvanceTimerRef.current = null;
    setQueueExecuting(false);
    setActiveWaypointIndex(-1);
    cancelGoal();
    toast.info(t("Queue stopped"));
  }, [cancelGoal]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (window.NAV2D) {
        window.NAV2D.arePointsSettable = false;
        window.NAV2D._poseCallback = null;
      }
    };
  }, []);

  const modeBtn = (label, shortLabel, m, activeLabel, shortActiveLabel) => {
    const active = mode === m;
    return (
      <button
        onClick={() => (active ? deactivateMode() : activateMode(m))}
        className={`flex min-h-[52px] w-full items-center justify-center rounded-xl border px-2 py-2 text-center font-[RobotoMono] text-[10px] font-semibold leading-tight transition-colors sm:px-3 sm:text-sm ${
          active
            ? "border-themeBlue bg-themeBlue text-white"
            : "border-borderSubtle bg-bgCard text-themeBlue hover:border-themeBlue"
        }`}
      >
        <span className="sm:hidden">
          {t(active ? shortActiveLabel : shortLabel)}
        </span>
        <span className="hidden sm:inline">
          {t(active ? activeLabel : label)}
        </span>
      </button>
    );
  };

  return (
    <>
      <ToastContainer position="bottom-right" theme="dark" />

      <div className="flex min-h-[calc(100vh-145px)] min-w-0 flex-col gap-3 py-3">
        {!INSPECTION_PROFILE && <SystemAlerts />}
        {INSPECTION_PROFILE && (
          <p className="dashboard-card p-3 text-sm text-statusYellow">
            {t(
              "Project navigation and localization interfaces are unconfigured. Status: UNKNOWN.",
            )}
          </p>
        )}
        <MapLayers />

        {/* Keep telemetry in a narrow sidebar beside the map on desktop. */}
        <section
          className={`grid min-w-0 grid-cols-1 gap-3 ${
            INSPECTION_PROFILE
              ? "xl:grid-cols-[minmax(0,1.4fr)_minmax(320px,1fr)]"
              : "xl:grid-cols-[minmax(190px,0.32fr)_minmax(0,1.35fr)_minmax(320px,1fr)]"
          }`}
        >
          {!INSPECTION_PROFILE && (
            <div className="order-3 flex min-w-0 flex-col gap-3 xl:order-1">
              <NavStatus />
              <RobotState compact showPosition={false} splitVelocityCards />
              <MetricCard
                compact
                label="Distance to target"
                value="—"
                unit="m"
              />
              <RobotState
                compact
                showPosition
                showVelocity={false}
                separateHeading
              />
            </div>
          )}
          <div
            className={`h-[340px] min-w-0 sm:h-[440px] xl:h-[500px] ${
              INSPECTION_PROFILE ? "order-1 xl:order-1" : "order-1 xl:order-2"
            }`}
            data-tour="map-canvas"
          >
            <Map
              ref={mapRef}
              onContextGoal={INSPECTION_PROFILE ? undefined : sendGoalAt}
              onContextSavePose={saveWaypointAt}
              onContextSetPose={
                INSPECTION_PROFILE ? undefined : setInitialPoseAt
              }
            />
          </div>
          <div
            className={`order-2 h-[340px] min-w-0 sm:h-[440px] xl:h-[500px] ${
              INSPECTION_PROFILE ? "xl:order-2" : "xl:order-3"
            }`}
          >
            <Camera />
          </div>
        </section>

        {/* Manual drive and saved waypoints share an aligned responsive grid. */}
        <section
          className="grid min-w-0 grid-cols-1 items-stretch gap-3 sm:grid-cols-2 lg:grid-cols-12"
          data-tour="manual-drive"
        >
          <div className="dashboard-card flex min-h-[176px] min-w-0 flex-col items-center justify-center gap-2 p-3 sm:col-span-1 lg:col-span-3">
            <p className="font-[RobotoMono] text-xs uppercase tracking-wider text-themeTextGray">
              {t("Manual")}
            </p>
            {!INSPECTION_PROFILE ? (
              <Joystick maxSpeed={config.maxLinearSpeed} compact />
            ) : (
              <p className="text-center text-xs text-themeTextGray">
                {t("Manual control interface not configured")}
              </p>
            )}
            <button
              onClick={emergencyStop}
              disabled={INSPECTION_PROFILE}
              title={
                INSPECTION_PROFILE
                  ? "Software Stop unavailable: robot interface not configured; not physical E-STOP"
                  : "Software stop request; not physical E-STOP"
              }
              className="mt-1 h-9 w-9 shrink-0 rounded-full border-2 border-statusRed bg-statusRed/10 font-[RobotoMono] text-[9px] font-bold leading-none text-statusRed transition-colors hover:bg-statusRed hover:text-white"
            >
              {INSPECTION_PROFILE ? t("Software Stop") : "STOP"}
            </button>
          </div>

          <div className="min-w-0 sm:col-span-2 lg:col-span-5 [&>div]:h-full">
            <WaypointLibrary
              waypoints={waypoints}
              onAdd={saveWaypointAt}
              onGo={INSPECTION_PROFILE ? undefined : goToWaypoint}
              onRemove={removeWaypoint}
            />
          </div>

          {!INSPECTION_PROFILE && (
            <div className="dashboard-card flex min-w-0 flex-col gap-3 p-3 font-[RobotoMono] sm:col-span-2 sm:p-4 lg:col-span-4">
              <div>
                <p className="mb-2 text-xs uppercase tracking-wider text-themeTextGray">
                  {t("Saved Routes")}
                </p>
                <select
                  value={selectedRoute}
                  onChange={(event) => {
                    const route = event.target.value;
                    setSelectedRoute(route);
                    selectedRouteRef.current = route;
                    previewedRouteRef.current = "";
                    setWaypointQueue([]);
                    setActiveWaypointIndex(-1);
                    setCompletedWaypointCount(0);
                    loadSelectedRoute(route);
                  }}
                  disabled={
                    routeLoading || queueExecuting || !savedRoutes.length
                  }
                  className="min-h-10 w-full min-w-0 rounded-lg border border-borderSubtle bg-bgCard px-3 text-sm text-textWhiteHover outline-none focus:border-themeBlue disabled:opacity-50"
                >
                  <option value="" disabled hidden>
                    {t("Select a saved route…")}
                  </option>
                  {savedRoutes.map((route) => (
                    <option key={route.value} value={route.value}>
                      {route.label}
                    </option>
                  ))}
                </select>
                {!savedRoutes.length && (
                  <p className="mt-2 text-xs leading-5 text-themeTextGray">
                    {t("Create and save a route on the Routes page first.")}
                  </p>
                )}
              </div>
              <button
                onClick={executeSelectedRoute}
                disabled={
                  !selectedRoute ||
                  routeLoading ||
                  queueExecuting ||
                  !savedRoutes.length
                }
                className="min-h-10 w-full rounded-lg border border-themeBlue bg-themeBlue/10 px-4 text-xs font-semibold text-themeBlue transition-colors hover:bg-themeBlue hover:text-white disabled:cursor-not-allowed disabled:opacity-50"
              >
                {t(routeLoading ? "Loading route…" : "Execute Route")}
              </button>
              {waypointQueue.length > 0 && (
                <div className="min-w-0 border-t border-borderSubtle pt-3">
                  <div className="mb-2 flex items-center justify-between gap-2">
                    <p className="text-xs uppercase tracking-wider text-themeTextGray">
                      {t("Waypoint Queue")} ({waypointQueue.length})
                    </p>
                    <button
                      onClick={() => {
                        setWaypointQueue([]);
                        setActiveWaypointIndex(-1);
                        setCompletedWaypointCount(0);
                        stopQueue();
                      }}
                      className="text-xs text-statusRed hover:underline"
                    >
                      <T>{"Clear"}</T>
                    </button>
                  </div>
                  <div className="mb-3 flex max-h-36 flex-wrap gap-1.5 overflow-y-auto">
                    {waypointQueue.map((wp, i) => (
                      <span
                        key={i}
                        className={`rounded border px-2 py-0.5 text-xs ${
                          queueExecuting && i === activeWaypointIndex
                            ? "border-themeBlue bg-themeBlue/20 text-themeBlue"
                            : "border-borderSubtle text-themeTextGray"
                        }`}
                      >
                        {i + 1}: ({wp.position.x.toFixed(1)}, {wp.position.y.toFixed(1)}) m
                      </span>
                    ))}
                  </div>
                  {queueExecuting ? (
                    <button onClick={stopQueue} className="w-full rounded-lg border border-statusRed bg-bgCard py-1.5 text-xs font-semibold text-statusRed hover:bg-statusRed hover:text-white">
                      <T>{"Stop Queue"}</T>
                    </button>
                  ) : (
                    <button onClick={executeQueue} className="w-full rounded-lg border border-themeBlue bg-themeBlue/10 py-1.5 text-xs font-semibold text-themeBlue hover:bg-themeBlue hover:text-white">
                      <T>{"Execute Queue"}</T>
                    </button>
                  )}
                </div>
              )}
            </div>
          )}
        </section>

        {/* Map actions and route execution share a single operations area. */}
        {!INSPECTION_PROFILE && (
          <section className="min-w-0" data-tour="map-actions">
            <div className="dashboard-card flex min-w-0 flex-col justify-center gap-3 p-3 sm:p-4">
              <div className="grid grid-cols-2 gap-2 sm:grid-cols-4">
                {modeBtn(
                  "○ Send Goal",
                  "Goal",
                  "goal",
                  "● Click to Send Goal",
                  "● Goal",
                )}
                {modeBtn(
                  "⊕ Correct Robot's Position",
                  "Fix Position",
                  "pose",
                  "● Click to Correct Position",
                  "● Fixing",
                )}
                {modeBtn(
                  "＋ Add Waypoint",
                  "Waypoint",
                  "waypoint",
                  "● Adding Waypoints",
                  "● Adding",
                )}
                <button
                  onClick={sendHome}
                  className="min-h-[52px] w-full rounded-xl border border-borderSubtle bg-bgCard px-2 py-2 text-center font-[RobotoMono] text-[10px] font-semibold leading-tight text-textWhiteHover transition-colors hover:border-themeBlue hover:text-themeBlue sm:px-3 sm:text-sm"
                >
                  <span className="sm:hidden">
                    <T>{"Home"}</T>
                  </span>
                  <span className="hidden sm:inline">
                    <T>{"⌂ Go Home"}</T>
                  </span>
                </button>
              </div>

              <p className="min-h-5 px-1 font-[RobotoMono] text-xs leading-5 text-themeTextGray">
                {mode === "goal" &&
                  t(
                    "Click map to navigate. Drag before releasing to set heading.",
                  )}
                {mode === "pose" &&
                  t(
                    "Click the map to tell the robot where it currently is. Drag to set heading. One-shot.",
                  )}
                {mode === "waypoint" &&
                  t(
                    "Each click adds a waypoint. Drag to set heading. Execute all below.",
                  )}
                {!mode && t("Select a mode above to interact with the map.")}
              </p>
            </div>

          </section>
        )}
      </div>
    </>
  );
};

export default MapPage;

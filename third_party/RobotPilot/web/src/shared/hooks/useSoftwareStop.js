import { useCallback, useContext, useEffect, useRef, useState } from "react";

import { AuthContext, useRos, useRosStatus } from "../../app/App";
import { hasRole } from "../auth/roleAccess";
import { AppConfig } from "../constants";
import { useT } from "../i18n/i18n";

const STATE_STALE_MS = 3000;
const ACK_TIMEOUT_MS = 3000;

export const softwareStopCapabilities = (mode, identity) => {
  return {
    requestStop: hasRole(mode, identity, "Operator"),
    releaseStop: hasRole(mode, identity, "Engineer"),
  };
};

export const canStartMission = ({ robotState, stateFresh, requestState }) =>
  Boolean(
    stateFresh &&
      robotState?.active === false &&
      !["pending", "unknown"].includes(requestState?.status),
  );

export const isSoftwareStopStateFresh = (state, now = Date.now()) =>
  Boolean(state && now - state.receivedAt <= STATE_STALE_MS);

export const parseSoftwareStopState = (value) => {
  try {
    const state = typeof value === "string" ? JSON.parse(value) : value;
    if (!state || typeof state.active !== "boolean") return null;
    if (
      typeof state.observed_at !== "string" ||
      !Number.isFinite(Date.parse(state.observed_at))
    )
      return null;
    return {
      active: state.active,
      source: typeof state.source === "string" ? state.source : "unknown",
      reason: typeof state.reason === "string" ? state.reason : "",
      request_id: typeof state.request_id === "string" ? state.request_id : "",
      observed_at: state.observed_at,
      result: ["rejected", "degraded"].includes(state.result)
        ? state.result
        : "confirmed",
      durable: state.durable === true,
      physical_estop:
        state.physical_estop &&
        typeof state.physical_estop.available === "boolean"
          ? state.physical_estop
          : { available: false, active: null },
      receivedAt: Date.now(),
    };
  } catch {
    return null;
  }
};

const createRequestId = () =>
  globalThis.crypto?.randomUUID?.() ||
  `software-stop-${Date.now()}-${Math.random().toString(16).slice(2)}`;

const useSoftwareStop = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  const { mode, identity } = useContext(AuthContext);
  const { t } = useT();
  const [robotState, setRobotState] = useState(null);
  const [requestState, setRequestState] = useState(null);
  const [, setFreshnessTick] = useState(0);
  const requestTopicRef = useRef(null);
  const acknowledgementTimerRef = useRef(null);
  const capabilities = softwareStopCapabilities(mode, identity);
  const canRequestStop = capabilities.requestStop;
  const canReleaseStop = capabilities.releaseStop;
  const stateFresh = isSoftwareStopStateFresh(robotState);

  useEffect(() => {
    if (!robotState) return undefined;
    const staleAt = robotState.receivedAt + STATE_STALE_MS + 1;
    const timeout = window.setTimeout(
      () => setFreshnessTick((tick) => tick + 1),
      Math.max(0, staleAt - Date.now()),
    );
    return () => window.clearTimeout(timeout);
  }, [robotState]);

  useEffect(() => {
    if (!ros || rosStatus !== "connected" || !window.ROSLIB) return undefined;
    const requestTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.SOFTWARE_STOP_REQUEST_TOPIC,
      messageType: "std_msgs/String",
    });
    const stateTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.SOFTWARE_STOP_STATE_TOPIC,
      messageType: "std_msgs/String",
    });
    requestTopicRef.current = requestTopic;
    const onState = ({ data }) => {
      const nextState = parseSoftwareStopState(data);
      if (!nextState) return;
      setRobotState(nextState);
      setRequestState((current) => {
        if (!current || current.requestId !== nextState.request_id)
          return current;
        if (acknowledgementTimerRef.current) {
          window.clearTimeout(acknowledgementTimerRef.current);
          acknowledgementTimerRef.current = null;
        }
        return {
          ...current,
          status:
            nextState.result === "rejected"
              ? "rejected"
              : nextState.result === "degraded"
              ? "unknown"
              : "confirmed",
          reason: nextState.reason,
        };
      });
    };
    stateTopic.subscribe(onState);

    const stateRequest = JSON.stringify({
      action: "get_state",
      request_id: createRequestId(),
    });
    requestTopic.publish(new window.ROSLIB.Message({ data: stateRequest }));

    return () => {
      stateTopic.unsubscribe(onState);
      requestTopic.unadvertise?.();
      requestTopicRef.current = null;
      if (acknowledgementTimerRef.current) {
        window.clearTimeout(acknowledgementTimerRef.current);
        acknowledgementTimerRef.current = null;
      }
    };
  }, [ros, rosStatus]);

  useEffect(() => {
    if (rosStatus !== "connected") setRobotState(null);
  }, [rosStatus]);

  const request = useCallback(
    (action, reason = "") => {
      if (rosStatus !== "connected" || !requestTopicRef.current) {
        setRequestState({
          action,
          status: "unknown",
          requestId: "",
          reason: "ROS bridge offline",
        });
        return null;
      }
      const requestId = createRequestId();
      setRequestState({ action, status: "pending", requestId, reason: "" });
      requestTopicRef.current.publish(
        new window.ROSLIB.Message({
          data: JSON.stringify({ action, reason, request_id: requestId }),
        }),
      );
      if (acknowledgementTimerRef.current)
        window.clearTimeout(acknowledgementTimerRef.current);
      acknowledgementTimerRef.current = window.setTimeout(() => {
        setRequestState((current) =>
          current?.requestId === requestId && current.status === "pending"
            ? { ...current, status: "unknown" }
            : current,
        );
        acknowledgementTimerRef.current = null;
      }, ACK_TIMEOUT_MS);
      return requestId;
    },
    [rosStatus],
  );

  const requestStop = useCallback(() => {
    if (!canRequestStop) return null;
    return request("request_stop", "Software stop requested from Web HMI");
  }, [canRequestStop, request]);

  const requestRelease = useCallback(() => {
    if (
      !canReleaseStop ||
      !stateFresh ||
      !robotState?.active ||
      !robotState.durable ||
      robotState.result !== "confirmed"
    )
      return null;
    const confirmed = window.confirm(t("Release software stop confirmation"));
    if (!confirmed) return null;
    return request(
      "request_release",
      "Engineer confirmed software stop release",
    );
  }, [canReleaseStop, request, robotState, stateFresh, t]);

  return {
    robotState,
    stateFresh,
    requestState,
    canRequestStop,
    canReleaseStop,
    requestStop,
    requestRelease,
  };
};

export default useSoftwareStop;

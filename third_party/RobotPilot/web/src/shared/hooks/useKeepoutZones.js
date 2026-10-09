import { useCallback, useEffect, useRef, useState } from "react";

import { useRos, useRosStatus } from "../../app/App";
import { useRoleAccess } from "../auth/roleAccess";

/** Map rules are authoritative on the robot and scoped to the active map. */
export default function useKeepoutZones() {
  const ros = useRos();
  const status = useRosStatus();
  const editAccess = useRoleAccess("Engineer");
  const [snapshot, setSnapshot] = useState(null);
  const [observedAt, setObservedAt] = useState(null);
  const [connected, setConnected] = useState(false);
  const [awaitingMapRules, setAwaitingMapRules] = useState(false);
  const [now, setNow] = useState(Date.now());
  const [error, setError] = useState("");
  const commandRef = useRef(null);
  const pendingRef = useRef(null);
  const catalogMapRef = useRef("");
  const snapshotRef = useRef(snapshot);
  snapshotRef.current = snapshot;

  useEffect(() => {
    const timer = window.setInterval(() => setNow(Date.now()), 1000);
    return () => window.clearInterval(timer);
  }, []);

  const snapshotFresh =
    Boolean(snapshot?.ready) &&
    connected &&
    Boolean(observedAt) &&
    !awaitingMapRules &&
    now - Date.parse(observedAt) <= 5000;

  useEffect(() => {
    if (!ros || status !== "connected" || !window.ROSLIB) {
      setConnected(false);
      commandRef.current = null;
      pendingRef.current = null;
      return undefined;
    }
    setConnected(true);
    const stateTopic = new window.ROSLIB.Topic({
      ros,
      name: "/area_rules/state",
      messageType: "std_msgs/String",
    });
    const ackTopic = new window.ROSLIB.Topic({
      ros,
      name: "/area_rules/ack",
      messageType: "std_msgs/String",
    });
    const commandTopic = new window.ROSLIB.Topic({
      ros,
      name: "/area_rules/command",
      messageType: "std_msgs/String",
    });
    const catalogTopic = new window.ROSLIB.Topic({
      ros,
      name: "/ackermann/routes/catalog",
      messageType: "std_msgs/String",
    });
    commandRef.current = commandTopic;
    catalogTopic.subscribe((message) => {
      try {
        const active = JSON.parse(message.data || "{}").active_files || {};
        const nextMapId =
          active.map_id && active.map_id !== "Null" ? active.map_id : "";
        if (nextMapId !== (snapshotRef.current?.map_id || "")) {
          setAwaitingMapRules(true);
          window.NAV2D?.setAreaRules?.([]);
        }
        catalogMapRef.current = nextMapId;
      } catch {
        // Catalog messages are informational; the robot rule state remains authoritative.
      }
    });
    stateTopic.subscribe((message) => {
      try {
        const next = JSON.parse(message.data);
        if (!Array.isArray(next.rules)) return;
        if (!catalogMapRef.current || next.map_id !== catalogMapRef.current) {
          setAwaitingMapRules(true);
          window.NAV2D?.setAreaRules?.([]);
          return;
        }
        setSnapshot(next);
        setObservedAt(new Date().toISOString());
        setAwaitingMapRules(false);
        setError("");
        window.NAV2D?.setAreaRules?.(next.rules);
      } catch {
        setError("无法解析机器人返回的地图规则");
      }
    });
    ackTopic.subscribe((message) => {
      try {
        const ack = JSON.parse(message.data);
        if (ack.request_id !== pendingRef.current) return;
        pendingRef.current = null;
        setError(ack.ok ? "" : ack.error || "地图规则保存失败");
      } catch {
        // Ignore unrelated or malformed acknowledgements.
      }
    });
    return () => {
      stateTopic.unsubscribe();
      ackTopic.unsubscribe();
      catalogTopic.unsubscribe();
      commandRef.current = null;
      pendingRef.current = null;
    };
  }, [ros, status]);

  const send = useCallback(
    (action, rule) => {
      if (!editAccess.allowed) {
        setError(editAccess.reason);
        return false;
      }
      if (
        !snapshotFresh ||
        !snapshot.map_key ||
        !commandRef.current ||
        pendingRef.current
      ) {
        setError("地图规则服务未就绪，或仍在等待上一次操作");
        return false;
      }
      setError("");
      const requestId = `web-area-${Date.now()}-${Math.random()
        .toString(36)
        .slice(2)}`;
      pendingRef.current = requestId;
      commandRef.current.publish(
        new window.ROSLIB.Message({
          data: JSON.stringify({
            request_id: requestId,
            map_key: snapshot.map_key,
            expected_version: snapshot.version,
            action,
            rule,
          }),
        }),
      );
      // Lost acknowledgements must not leave the editor locked forever.
      setTimeout(() => {
        if (pendingRef.current === requestId) {
          pendingRef.current = null;
          setError("未收到机器人确认，请检查连接和规则列表");
        }
      }, 5000);
      return true;
    },
    [editAccess, snapshot, snapshotFresh],
  );

  return {
    rules: snapshot?.rules || [],
    ready: snapshotFresh,
    mapKey: snapshot?.map_key || null,
    mapId: snapshot?.map_id || null,
    mapVersionId: snapshot?.map_version_id || null,
    observedAt,
    stale:
      !connected ||
      awaitingMapRules ||
      !observedAt ||
      now - Date.parse(observedAt) > 5000,
    error,
    canEdit: editAccess.allowed,
    permissionReason: editAccess.reason,
    upsertRule: (rule) => send("upsert", rule),
    deleteRule: (id) => send("delete", { id }),
  };
}

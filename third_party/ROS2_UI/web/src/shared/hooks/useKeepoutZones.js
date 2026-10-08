import { useCallback, useEffect, useRef, useState } from "react";

import { useRos, useRosStatus } from "../../app/App";

/** Map rules are authoritative on the robot and scoped to the active map. */
export default function useKeepoutZones() {
  const ros = useRos();
  const status = useRosStatus();
  const [snapshot, setSnapshot] = useState(null);
  const [error, setError] = useState("");
  const commandRef = useRef(null);
  const pendingRef = useRef(null);

  useEffect(() => {
    if (!ros || status !== "connected" || !window.ROSLIB) {
      setSnapshot(null);
      window.NAV2D?.setAreaRules?.([]);
      commandRef.current = null;
      pendingRef.current = null;
      return undefined;
    }
    const stateTopic = new window.ROSLIB.Topic({
      ros, name: "/area_rules/state", messageType: "std_msgs/String",
    });
    const ackTopic = new window.ROSLIB.Topic({
      ros, name: "/area_rules/ack", messageType: "std_msgs/String",
    });
    const commandTopic = new window.ROSLIB.Topic({
      ros, name: "/area_rules/command", messageType: "std_msgs/String",
    });
    commandRef.current = commandTopic;
    stateTopic.subscribe((message) => {
      try {
        const next = JSON.parse(message.data);
        if (!Array.isArray(next.rules)) return;
        setSnapshot(next);
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
      commandRef.current = null;
      pendingRef.current = null;
    };
  }, [ros, status]);

  const send = useCallback((action, rule) => {
    if (!snapshot?.ready || !snapshot.map_key || !commandRef.current || pendingRef.current) {
      setError("地图规则服务未就绪，或仍在等待上一次操作");
      return false;
    }
    setError("");
    const requestId = `web-area-${Date.now()}-${Math.random().toString(36).slice(2)}`;
    pendingRef.current = requestId;
    commandRef.current.publish(new window.ROSLIB.Message({ data: JSON.stringify({
      request_id: requestId,
      map_key: snapshot.map_key,
      expected_version: snapshot.version,
      action,
      rule,
    }) }));
    // Lost acknowledgements must not leave the editor locked forever.
    setTimeout(() => {
      if (pendingRef.current === requestId) {
        pendingRef.current = null;
        setError("未收到机器人确认，请检查连接和规则列表");
      }
    }, 5000);
    return true;
  }, [snapshot]);

  return {
    rules: snapshot?.rules || [],
    ready: Boolean(snapshot?.ready),
    mapKey: snapshot?.map_key || null,
    error,
    upsertRule: (rule) => send("upsert", rule),
    deleteRule: (id) => send("delete", { id }),
  };
}

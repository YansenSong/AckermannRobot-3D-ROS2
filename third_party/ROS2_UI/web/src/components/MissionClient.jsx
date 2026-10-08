import { useEffect } from "react";
import { toast } from "react-toastify";
import { useRos, useRosStatus } from "../app/App";
import { AppConfig } from "../shared/constants";
import {
  replaceMissionsFromRobot,
  setMissionOnline,
} from "../shared/missions/missions";
import { setHistory, setRun } from "../shared/missions/missionClient";
import { setMissionCommandTransport } from "../shared/missions/transport";
import { newMissionId } from "../shared/missions/id";

// A ROS bridge only: closing this tab cannot stop the robot-side executor.
const MissionClient = () => {
  const ros = useRos();
  const rosStatus = useRosStatus();
  useEffect(() => {
    if (rosStatus !== "connected") {
      setMissionOnline(false);
      setMissionCommandTransport(null);
      return;
    }
    const commandTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.MISSION_COMMAND_TOPIC,
      messageType: "std_msgs/String",
    });
    const stateTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.MISSION_STATE_TOPIC,
      messageType: "std_msgs/String",
    });
    const ackTopic = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.MISSION_ACK_TOPIC,
      messageType: "std_msgs/String",
    });
    let latestState = 0;
    const send = (command) =>
      commandTopic.publish(
        new window.ROSLIB.Message({
          data: JSON.stringify({ ...command, request_id: newMissionId() }),
        }),
      );
    setMissionCommandTransport(send);
    stateTopic.subscribe((msg) => {
      try {
        const state = JSON.parse(msg.data);
        if (state.schema_version !== 1) return;
        latestState = Date.now();
        setMissionOnline(true);
        replaceMissionsFromRobot(state.missions);
        setRun(state.run);
        setHistory(state.history);
      } catch {
        /* ignore malformed ROS messages */
      }
    });
    ackTopic.subscribe((msg) => {
      try {
        const ack = JSON.parse(msg.data);
        if (ack.ok === false)
          toast.error(`Mission command rejected: ${ack.error}`);
      } catch {
        /* ignore malformed ROS messages */
      }
    });
    send({ command: "query" });
    const watchdog = setInterval(() => {
      if (Date.now() - latestState > 6000) setMissionOnline(false);
      else send({ command: "query" });
    }, 3000);
    return () => {
      clearInterval(watchdog);
      stateTopic.unsubscribe();
      ackTopic.unsubscribe();
      setMissionCommandTransport(null);
      setMissionOnline(false);
    };
  }, [ros, rosStatus]);
  return null;
};
export default MissionClient;

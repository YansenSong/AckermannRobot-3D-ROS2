import React, { useRef, useEffect, useState, useCallback } from "react";
import { Joystick } from "react-joystick-component";

import { AppConfig } from "../shared/constants/index";
import { useRos, useRuntimeConfig } from "../app/App";

// W3C Gamepad API 标准映射：左摇杆使用 axes[0]（x，正值向右）和 axes[1]（y，正值向下；与摇杆库 evt.y 的向上正值相反）。
const GAMEPAD_DEADZONE = 0.12;
const applyDeadzone = (value) => (Math.abs(value) > GAMEPAD_DEADZONE ? value : 0);

const JoystickComponent = ({ maxSpeed, compact = false }) => {
  const ros = useRos();
  const { config } = useRuntimeConfig();
  const joysticRef = useRef();
  const [size, setSize] = useState(180);
  const [stickSize, setStickSize] = useState(120);
  const [gamepadConnected, setGamepadConnected] = useState(false);

  const effectiveMax = maxSpeed ?? config.maxLinearSpeed;

  const cmdVel = useRef(null);
  const stopOverride = useRef(null);
  const intervalRef = useRef(null);
  const latestCoordsRef = useRef(null);
  const drivingRef = useRef(false);

  useEffect(() => {
    if (!ros || !window.ROSLIB) return;
    cmdVel.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CMD_VEL_TOPIC,
      messageType: "geometry_msgs/Twist",
    });
    stopOverride.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.STOP_TOPIC,
      messageType: "std_msgs/Bool",
    });
  }, [ros]);

  // 更新正在运行的发布流程，而不是每次调用都重启定时器。下方的游戏手柄轮询器在摇杆按住期间每帧都会调用此函数；
  // 若如此频繁地重启 100 ms 定时器，定时器将一直无法触发。
  const setDataToRos = useCallback((coordsData) => {
    latestCoordsRef.current = coordsData;
    if (!drivingRef.current) {
      drivingRef.current = true;
      stopOverride.current?.publish(new window.ROSLIB.Message({ data: false }));
    }
    if (!intervalRef.current) {
      intervalRef.current = setInterval(() => {
        if (cmdVel.current && latestCoordsRef.current) {
          cmdVel.current.publish(new window.ROSLIB.Message(latestCoordsRef.current));
        }
      }, 100);
    }
  }, []);

  const handleJoysticMove = useCallback(
    (evt) => {
      setDataToRos({
        linear: { x: evt.y * effectiveMax, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: -evt.x * config.maxAngularSpeed },
      });
    },
    [setDataToRos, effectiveMax, config.maxAngularSpeed],
  );

  const handleStop = useCallback(() => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    latestCoordsRef.current = null;
    drivingRef.current = false;
    if (cmdVel.current) {
      cmdVel.current.publish(
        new window.ROSLIB.Message({
          linear: { x: 0, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: 0 },
        }),
      );
    }
  }, []);

  // 游戏手柄支持：备用输入，沿用屏幕摇杆使用的 setDataToRos/handleStop 流程，因此会向相同的 cmd_vel topic
  // 发布数据并遵守相同的速度限制。通过 rAF 轮询（Gamepad API 不提供摇杆移动事件）；采用边沿触发，
  // 确保松开时 handleStop 只调用一次，而不是每帧都调用。
  useEffect(() => {
    const gamepadActiveRef = { current: false };
    let rafId;

    const poll = () => {
      const pads = navigator.getGamepads ? navigator.getGamepads() : [];
      const gp = pads && pads[0];
      const x = gp ? applyDeadzone(gp.axes[0] || 0) : 0;
      const y = gp ? applyDeadzone(gp.axes[1] || 0) : 0;

      if (x !== 0 || y !== 0) {
        gamepadActiveRef.current = true;
        handleJoysticMove({ x, y: -y });
      } else if (gamepadActiveRef.current) {
        gamepadActiveRef.current = false;
        handleStop();
      }
      rafId = requestAnimationFrame(poll);
    };
    rafId = requestAnimationFrame(poll);

    const onConnect = () => setGamepadConnected(true);
    const onDisconnect = () => setGamepadConnected(false);
    window.addEventListener("gamepadconnected", onConnect);
    window.addEventListener("gamepaddisconnected", onDisconnect);
    setGamepadConnected((navigator.getGamepads?.() || []).some(Boolean));

    return () => {
      cancelAnimationFrame(rafId);
      window.removeEventListener("gamepadconnected", onConnect);
      window.removeEventListener("gamepaddisconnected", onDisconnect);
    };
  }, [handleJoysticMove, handleStop]);

  useEffect(() => {
    if (!joysticRef.current) return;
    const blockWidth = joysticRef.current.getBoundingClientRect().width;
    setSize(blockWidth);
    setStickSize(blockWidth * 0.66);

    return () => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current);
      }
    };
  }, []);

  return (
    <div
      ref={joysticRef}
      className={`relative ${
        compact
          ? "h-[82px] w-[82px] lg:h-[88px] lg:w-[88px] 2xl:h-[96px] 2xl:w-[96px]"
          : "h-[100px] w-[100px] lg:h-[140px] lg:w-[140px] 2xl:h-[180px] 2xl:w-[180px]"
      }`}
    >
      <div className="absolute">
        <Joystick
          throttle={150}
          size={size}
          stickSize={stickSize}
          baseColor="#17171d"
          stickColor="#8b5cf6"
          move={handleJoysticMove}
          stop={handleStop}
        />
      </div>
      {gamepadConnected && (
        <span
          title="Gamepad connected — left stick drives"
          className="absolute -right-1 -top-1 h-2.5 w-2.5 rounded-full bg-statusGreen shadow-[0_0_0_2px_rgba(0,0,0,0.4)]"
        />
      )}
    </div>
  );
};

export default JoystickComponent;

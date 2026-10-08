import React, {
  useCallback,
  useContext,
  useEffect,
  createContext,
  useState,
  useMemo,
} from "react";

import "./configs/normalize.css";
import "../shared/styles/fonts.css";
import "../shared/styles/global.css";
import "../shared/styles/icons.css";

import { AppConfig } from "../shared/constants";
import {
  loadRuntimeConfig,
  saveRuntimeConfig,
  resolveRosbridgeHost,
} from "../shared/constants/runtimeConfig";

import withProviders from "./providers";
import Routes from "../pages";

export const RosContext = createContext(null);
export const RosStatusContext = createContext("disconnected");
export const ThemeContext = createContext({
  theme: "dark",
  toggleTheme: () => {},
});
export const RuntimeConfigContext = createContext({
  config: loadRuntimeConfig(),
  updateConfig: () => {},
});
export const AuthContext = createContext({
  mode: "open",
  identity: null,
  authReady: false,
  setIdentity: () => {},
});

// Standard way for a panel/page to reach the shared ROS connection. One ROSLIB.Ros instance is created here
// and shared app-wide; panels never create their own.
export const useRos = () => useContext(RosContext);
export const useRosStatus = () => useContext(RosStatusContext);
// Connection host/port, camera port, and speed-limit overrides editable on
// the Config page (/config) and persisted to localStorage — see
// shared/constants/runtimeConfig.js for the shape and defaults.
export const useRuntimeConfig = () => useContext(RuntimeConfigContext);

const App = () => {
  const configuredAuthMode = import.meta.env.VITE_AUTH_MODE;
  const [ros] = useState(new window.ROSLIB.Ros());
  const [status, setStatus] = useState("disconnected");
  const [authMode, setAuthMode] = useState(
    configuredAuthMode === "open" ? "open" : "unknown",
  );
  const [identity, setIdentity] = useState(null);
  const [authReady, setAuthReady] = useState(configuredAuthMode === "open");

  useEffect(() => {
    if (configuredAuthMode === "open") return undefined;
    let active = true;
    const apiBase =
      window.location.port === "3000"
        ? `http://${window.location.hostname}:5050`
        : "";
    const loadIdentity = async () => {
      try {
        const statusResponse = await fetch(`${apiBase}/api/auth/status`, {
          credentials: "include",
          cache: "no-store",
        });
        if (!statusResponse.ok) throw new Error("Authentication status unavailable");
        const auth = await statusResponse.json();
        if (!active) return;
        setAuthMode(auth.mode);
        if (auth.mode === "local") {
          const meResponse = await fetch(`${apiBase}/api/v1/auth/me`, {
            credentials: "include",
            cache: "no-store",
          });
          if (meResponse.ok) setIdentity(await meResponse.json());
        }
      } catch (error) {
        if (active) setAuthMode("unavailable");
        console.error("Unable to load authentication state", error);
      } finally {
        if (active) setAuthReady(true);
      }
    };
    loadIdentity();
    return () => {
      active = false;
    };
  }, [configuredAuthMode]);

  const [runtimeConfig, setRuntimeConfig] = useState(() => loadRuntimeConfig());

  const updateRuntimeConfig = useCallback((partial) => {
    setRuntimeConfig((prev) => {
      const next = { ...prev, ...partial };
      saveRuntimeConfig(next);
      return next;
    });
  }, []);

  const runtimeConfigValue = useMemo(
    () => ({ config: runtimeConfig, updateConfig: updateRuntimeConfig }),
    [runtimeConfig, updateRuntimeConfig],
  );

  const [theme, setTheme] = useState(
    () => localStorage.getItem("theme") || "dark",
  );

  const toggleTheme = useCallback(() => {
    setTheme((t) => {
      const next = t === "light" ? "dark" : "light";
      localStorage.setItem("theme", next);
      return next;
    });
  }, []);

  useEffect(() => {
    document.documentElement.classList.toggle("dark", theme === "dark");
    document.documentElement.style.colorScheme = theme;
  }, [theme]);

  const themeValue = useMemo(
    () => ({ theme, toggleTheme }),
    [theme, toggleTheme],
  );

  // Depend only on the two connection-relevant fields (not the whole
  // runtimeConfig object) so unrelated settings changes — e.g. a speed
  // limit — don't tear down and reopen the rosbridge connection.
  const { rosbridgeHost, rosbridgePort } = runtimeConfig;

  const tryToConnect = useCallback(async () => {
    if (!authReady || !["open", "local"].includes(authMode)) return;
    if (authMode === "local" && !identity) return;
    const host = resolveRosbridgeHost({ rosbridgeHost });
    try {
      if (authMode === "local") {
        const localPage =
          ["3000", "5050"].includes(window.location.port) &&
          ["127.0.0.1", "localhost"].includes(window.location.hostname);
        const gatewayUrl = localPage
          ? `ws://${host}:9091`
          : `${window.location.protocol === "https:" ? "wss:" : "ws:"}//${
              window.location.host
            }/rosbridge`;
        ros.connect(gatewayUrl);
      } else if (authMode === "open") {
        ros.connect("ws://" + host + ":" + rosbridgePort);
      }
    } catch (err) {
      console.log("Connecting error", err);
    }
  }, [ros, rosbridgeHost, rosbridgePort, authReady, authMode, identity]);

  useEffect(() => {
    let reconnectTimeout = null;

    const handleConnect = () => {
      setStatus("connected");
    };

    const handleClose = () => {
      setStatus("disconnected");
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      reconnectTimeout = setTimeout(tryToConnect, AppConfig.RECONNECTION_TIME);
    };

    const handleError = () => {
      setStatus("error");
      if (reconnectTimeout) clearTimeout(reconnectTimeout);
      reconnectTimeout = setTimeout(tryToConnect, AppConfig.RECONNECTION_TIME);
    };

    ros.on("connection", handleConnect);
    ros.on("close", handleClose);
    ros.on("error", handleError);

    tryToConnect();

    return () => {
      // Detach listeners before closing so this deliberate close (settings
      // changed, or the app is unmounting) doesn't trigger handleClose's
      // own reconnect-with-stale-settings scheduling.
      ros.off("connection", handleConnect);
      ros.off("close", handleClose);
      ros.off("error", handleError);
      if (reconnectTimeout) {
        clearTimeout(reconnectTimeout);
      }
      ros.close();
    };
  }, [ros, tryToConnect]);

  return (
    <RuntimeConfigContext.Provider value={runtimeConfigValue}>
      <ThemeContext.Provider value={themeValue}>
        <AuthContext.Provider
          value={{ mode: authMode, identity, authReady, setIdentity }}
        >
          <RosContext.Provider value={ros}>
            <RosStatusContext.Provider value={status}>
              <Routes />
            </RosStatusContext.Provider>
          </RosContext.Provider>
        </AuthContext.Provider>
      </ThemeContext.Provider>
    </RuntimeConfigContext.Provider>
  );
};

export default withProviders(App);

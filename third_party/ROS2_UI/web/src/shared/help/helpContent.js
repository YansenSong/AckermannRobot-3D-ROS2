/**
 * Per-page "What is this?" help content, keyed by the same absolute path
 * used in web/src/pages/registry.js. Read by HelpWidget.jsx. Pure data —
 * a contributor adding a page via registerPage() can add an entry here
 * too, but the app works fine (falls back to a generic message) if they
 * don't.
 */
export const PAGE_HELP = {
  "/": {
    title: "Map",
    summary:
      "The main operational view: the live occupancy map, the robot's position, and every control to drive or send it somewhere.",
    tips: [
      "Drag with the left mouse button to pan; scroll or pinch to zoom.",
      "Layer toggles above the map control what's drawn — laser scan, planned path, saved waypoints, zones, and the robot's trail.",
      "Choose a route saved on the Routes page under Saved Routes, then Execute Route to follow its waypoints in order.",
      "Goal Mode / Set Pose / Add Waypoint / Go Home change what a click on the map does, or send it straight to the origin.",
      "Right-click the map for a quick menu — send a goal, save a waypoint, or set the initial pose — without switching modes first.",
      "The joystick drives the robot manually at any time; the max-speed slider caps how fast, and STOP halts it and cancels any active goal.",
      "Linear and angular velocity update live next to the joystick.",
      "Dock/Undock trigger the robot's charging-dock behaviors, when supported.",
    ],
  },
  "/route": {
    title: "Routes",
    summary:
      "Create, edit, and save waypoint routes for the map currently loaded in the Ackermann simulation.",
    tips: [
      "Create a route or edit one saved for the active simulation map.",
      "Auto-plan uses the global planner to draw a path from the robot's current position through each waypoint in order.",
      "Save stores the route under the active map so it remains available after restarting the UI.",
      "Select and execute saved routes from the Map page.",
    ],
  },
  "/info": {
    title: "Status",
    summary:
      "A live operational readout: camera feed, pose/velocity telemetry, battery, and system health — the page to glance at while the robot is doing something.",
    tips: [
      "The camera panel is paused by default to save bandwidth — press Start when you need to actually see through it.",
      "The battery trend sparkline shows the last several readings, not just the instantaneous value.",
    ],
  },
  "/devices": {
    title: "Devices",
    summary:
      "A manual registry of external hardware — USB, CAN, network, and Raspberry-Pi-attached devices — with live status wherever a ROS topic is available.",
    tips: [
      "There's no plug-and-play auto-detection here (that needs OS-level access this app doesn't have) — register what's connected yourself.",
      "The detected-serial-ports list is real, though: it reads actual USB-serial devices present on the machine running the backend.",
      "A device only shows Online if its status topic is actively publishing — a device is never assumed connected just because you registered it.",
    ],
  },
  "/health": {
    title: "Health Centre",
    summary:
      'One place to answer "is the whole robot ready?" — an overall rollup built from every other page\'s live signals.',
    tips: [
      "Click any listed issue to jump straight to the page where it can actually be fixed.",
      "Ready with warnings means nothing is broken, but something (low battery, a missing topic, an offline device) is worth a look.",
      "Recent faults is a running log for this browser session only — it resets on reload, it isn't a persisted history.",
    ],
  },
  "/recordings": {
    title: "Recordings",
    summary:
      "Record real rosbag sessions and replay them later — useful for debugging, demos, lessons, and dataset collection.",
    tips: [
      "Replayed telemetry is always clearly labeled (a Replay mode banner appears on every page) — it's never shown as if it were a live robot.",
      "Stopping a replay can take several seconds — ros2 bag play needs a moment to shut down cleanly, that's expected, not a stuck button.",
      "Recording all topics is the safest default if you're not sure what you'll need later.",
    ],
  },
  "/config": {
    title: "Config",
    summary:
      "Connection settings, safety limits, and notification preferences for this browser — nothing here is shared with other operators.",
    tips: ["Changing the robot's address or port reconnects immediately."],
  },
};

export const DEFAULT_HELP = {
  title: "RobotPilot",
  summary: "No page-specific help is available here yet.",
  tips: [],
};

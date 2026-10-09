#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UI_ROOT="${PROJECT_ROOT}/third_party/RobotPilot"
WEB_DIR="${UI_ROOT}/web"
BACKEND_SCRIPT="${PROJECT_ROOT}/scripts/run_ui_backend.sh"

# This launcher is for loopback-only development. AUTH_MODE=open is rejected
# by Flask if someone changes the backend bind address to a remote interface.
export AUTH_MODE="${AUTH_MODE:-open}"
if [[ "${AUTH_MODE}" != "open" ]]; then
  echo "ERROR: the Vite development launcher supports AUTH_MODE=open only. Use the built Flask app with HTTPS for AUTH_MODE=local." >&2
  exit 1
fi
export VITE_AUTH_MODE=open
VITE_HOST="${VITE_HOST:-127.0.0.1}"
case "${VITE_HOST}" in
  127.0.0.1|localhost|::1) ;;
  *) echo "ERROR: the Vite development server must bind to loopback." >&2; exit 1 ;;
esac
# This repository's bundled development launcher targets Gazebo simulation.
# Explicit overrides remain available when using a different robot target.
export ROBOT_MODE="${ROBOT_MODE:-simulation}"
if [[ -z "${BATTERY_SOURCE:-}" ]]; then
  if [[ "${ROBOT_MODE}" == "simulation" ]]; then
    export BATTERY_SOURCE=sim
  else
    export BATTERY_SOURCE=disabled
  fi
fi

if ! command -v npm >/dev/null 2>&1; then
  echo "ERROR: npm not found. Install Node.js and npm to run the UI frontend."
  exit 1
fi

if ! command -v setsid >/dev/null 2>&1; then
  echo "ERROR: setsid not found. Install util-linux to manage UI service processes." >&2
  exit 1
fi

if [ ! -d "${WEB_DIR}/node_modules" ]; then
  echo "ERROR: frontend dependencies are missing. Run 'npm ci' in ${WEB_DIR}."
  exit 1
fi

if command -v ss >/dev/null 2>&1 && [ -n "$(ss -H -ltn '( sport = :5050 )')" ]; then
  echo "ERROR: UI backend port 5050 is already in use. Stop the previous UI launch before starting this script." >&2
  ss -ltnp '( sport = :5050 )' >&2 || true
  exit 1
fi

frontend_pid=""
backend_pid=""

cleanup() {
  local status=$?
  local pid
  trap - EXIT INT TERM

  for pid in "${frontend_pid}" "${backend_pid}"; do
    if [ -n "${pid}" ]; then
      kill -INT -- "-${pid}" 2>/dev/null || true
    fi
  done

  for pid in "${frontend_pid}" "${backend_pid}"; do
    if [ -n "${pid}" ]; then
      for _ in {1..30}; do
        if ! kill -0 -- "-${pid}" 2>/dev/null; then
          break
        fi
        sleep 0.1
      done
      if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || true
        for _ in {1..15}; do
          if ! kill -0 -- "-${pid}" 2>/dev/null; then
            break
          fi
          sleep 0.1
        done
      fi
      if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -KILL -- "-${pid}" 2>/dev/null || true
      fi
      wait "${pid}" 2>/dev/null || true
    fi
  done

  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Development auth mode: ${AUTH_MODE}."
echo "Robot mode: ${ROBOT_MODE}; battery source: ${BATTERY_SOURCE}."
echo "Starting RobotPilot backend at http://127.0.0.1:5050/ ..."
setsid bash "${BACKEND_SCRIPT}" &
backend_pid=$!

echo "Starting RobotPilot frontend at http://localhost:3000/ ..."
(
  cd "${WEB_DIR}"
  exec setsid ./node_modules/.bin/vite --host "${VITE_HOST}"
) &
frontend_pid=$!

# Keep the combined launcher alive while both services run. If either exits,
# cleanup stops the other service and returns the exited process's status.
set +e
wait -n "${backend_pid}" "${frontend_pid}"
status=$?
set -e
exit "${status}"

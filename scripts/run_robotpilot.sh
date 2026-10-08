#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UI_ROOT="${PROJECT_ROOT}/third_party/RobotPilot"
WEB_DIR="${UI_ROOT}/web"
BACKEND_SCRIPT="${UI_ROOT}/scripts/run_ui_backend.sh"

# This launcher is for local development: open mode skips the sign-in screen.
# The local auth mode remains available for later protected deployments.
export AUTH_MODE=open
export VITE_AUTH_MODE=open

if ! command -v npm >/dev/null 2>&1; then
  echo "ERROR: npm not found. Install Node.js and npm to run the UI frontend."
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
  status=$?
  trap - EXIT INT TERM

  for pid in "${frontend_pid}" "${backend_pid}"; do
    if [ -n "${pid}" ]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done

  for pid in "${frontend_pid}" "${backend_pid}"; do
    if [ -n "${pid}" ]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done

  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Development auth mode: open (login is disabled)."
echo "Starting RobotPilot backend in development open mode at http://127.0.0.1:5050/ ..."
bash "${BACKEND_SCRIPT}" &
backend_pid=$!

echo "Starting RobotPilot frontend at http://localhost:3000/ ..."
(
  cd "${WEB_DIR}"
  exec ./node_modules/.bin/vite --host 0.0.0.0
) &
frontend_pid=$!

# Keep the combined launcher alive while both services run. If either exits,
# cleanup stops the other service and returns the exited process's status.
set +e
wait -n "${backend_pid}" "${frontend_pid}"
status=$?
set -e
exit "${status}"

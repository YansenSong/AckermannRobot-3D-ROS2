#!/usr/bin/env bash
set -euo pipefail

: "${ROBOTPILOT_PLATFORM_DB:?Set ROBOTPILOT_PLATFORM_DB to the backend database path}"
: "${ROBOTPILOT_BACKUP_DIR:?Set ROBOTPILOT_BACKUP_DIR to a protected backup directory}"
ROBOTPILOT_BACKUP_KEEP="${ROBOTPILOT_BACKUP_KEEP:-30}"

if [[ -n "${ROS_SETUP:-}" ]]; then
    [[ -r "$ROS_SETUP" ]] || { echo "ROS setup not readable: $ROS_SETUP" >&2; exit 2; }
    # shellcheck disable=SC1090
    set +u
    source "$ROS_SETUP"
    set -u
fi
if [[ -n "${ROBOTPILOT_WORKSPACE_SETUP:-}" ]]; then
    [[ -r "$ROBOTPILOT_WORKSPACE_SETUP" ]] || {
        echo "Workspace setup not readable: $ROBOTPILOT_WORKSPACE_SETUP" >&2
        exit 2
    }
    # shellcheck disable=SC1090
    set +u
    source "$ROBOTPILOT_WORKSPACE_SETUP"
    set -u
fi

snapshot_database() {
    local label="$1"
    local database="$2"
    python3 -m robotpilot_ui_package.db_maintenance snapshot \
        "$database" "$ROBOTPILOT_BACKUP_DIR/$label" \
        --keep-last "$ROBOTPILOT_BACKUP_KEEP"
}

snapshot_database platform "$ROBOTPILOT_PLATFORM_DB"
if [[ -n "${ROBOTPILOT_AUTH_DB:-}" ]]; then
    snapshot_database auth "$ROBOTPILOT_AUTH_DB"
fi
if [[ -n "${ROBOTPILOT_MISSION_DB:-}" ]]; then
    snapshot_database mission "$ROBOTPILOT_MISSION_DB"
fi

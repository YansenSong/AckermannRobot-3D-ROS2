#!/bin/bash
# 手动启动 NeuPAN 节点（需要 conda neupan 环境）
# 用法: bash scripts/run_neupan.sh

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
NEUPAN_USE_SIM_TIME="${NEUPAN_USE_SIM_TIME:-false}"
VEHICLE_CONFIG="${VEHICLE_CONFIG:-$PROJECT_DIR/config/vehicle.yaml}"
ROBOT_CONFIG_DIR="$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot"
PLANNER_TEMPLATE="$ROBOT_CONFIG_DIR/planner.yaml"
RUNTIME_PLANNER="$(mktemp /tmp/ackermann_neupan_planner.XXXXXX)"

cleanup() {
    rm -f "$RUNTIME_PLANNER"
}
trap cleanup EXIT INT TERM

if [ ! -f "$VEHICLE_CONFIG" ]; then
    echo "[ERROR] vehicle config not found: $VEHICLE_CONFIG" >&2
    exit 1
fi

source "$PROJECT_DIR/install/setup.bash"

eval "$(conda shell.bash hook)"
conda activate neupan

export PYTHONPATH="$PROJECT_DIR/third_party/NeuPAN:$CONDA_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"

python3 "$PROJECT_DIR/scripts/render_neupan_planner.py" \
  --vehicle "$VEHICLE_CONFIG" \
  --template "$PLANNER_TEMPLATE" \
  --output "$RUNTIME_PLANNER"

readarray -t VEHICLE_FRAMES < <(python3 - "$VEHICLE_CONFIG" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    data = yaml.safe_load(stream) or {}
frames = data.get('vehicle', {}).get('frames', {})
print(frames.get('base', 'rear_axle_link'))
print(frames.get('lidar', 'laser_link'))
PY
)
BASE_FRAME="${VEHICLE_FRAMES[0]}"
LIDAR_FRAME="${VEHICLE_FRAMES[1]}"

echo "=== NeuPAN real vehicle ==="
echo "Python: $(which python3)"
echo "Conda env: $CONDA_DEFAULT_ENV"
echo "NeuPAN source: $PROJECT_DIR/third_party/NeuPAN"
echo "Vehicle config: $VEHICLE_CONFIG"
echo "Runtime planner: $RUNTIME_PLANNER"
echo "Base frame: $BASE_FRAME"
echo "LiDAR frame: $LIDAR_FRAME"
echo "Use sim time: $NEUPAN_USE_SIM_TIME"

python3 - <<'PY'
import neupan
import neupan_ros2

print("NeuPAN core:", neupan.__file__)
print("NeuPAN ROS2:", neupan_ros2.__file__)
PY

python3 -c "
from neupan_ros2.neupan_node import main
main()
" --ros-args \
  --params-file "$ROBOT_CONFIG_DIR/robot.yaml" \
  -p robot_config_dir:="$ROBOT_CONFIG_DIR" \
  -p planner_config_file:="$RUNTIME_PLANNER" \
  -p base_frame:="$BASE_FRAME" \
  -p lidar_frame:="$LIDAR_FRAME" \
  -p use_sim_time:="$NEUPAN_USE_SIM_TIME"

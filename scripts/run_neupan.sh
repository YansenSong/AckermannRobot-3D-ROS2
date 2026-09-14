#!/bin/bash
# 手动启动 NeuPAN 节点（需要 conda neupan 环境）
# 用法: bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
NEUPAN_USE_SIM_TIME="${NEUPAN_USE_SIM_TIME:-false}"
ROBOT_CONFIG_DIR="$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot"

source "$PROJECT_DIR/install/setup.bash"

eval "$(conda shell.bash hook)"
conda activate neupan

export PYTHONPATH="$PROJECT_DIR/third_party/NeuPAN:$CONDA_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

echo "=== NeuPAN real vehicle ==="
echo "Python: $(which python3)"
echo "Conda env: $CONDA_DEFAULT_ENV"
echo "NeuPAN source: $PROJECT_DIR/third_party/NeuPAN"
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
  -p use_sim_time:="$NEUPAN_USE_SIM_TIME"

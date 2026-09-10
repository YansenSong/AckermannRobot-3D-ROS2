#!/bin/bash
# 手动启动 NeuPAN 节点（需要 conda neupan 环境）
# 用法: bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
NEUPAN_USE_SIM_TIME="${NEUPAN_USE_SIM_TIME:-true}"
ROBOT_CONFIG_DIR="$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot"

# source ROS 2 workspace (先干这个，保证 ros2 命令可用)
source "$PROJECT_DIR/install/setup.bash"

# 激活 conda 环境 (放在 ROS 2 之后，conda Python 才能被 ros2 run 使用)
eval "$(conda shell.bash hook)"
conda activate neupan

# NeuPAN ROS2 wrapper comes from this workspace; keep the project-local
# NeuPAN core first so this task does not silently switch to another install.
export PYTHONPATH="$PROJECT_DIR/third_party/NeuPAN:$CONDA_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

echo "=== NeuPAN ==="
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

# 启动官方基线 wrapper (直接调用 main()，绕开入口脚本的 metadata 问题)
python3 -c "
from neupan_ros2.neupan_node import main
main()
" --ros-args \
  --params-file "$ROBOT_CONFIG_DIR/robot.yaml" \
  -p robot_config_dir:="$ROBOT_CONFIG_DIR" \
  -p use_sim_time:="$NEUPAN_USE_SIM_TIME"

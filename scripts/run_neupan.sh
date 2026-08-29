#!/bin/bash
# 手动启动 NeuPAN 节点（需要 conda neupan 环境）
# 用法: bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
NEUPAN_USE_SIM_TIME="${NEUPAN_USE_SIM_TIME:-true}"

# source ROS 2 workspace (先干这个，保证 ros2 命令可用)
source "$PROJECT_DIR/install/setup.bash"

# 激活 conda 环境 (放在 ROS 2 之后，conda Python 才能被 ros2 run 使用)
eval "$(conda shell.bash hook)"
conda activate neupan

# 设置 Python 路径：优先使用当前主仓库中的 NeuPAN 子模块。
# Conda 中预装的 neupan 可能是旧版本；如果它排在子模块前面，运行时
# 会绕过源码中的 Ackermann 修复（例如 min_speed 上下界）。
export PYTHONPATH="$PROJECT_DIR/src/NeuPAN:$CONDA_PREFIX/lib/python3.10/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

echo "=== NeuPAN ==="
echo "Python: $(which python3)"
echo "Conda env: $CONDA_DEFAULT_ENV"
echo "NeuPAN source: $PROJECT_DIR/src/NeuPAN"
echo "Use sim time: $NEUPAN_USE_SIM_TIME"

# 启动 NeuPAN (直接调用 main()，绕开入口脚本的 importlib.metadata 问题)
python3 -c "
from neupan_ros2.neupan_node import main
  main()
" --ros-args \
  --params-file "$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot/robot.yaml" \
  -p robot_config_dir:="$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot" \
  -p use_sim_time:="$NEUPAN_USE_SIM_TIME"

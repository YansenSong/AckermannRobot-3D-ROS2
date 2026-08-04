#!/bin/bash
# 手动启动 NeuPAN 节点（需要 conda neupan 环境）
# 用法: bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

# source ROS 2 workspace (先干这个，保证 ros2 命令可用)
source "$PROJECT_DIR/install/setup.bash"

# 激活 conda 环境 (放在 ROS 2 之后，conda Python 才能被 ros2 run 使用)
eval "$(conda shell.bash hook)"
conda activate neupan

# 设置 Python 路径 (conda 包优先)
export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:$LD_LIBRARY_PATH"

echo "=== NeuPAN (实车 real_vehicle) ==="
echo "Python: $(which python3)"
echo "Conda env: $CONDA_DEFAULT_ENV"

# 启动 NeuPAN (直接调用 main()，绕开入口脚本的 importlib.metadata 问题)
python3 -c "
from neupan_ros2.neupan_node import main
main()
" --ros-args \
  -p use_sim_time:=false \
  --params-file "$PROJECT_DIR/src/neupan_ros2/config/robots/real_vehicle/robot.yaml" \
  -p robot_config_dir:="$PROJECT_DIR/src/neupan_ros2/config/robots/real_vehicle"

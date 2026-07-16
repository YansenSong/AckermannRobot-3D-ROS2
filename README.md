# ackermann_robot_simulation

### 编译与环境设置

* **编译指令：**
    在 ackermann_robot_simulation 路径下直接执行 colcon build。
* **环境加载：**
    为了避免每次编译后都需要手动 source install/setup.bash，强烈建议将其路径添加到系统环境中。
* **编译前依赖包安装：**
sudo apt update && sudo apt install -y \
  ros-humble-topic-tools \
  ros-humble-robot-localization \
  ros-humble-cartographer* \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-twist-stamper \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ackermann-steering-controller
* **清除与重新编译：**
  删除旧的构建文件，确保全新安装：
  rm -rf build install log
  重新编译：
  colcon build
  加载环境：
  source install/setup.bash

### 重要注意事项：显卡驱动

注意检查显卡驱动是否安装并配置正确！仿真设置中调用了 GPU。如果未正确安装驱动，将导致仿真错误！

### 特殊配置说明

1.  **里程计重映射与融合：**
   ros2_control 中原始里程计输出已重映射：odom -> odom_wheel
   EKF融合后的输出话题为：/odometry/filtered

3.  **开发状态：**
   默认以里程计融合模型进行开发。

4. **_2d文件说明:**
   后缀为2D的是将3D点云压缩成2D点云进行后续处理

### 运行与演示

#### 模型预览

* **RViz 预览小车模型：**
  ros2 launch ackermann_robot review.launch.py
* **Gazebo 预览小车模型（基于 EKF 融合）：**
  ros2 launch ackermann_robot gazebo.launch.py

#### 控制

* **键盘控制：**
  ros2 launch ackermann_robot keyboard_control.launch.py

#### Nav2 导航

1.  **轮式里程计 + IMU 融合 (EKF)**
  仿真启动文件：ros2 launch ackermann_robot gazebo.launch.py
  Nav2 启动文件：ros2 launch ackermann_navigation2 nav2_2d.launch.py
  
2.  **轮式里程计 + IMU 融合 + rf2o雷达里程计 (EKF)**
  仿真启动文件：ros2 launch ackermann_robot gazebo.launch.py
  Nav2 启动文件：ros2 launch ackermann_navigation2 nav2_2d_rf2.launch.py

#### Nav2 未知环境导航建图

* **运行 Gazebo：**
    ros2 launch ackermann_robot gazebo.launch.py
* **启动 Nav2 SLAM：**
    ros2 launch ackermann_navigation2 nav2_slam_2d.launch.py

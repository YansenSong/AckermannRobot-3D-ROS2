from setuptools import find_packages, setup
from glob import glob
import os

package_name = "lpms_ig1_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="young",
    maintainer_email="young@example.com",
    description="LPMS-IG1 CANopen 16-bit SocketCAN to standard ROS 2 IMU topics",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lpms_ig1_node = lpms_ig1_ros2.lpms_ig1_node:main",
            "gyro_calibration = lpms_ig1_ros2.gyro_calibration:main",
            "accel_calibration = lpms_ig1_ros2.accel_calibration:main",
        ],
    },
)

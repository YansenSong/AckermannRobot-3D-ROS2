from setuptools import find_packages, setup

package_name = 'motion_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bridge_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='UDP bridge from /cmd_vel to STM32 26-byte protocol',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = motion_control.bridge_node:main',
        ],
    },
)

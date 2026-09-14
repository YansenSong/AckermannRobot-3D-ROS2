from setuptools import find_packages, setup

package_name = 'motion_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            ['config/bridge_params.yaml'],
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/motion_interface.launch.py'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'Ackermann command safety gate and STM32 UDP hardware interface'
    ),
    license='Apache-2.0 / BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_gate = motion_interface.command_gate:main',
            'stm32_bridge = motion_interface.stm32_bridge:main',
        ],
    },
)

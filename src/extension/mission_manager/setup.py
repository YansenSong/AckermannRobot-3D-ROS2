from setuptools import setup


setup(
    name="ackermann_mission",
    version="0.1.0",
    packages=["ackermann_mission"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ackermann_mission"]),
        ("share/ackermann_mission", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="young",
    maintainer_email="young@example.com",
    description="Robot-side persistent mission execution and status",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission_manager = ackermann_mission.node:main",
        ],
    },
)

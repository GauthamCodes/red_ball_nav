from setuptools import setup
import os
from glob import glob

package_name = "red_ball_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        # Required by ament
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Color-based object search and Nav2 navigation for TurtleBot3",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Entry point: ros2 run red_ball_nav color_search_node
            "color_search_node = red_ball_nav.color_search_node:main",
        ],
    },
)

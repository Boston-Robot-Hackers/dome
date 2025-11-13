#!/usr/bin/env python3
# This is my version of launching the slam launch file from linorobot2

import os

from better_launch import BetterLaunch, launch_this

os.environ["RCUTILS_LOGGING_MIN_SEVERITY"] = "WARN"  # Only display WARN, ERROR, FATAL


@launch_this(ui=True)
def start_nav(rviz_arg: bool = False):
    bl = BetterLaunch()

    bl.include(
        "linorobot2_navigation",
        "slam.launch.py",
        rviz=rviz_arg,
        sim=False,
        slam_config="/home/pitosalas/ros2_ws/src/dome/config/slam.yaml",
        nav_config="/home/pitosalas/ros2_ws/src/dome/config/navigation.yaml",
    )
    bl.logger.info("***********")

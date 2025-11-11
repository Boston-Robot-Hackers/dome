#!/usr/bin/env python3
import os

from better_launch import BetterLaunch, launch_this

os.environ["RCUTILS_LOGGING_MIN_SEVERITY"] = "WARN"  # or INFO, WARN, ERROR, FATAL

# This is my version of launching the slam launch file from linorobot2


@launch_this(ui=False)
def start_nav():
    bl = BetterLaunch()

    bl.include(
        "linorobot2_navigation",
        "slam.launch.py",
        rviz=True,
        sim=False,
        config="/home/pitosalas/ros2_ws/src/dome/config/slam.yaml",
    )
    bl.logger.info("***********")

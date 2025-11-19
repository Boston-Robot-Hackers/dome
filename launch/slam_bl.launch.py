#!/usr/bin/env python3
# This is my version of launching the slam launch file from linorobot2

import os

from ament_index_python.packages import get_package_share_directory
from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def start_nav(rviz_arg: bool = False):
    bl = BetterLaunch()
    dome_path = get_package_share_directory("dome")

    slam_config = os.path.join(dome_path, "config", "slam.yaml")

    nav_config = os.path.join(dome_path, "config", "navigation.yaml")

    bl.include(
        "linorobot2_navigation",
        "slam.launch.py",
        rviz=rviz_arg,
        sim=False,
        slam_config=slam_config,
        nav_config=nav_config,
    )
    bl.logger.info("***********")

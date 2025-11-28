#!/usr/bin/env python3
# This is my version of launching the slam launch file from linorobot2

from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def start_nav(rviz_arg: bool = False):
    bl = BetterLaunch()
    slam_config = bl.find("dome", "slam.yaml")
    nav_config = bl.find("dome", "navigation.yaml")

    bl.include(
        "linorobot2_navigation",
        "slam.launch.py",
        rviz=rviz_arg,
        sim=False,
        slam_config=slam_config,
        nav_config=nav_config,
    )
    bl.logger.info("***********")

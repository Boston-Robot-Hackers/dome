#!/usr/bin/env python3
# robot_brain.launch.py — perception, behavior, speech, and voice stack
# Author: Pito Salas and Claude Code
# Open Source Under MIT license
from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def robot_brain_launch(
    load_map: bool = False,
    spin_survey: bool = False,
):
    bl = BetterLaunch()

    bl.include(
        "oak_roboflow_ros",
        "robot.launch.py",
        load_map=load_map,
        spin_survey=spin_survey,
    )

    bl.include(
        "dome_control",
        "robot.launch.py",
    )

    bl.include(
        "robot_voice",
        "robot.launch.py",
    )

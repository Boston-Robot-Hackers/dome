#!/usr/bin/env python3
# This is my version of launching the slam launch file from linorobot2


from better_launch import BetterLaunch, launch_this


@launch_this(ui=True)
def start_nav(rviz_arg: bool = False):
    bl = BetterLaunch()
    urdf_path = bl.find("dome", "dome1.urdf.xacro")
    ekf_config_path = bl.find("dome", "ekf.yaml")
    bl.include(
        "linorobot2_bringup",
        "bringup.launch.py",
        rviz=rviz_arg,
        sim=False,
        base_serial_port="/dev/esp32",
        micro_ros_baudrate="921600",
        madgwick="false",
        orientation_stddev="0.01",
        joy="false",
        urdf=urdf_path,
        ekf_config_path=ekf_config_path,
    )
    bl.logger.info("***********")

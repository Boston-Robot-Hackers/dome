from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to your URDF/Xacro file
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("dome"), "config", "dome1.urdf.xacro"]
    )

    # Robot State Publisher with robot_description parameter
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        PathJoinSubstitution([FindExecutable(name="xacro")]),
                        " ",
                        urdf_file,
                    ]
                )
            }
        ],
    )

    # Include linorobot2 bringup
    linorobot2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("linorobot2_bringup"),
                        "launch",
                        "bringup.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "base_serial_port": "/dev/esp32",
            "micro_ros_baudrate": "921600",
            "madgwick": "true",
            "orientation_stddev": "0.01",
            "madgwick_imu_topic": "/imu/corrected_data",
            "joy": "false",
            "urdf": urdf_file,
        }.items(),
    )

    # Include imu_apply
    imu_apply = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("dome"), "launch", "imu_apply.launch.py"]
                )
            ]
        )
    )

    return LaunchDescription([linorobot2_bringup, imu_apply, robot_state_publisher])

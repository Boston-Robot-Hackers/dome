from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Create launch configuration variables
    calib_file = LaunchConfiguration(
        "calib_file", default="/home/pitosalas/imu_calibration.yaml"
    )
    calibrate_gyros = LaunchConfiguration("calibrate_gyros", default="false")
    gyro_calib_samples = LaunchConfiguration("gyro_calib_samples", default="200")
    raw_topic = LaunchConfiguration("raw_topic", default="/imu/data_raw")
    corrected_topic = LaunchConfiguration(
        "corrected_topic", default="/imu/corrected_data"
    )

    # Declare the launch arguments
    declare_calib_file_cmd = DeclareLaunchArgument(
        "calib_file",
        default_value="/home/pitosalas/imu_calibration.yaml",
        description="Path to the IMU calibration YAML file",
    )

    declare_calibrate_gyros_cmd = DeclareLaunchArgument(
        "calibrate_gyros",
        default_value="false",
        description="Whether to calibrate gyros",
    )

    declare_gyro_calib_samples_cmd = DeclareLaunchArgument(
        "gyro_calib_samples",
        default_value="200",
        description="Number of samples to use for gyro calibration",
    )

    declare_raw_topic_cmd = DeclareLaunchArgument(
        "raw_topic", default_value="/imu/data", description="Topic for raw IMU data"
    )

    declare_corrected_topic_cmd = DeclareLaunchArgument(
        "corrected_topic",
        default_value="/imu/corrected_data",
        description="Topic for corrected IMU data",
    )

    # Create the node
    imu_calib_node = Node(
        package="imu_calib",
        executable="apply_calib_node",
        name="apply_calib_node",
        parameters=[
            {
                "calib_file": calib_file,
                "calibrate_gyros": calibrate_gyros,
                "gyro_calib_samples": gyro_calib_samples,
            }
        ],
        remappings=[("raw", raw_topic), ("corrected", corrected_topic)],
    )

    # Return the launch description
    return LaunchDescription(
        [
            declare_calib_file_cmd,
            declare_calibrate_gyros_cmd,
            declare_gyro_calib_samples_cmd,
            declare_raw_topic_cmd,
            declare_corrected_topic_cmd,
            imu_calib_node,
        ]
    )

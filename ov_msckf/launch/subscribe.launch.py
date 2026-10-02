from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="ov_enable", default_value="true", description="enable OpenVINS node"
    ),
    DeclareLaunchArgument(
        name="rviz_enable", default_value="false", description="enable rviz node"
    ),
    DeclareLaunchArgument(
        name="bag_play", default_value="true", description="play the EuRoC ROS 2 bag"
    ),
    DeclareLaunchArgument(
        name="bag_path",
        default_value="/home/he/datasets/euroc/V1_01_easy_db",
        description="directory containing the EuRoC ROS 2 bag metadata.yaml",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="EuRoC ASL groundtruth CSV; auto-detected from bag_path when empty",
    ),
    DeclareLaunchArgument(
        name="dosave", default_value="false", description="save estimated poses to a text file"
    ),
    DeclareLaunchArgument(
        name="path_est",
        default_value="/output/traj_estimate.txt",
        description="estimated pose output file when dosave is true",
    ),
    DeclareLaunchArgument(
        name="config",
        default_value="euroc_mav",
        description="euroc_mav, tum_vi, rpng_aruco...",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined based on provided 'config' above",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we have more than 1 camera, if we should try to track stereo constraints between pairs",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular (all mono tracking)",
    ),
    DeclareLaunchArgument(
        name="save_total_state",
        default_value="false",
        description="record the total state with calibration and features to a txt file",
    )
]

def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    config = LaunchConfiguration("config").perform(context)
    if not config_path:
        configs_dir = os.path.join(get_package_share_directory("ov_msckf"), "config")
        available_configs = os.listdir(configs_dir)
        if config in available_configs:
            config_path = os.path.join(
                            get_package_share_directory("ov_msckf"),
                            "config",config,"estimator_config.yaml"
                        )
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    else:
        if not os.path.isfile(config_path):
            return [
                LogInfo(
                    msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                        config_path)
                )
            ]
    path_gt = LaunchConfiguration("path_gt").perform(context)
    if not path_gt and config == "euroc_mav":
        bag_path = os.path.normpath(LaunchConfiguration("bag_path").perform(context))
        sequence = os.path.basename(bag_path)
        if sequence.endswith("_db"):
            sequence = sequence[:-3]
        candidate = os.path.join(
            os.path.dirname(bag_path), "ASL", sequence,
            "mav0", "state_groundtruth_estimate0", "data.csv",
        )
        if os.path.isfile(candidate):
            path_gt = candidate
    if path_gt and not os.path.isfile(path_gt):
        raise RuntimeError("Groundtruth CSV does not exist: {}".format(path_gt))

    node_parameters = [
        {"verbosity": LaunchConfiguration("verbosity")},
        {"use_stereo": LaunchConfiguration("use_stereo")},
        {"max_cameras": LaunchConfiguration("max_cameras")},
        {"save_total_state": LaunchConfiguration("save_total_state")},
        {"dosave": LaunchConfiguration("dosave")},
        {"path_est": LaunchConfiguration("path_est")},
        {"config_path": config_path},
    ]
    if path_gt:
        node_parameters.append({"path_gt": path_gt})

    node1 = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output='screen',
        parameters=node_parameters,
    )

    node2 = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"
            ),
            "--ros-args",
            "--log-level",
            "warn",
            ],
    )

    bag_player = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", LaunchConfiguration("bag_path"),
            "--topics", "/imu0", "/cam0/image_raw", "/cam1/image_raw",
            "--disable-keyboard-controls",
        ],
        output="screen",
    )
    delayed_bag_player = TimerAction(
        period=3.0,
        actions=[bag_player],
        condition=IfCondition(LaunchConfiguration("bag_play")),
    )

    return [node1, node2, delayed_bag_player]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld

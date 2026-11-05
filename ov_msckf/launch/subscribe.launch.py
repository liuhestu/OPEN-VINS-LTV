from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import math
import os

launch_args = [
    # Estimator configuration
    DeclareLaunchArgument(
        "config",
        default_value="euroc_ltv",
        description="configuration directory containing estimator_config.yaml and optional ltv.yaml",
    ),
    DeclareLaunchArgument(
        "config_path",
        default_value="",
        description="explicit estimator_config.yaml; otherwise use config",
    ),

    # LTV mode and scene presets
    DeclareLaunchArgument(
        "ltv_scene",
        default_value="auto",
        description="infer MH/Vicon during replay; otherwise use YAML scene",
    ),
    DeclareLaunchArgument("ltv_mode", default_value="", description="override LTV mode: OFF/G/V/L/GVL"),

    # Node namespace and RViz
    DeclareLaunchArgument("namespace", default_value="ov_msckf"),
    DeclareLaunchArgument("rviz", default_value="true", description="start RViz"),

    # Camera setup
    DeclareLaunchArgument("max_cameras", default_value="2"),
    DeclareLaunchArgument("use_stereo", default_value="true"),

    # Dataset and ROS 2 bag playback
    DeclareLaunchArgument("dataset", default_value="V1_01_easy"),
    DeclareLaunchArgument(
        "dobag",
        default_value="true",
        description="play the ROS 2 bag; false subscribes to live topics",
    ),
    DeclareLaunchArgument(
        "bag",
        default_value="",
        description="bag directory; empty uses /home/he/datasets/euroc/<dataset>_db",
    ),
    DeclareLaunchArgument(
        "bag_start",
        default_value="0",
        description="skip this many seconds from the beginning of the bag",
    ),
    DeclareLaunchArgument("bag_rate", default_value="1", description="positive playback rate"),

    # Estimated trajectory and timing output
    DeclareLaunchArgument("dosave", default_value="false", description="start ov_eval/pose_to_file"),
    DeclareLaunchArgument("path_est", default_value="/results/traj_estimate.txt"),
    DeclareLaunchArgument("dotime", default_value="false", description="record estimator timing"),
    DeclareLaunchArgument("path_time", default_value="/results/traj_timing.txt"),

    # Groundtruth trajectory alignment and display
    DeclareLaunchArgument("dolivetraj", default_value="true", description="start ov_eval/live_align_trajectory"),
    DeclareLaunchArgument(
        "alignment_type",
        default_value="posyaw",
        choices=["posyaw", "posyawsingle", "se3", "se3single", "sim3", "none"],
    ),
    DeclareLaunchArgument(
        "path_gt",
        default_value="",
        description="GT CSV or TXT; empty finds EuRoC ASL GT beside the bag",
    ),

    # Full estimator state output
    DeclareLaunchArgument("save_total_state", default_value="false"),
]


def resolve_inputs(config, config_path, ltv_mode, ltv_scene, bag, dobag, share_dir):
    config = config or "euroc_ltv"
    if not config_path:
        config_path = os.path.join(share_dir, "config", config, "estimator_config.yaml")
    if not os.path.isfile(config_path):
        raise RuntimeError("Estimator config does not exist: " + config_path)
    candidate = os.path.join(os.path.dirname(config_path), "ltv.yaml")
    ltv_config = candidate if os.path.isfile(candidate) else ""
    if ltv_mode not in ("", "OFF") and not ltv_config:
        raise RuntimeError("Requested LTV mode requires ltv.yaml beside estimator_config.yaml: " + candidate)
    if ltv_scene == "auto":
        ltv_scene = ""
        if dobag:
            sequence = os.path.basename(os.path.normpath(bag))
            if sequence.startswith("MH_"):
                ltv_scene = "MH"
            elif sequence.startswith(("V1_", "V2_")):
                ltv_scene = "Vicon"
    return config, config_path, ltv_config, ltv_scene


def launch_setup(context):
    # Read launch arguments and resolve the default bag path.
    value = lambda key: LaunchConfiguration(key).perform(context)
    bag = value("bag") or os.path.join("/home/he/datasets/euroc", value("dataset") + "_db")
    dobag = value("dobag").lower() in ("true", "1")
    dolivetraj = value("dolivetraj").lower() in ("true", "1")
    bag_start, bag_rate = float(value("bag_start")), float(value("bag_rate"))

    # Validate playback settings and bag metadata.
    if not math.isfinite(bag_start) or bag_start < 0 or not math.isfinite(bag_rate) or bag_rate <= 0:
        raise RuntimeError("bag_start must be finite and nonnegative; bag_rate must be finite and positive")
    if dobag and not os.path.isfile(os.path.join(bag, "metadata.yaml")):
        raise RuntimeError("ROS 2 bag metadata does not exist: " + bag)

    # Resolve estimator configuration and LTV settings.
    config, config_path, ltv_config, ltv_scene = resolve_inputs(
        value("config"),
        value("config_path"),
        value("ltv_mode"),
        value("ltv_scene"),
        bag,
        dobag,
        get_package_share_directory("ov_msckf"),
    )

    # Resolve and validate the groundtruth trajectory.
    path_gt = value("path_gt")
    if dolivetraj:
        if not path_gt and config in ("euroc_mav", "euroc_ltv"):
            sequence = os.path.basename(os.path.normpath(bag))
            if sequence.endswith("_db"):
                sequence = sequence[:-3]
            path_gt = os.path.join(
                os.path.dirname(bag),
                "ASL",
                sequence,
                "mav0",
                "state_groundtruth_estimate0",
                "data.csv",
            )
        if not path_gt or not os.path.isfile(path_gt):
            raise RuntimeError("dolivetraj requires a valid groundtruth CSV or TXT: " + path_gt)

    # Prepare the topic namespace and typed node parameters.
    topic_root = "/" + value("namespace").strip("/") if value("namespace").strip("/") else ""
    parameter = lambda key, kind: ParameterValue(LaunchConfiguration(key), value_type=kind)

    # Estimator node.
    estimator = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace=value("namespace"),
        output="screen",
        parameters=[{
            "config_path": config_path,
            "use_stereo": parameter("use_stereo", bool),
            "max_cameras": parameter("max_cameras", int),
            "record_timing_information": parameter("dotime", bool),
            "record_timing_filepath": parameter("path_time", str),
            "save_total_state": parameter("save_total_state", bool),
            "ltv_config": ltv_config,
            "ltv_mode": parameter("ltv_mode", str),
            "ltv_scene": ltv_scene,
        }],
    )

    # Estimated trajectory recorder.
    recorder = Node(
        package="ov_eval",
        executable="pose_to_file",
        name="recorder_estimate",
        namespace=value("namespace"),
        condition=IfCondition(LaunchConfiguration("dosave")),
        output="screen",
        parameters=[{
            "topic": topic_root + "/poseimu",
            "topic_type": "PoseWithCovarianceStamped",
            "output": parameter("path_est", str),
        }],
    )

    # Groundtruth trajectory alignment.
    aligner = Node(
        package="ov_eval",
        executable="live_align_trajectory",
        name="live_align_trajectory",
        namespace=value("namespace"),
        condition=IfCondition(LaunchConfiguration("dolivetraj")),
        output="log",
        parameters=[{
            "alignment_type": parameter("alignment_type", str),
            "path_gt": path_gt,
        }],
        remappings=[
            ("/ov_msckf/pathimu", topic_root + "/pathimu"),
            ("/ov_msckf/pathgt", topic_root + "/pathgt"),
        ],
    )

    # RViz visualization.
    rviz_topics = [
        "pathimu",
        "pathgt",
        "poseimu",
        "odomimu",
        "trackhist",
        "points_msckf",
        "points_slam",
        "points_aruco",
        "points_sim",
        "loop_depth_colored",
        "loop_feats",
    ]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz")),
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        remappings=[("/ov_msckf/" + topic, topic_root + "/" + topic) for topic in rviz_topics],
    )

    # Delayed bag playback.
    player = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", bag,
            "--start-offset", str(bag_start),
            "--rate", str(bag_rate),
            "--topics", "/imu0", "/cam0/image_raw", "/cam1/image_raw",
            "--disable-keyboard-controls",
        ],
        output="screen",
    )
    delayed_player = TimerAction(
        period=3.0,
        actions=[player],
        condition=IfCondition(LaunchConfiguration("dobag")),
    )

    return [estimator, recorder, aligner, rviz_node, delayed_player]


def generate_launch_description():
    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])

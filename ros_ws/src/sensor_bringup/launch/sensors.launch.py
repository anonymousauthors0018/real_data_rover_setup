from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import TimerAction, RegisterEventHandler, OpaqueFunction
from launch.events import Shutdown

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import EmitEvent

from datetime import datetime

def generate_launch_description():

    current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

    entered_object_name = input("Enter object name: ").strip()
    if not entered_object_name:
        entered_object_name = "unknown_object"

    entered_duration = input("Enter recording duration in seconds: ").strip()
    if not entered_duration:
        entered_duration = "15.0"

    session_name = LaunchConfiguration("session_name")
    base_dir = LaunchConfiguration("base_dir")
    object_name = LaunchConfiguration("object_name")
    record_duration_sec = LaunchConfiguration("record_duration_sec")
    # ---------- Launch args ----------
    camera_name = LaunchConfiguration("camera_name")
    enable_color = LaunchConfiguration("enable_color")
    enable_depth = LaunchConfiguration("enable_depth")
    align_depth = LaunchConfiguration("align_depth")

    # Radar node params
    out_root = LaunchConfiguration("radar_out_root")
    dca_cfg = LaunchConfiguration("dca_config_file")
    radar_cfg = LaunchConfiguration("radar_config_file")
    cli_port = LaunchConfiguration("cli_port")
    data_port = LaunchConfiguration("data_port")
    numframes = LaunchConfiguration("numframes_per_read")
    framebuf = LaunchConfiguration("frame_num_in_buf")

    # Sync params
    slop = LaunchConfiguration("slop_sec")
    queue_size = LaunchConfiguration("queue_size")

    # Rosbag params
    record = LaunchConfiguration("record")
    bag_dir = LaunchConfiguration("bag_dir")
    bag_name = LaunchConfiguration("bag_name")

    # ---------- RealSense include ----------
    realsense_share = get_package_share_directory("realsense2_camera")
    rs_launch = PathJoinSubstitution([realsense_share, "launch", "rs_launch.py"])

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            "camera_name": camera_name,
            "enable_color": enable_color,
            "rgb_camera.profile": "640,480,30",
            "enable_depth": enable_depth,
            "depth_module.profile": "640,480,30",
            # "align_depth.enable": align_depth,
        #     # Optional: enable_sync can help align RS streams internally
            "enable_sync": "true",
        }.items(),
    )

    set_rs_params = ExecuteProcess(
    cmd=[
        "bash", "-lc",
        "until ros2 node list | grep -q '^/camera/camera$'; do sleep 0.5; done; "
        "until ros2 param list /camera/camera | grep -q 'rgb_camera.auto_exposure_priority'; do sleep 0.5; done; "
        "ros2 param set /camera/camera rgb_camera.auto_exposure_priority false"
    ],
    output="screen",
    )

    # ---------- Radar capture node (from ti_dca_node) ----------
    radar = Node(
        package="ti_dca_node",
        executable="radar_capture_node",
        name="radar_capture_node",
        output="screen",
        parameters=[
            {"out_root": PathJoinSubstitution([base_dir, session_name, "radar_data"])},
            {'proc_mode': 'none'},  # Use ROS time and topics for radar capture node
            {'numframes_per_read':1},
            {"dca_config_file": dca_cfg},
            {"radar_config_file": radar_cfg},
            {"cli_port": cli_port},
            {"data_port": data_port},
        #     {"numframes_per_read": numframes},
            {"frame_num_in_buf": 32},
        ],
    )

    # ---------- Sync node (from ti_dca_node) ----------
    sync = Node(
        package="ti_dca_node",
        executable="radar_sync_node",
        name="radar_sync_node",
        output="screen",
        parameters=[
            {"rgb_topic": "/camera/color/image_raw"},
            {"depth_topic": "/camera/depth/image_rect_raw"},
            {"radar_ref_topic": "/radar/frame_ref"},
            {"slop_sec": slop},
            {"queue_size": queue_size},
        ],
    )


    bag_record = ExecuteProcess(
    cmd=[
        "bash", "-lc",
        "if [ \"${RECORD}\" = \"true\" ]; then "
        "mkdir -p \"${SESSION_DIR}\"; "
        "cd \"${SESSION_DIR}\"; "
        "ros2 bag record -o rosbag "
        "/camera/color/image_raw "
        "/camera/color/camera_info "
        "/camera/depth/image_rect_raw "
        "/camera/depth/camera_info "
        "/radar/frame_ref "
        "/synced/pair & "
        "BAG_PID=$!; "
        f"sleep 2.5; "
        f"echo 'Bag live. Recording for {entered_duration}s'; "
        f"sleep {entered_duration}; "
        "echo 'Sending SIGINT to bag PID '$BAG_PID; "
        "kill -INT $BAG_PID; "
        "wait $BAG_PID; "
        "echo 'Bag finalized.'; "
        "fi"
    ],
    additional_env={
        "RECORD":      record,
        "SESSION_DIR": PathJoinSubstitution([base_dir, session_name]),
    },
    output="screen",
    )

    auto_start = LaunchConfiguration("auto_start")
    start_delay = LaunchConfiguration("start_delay_sec")

    radar_auto = ExecuteProcess(
        cmd=[
            "bash", "-lc",
            "if [ \"${AUTO_START}\" = \"true\" ]; then "
            "echo 'Waiting for /radar/configure...'; "
            "until ros2 service list | grep -q '/radar/configure'; do sleep 0.2; done; "
            "echo 'Calling /radar/configure'; "
            "ros2 service call /radar/configure std_srvs/srv/Trigger '{}' ; "
            "echo 'Calling /radar/start'; "
            "ros2 service call /radar/start std_srvs/srv/Trigger '{}' ; "
            "fi"
        ],
        additional_env={"AUTO_START": auto_start},
        output="screen",
    )

    # Run it after a delay so DDS discovery finishes
    radar_auto_timer = TimerAction(
        period=start_delay,
        actions=[radar_auto]
    )

    start_bag_after_radar = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=radar_auto,
            on_exit=[bag_record],
        )
    )

    shutdown_after_bag = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=bag_record,
        on_exit=[EmitEvent(event=Shutdown(reason="Recording complete"))],
    )
    )

    


    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("enable_color", default_value="true"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("align_depth", default_value="true"),

        DeclareLaunchArgument("base_dir", default_value="/mnt/ssd/radar_data"),
        DeclareLaunchArgument("object_name", default_value=entered_object_name),
        DeclareLaunchArgument("record_duration_sec", default_value=entered_duration),

        DeclareLaunchArgument("dca_config_file", default_value="/workspace/pyRadar/configFiles/cf.json"),
        DeclareLaunchArgument("radar_config_file", default_value="/workspace/pyRadar/configFiles/xWR1843Boost_config_matlab.cfg"),
        DeclareLaunchArgument("cli_port", default_value="/dev/ti_radar_0"),
        DeclareLaunchArgument("data_port", default_value="/dev/ti_radar_1"),
        DeclareLaunchArgument("numframes_per_read", default_value="1"),
        DeclareLaunchArgument("frame_num_in_buf", default_value="64"),

        DeclareLaunchArgument("slop_sec", default_value="0.02"),
        DeclareLaunchArgument("queue_size", default_value="10"),

        DeclareLaunchArgument("record", default_value="true"),

        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("start_delay_sec", default_value="3.0"),

        DeclareLaunchArgument("session_name",default_value=[current_time, "_", LaunchConfiguration("object_name")]),
        DeclareLaunchArgument("bag_delay_sec", default_value="6.0"),

        realsense,
        set_rs_params,
        radar,
        sync,
        # bag_record,
        radar_auto_timer,
        start_bag_after_radar,
        shutdown_after_bag,
        
    ])
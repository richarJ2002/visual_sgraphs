from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)


def generate_launch_description():
    return LaunchDescription(
        [
            # ---------------------------------------------------------------- #
            # Global arguments declarations
            # ---------------------------------------------------------------- #

            DeclareLaunchArgument(
                "offline",
                default_value="true"
            ),

            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use timestamps published on /clock.",
            ),

            DeclareLaunchArgument(
                "launch_rviz",
                default_value="true"
            ),

            DeclareLaunchArgument(
                "colored_pointcloud",
                default_value="true"
            ),

            DeclareLaunchArgument(
                "visualize_segmented_scene",
                default_value="true"
            ),

            DeclareLaunchArgument(
                "semantic_scene_segmenter",
                default_value="yoso",
                description=("The method to segment the semantic scene "
                             "(if off, the baseline)"),
                choices=[
                    "yoso",
                    "pfcn",
                    "yolo26",
                    "off"
                ],
            ),

            # ---------------------------------------------------------------- #
            # Topics
            # ---------------------------------------------------------------- #

            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera"
            ),

            DeclareLaunchArgument(
                "sensor_config",
                default_value="SMapper_RealSense"
            ),

            DeclareLaunchArgument(
                "rgb_image_topic",
                default_value="/camera/realsense/color/image_raw"
            ),

            DeclareLaunchArgument(
                "rgb_camera_info_topic",
                default_value="/camera/realsense/color/camera_info",
            ),

            DeclareLaunchArgument(
                "depth_image_topic",
                default_value="/camera/realsense/aligned_depth_to_color/image_raw",
            ),

            DeclareLaunchArgument(
                "pointcloud_topic",
                default_value="/camera/depth/points",
            ),

            DeclareLaunchArgument(
                "generate_pointcloud",
                default_value="true",
                description=(
                    "Generate a registered cloud when the sensor does not "
                    "publish one."
                ),
            ),

            DeclareLaunchArgument(
                "direct_gazebo_flu_cloud",
                default_value=PythonExpression([
                    "'true' if '",
                    LaunchConfiguration("sensor_config"),
                    "' == 'Alpha_Gazebo' and '",
                    LaunchConfiguration("generate_pointcloud"),
                    "' == 'false' else 'false'",
                ]),
                description=(
                    "Convert a raw Gazebo FLU cloud to the optical camera "
                    "frame. Alpha_Gazebo raw clouds enable this by default."
                ),
            ),

            DeclareLaunchArgument(
                "imu_topic",
                default_value="/camera/realsense/imu",
            ),

            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description=(
                    "Verbosity for core/ROS logging: quiet, error, warn, info "
                    "or debug. Sets both the RCLCPP logger threshold and the "
                    "core Verbose threshold via the log_level parameter."
                ),
            ),

            DeclareLaunchArgument(
                "test_run_dir",
                default_value="",
                description=(
                    "Absolute root of the test-run folder "
                    "(test_runs/<YYYY-MM-DD-HH-MM-SS>[_<name>]); empty "
                    "writes nothing. Each snapshot writes "
                    "archive/sgraph/sgraph_<sec>_<nsec>.json, "
                    "output/sgraph_latest.json, "
                    "archive/trajectory/estimated_<sec>_<nsec>.csv "
                    "(keyframe camera poses) and "
                    "archive/map_points/map_points_<sec>_<nsec>.csv "
                    "(map points on a 0.1 m voxel grid); about once a "
                    "second the tracking image goes to archive/loccams/."
                ),
            ),

            DeclareLaunchArgument(
                "sgraph_archive_enabled",
                default_value="true",
                description=(
                    "Master on/off switch for everything written below "
                    "test_run_dir (snapshots, CSVs and loccams)."
                ),
            ),

            DeclareLaunchArgument(
                "sgraph_archive_interval_sec",
                default_value="5.0",
                description=(
                    "Minimum sim-clock seconds between snapshots (S-Graph "
                    "JSON, trajectory and map-point CSVs)."
                ),
            ),

            DeclareLaunchArgument(
                "sgraph_archive_max_files",
                default_value="0",
                description=(
                    "Maximum retained files per snapshot series (S-Graph "
                    "JSON, trajectory CSV, map-point CSV); 0 keeps all."
                ),
            ),

            # ---------------------------------------------------------------- #
            # VS-Graphs Node
            # ---------------------------------------------------------------- #

            Node(
                name="vs_graphs",
                package="vs_graphs",
                executable="ros_rgbd_inertial",
                output="both",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time")
                    },
                    {
                        "log_level": LaunchConfiguration("log_level")
                    },
                    {
                        "voc_file": LaunchConfiguration(
                            "voc_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/vocabulary/ORBvoc.txt.bin",
                            ],
                        )
                    },
                    {
                        "settings_file": LaunchConfiguration(
                            "settings_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/RGB-D-Inertial/",
                                LaunchConfiguration("sensor_config"),
                                ".yaml",
                            ],
                        )
                    },
                    {
                        "sys_params_file": LaunchConfiguration(
                            "sys_params_file",
                            default=[
                                get_package_share_directory("vs_graphs"),
                                "/config/system_params.yaml",
                            ],
                        )
                    },
                    {
                        "roll": 1.5707963268
                    },
                    {
                        "pitch": 0.0
                    },
                    {
                        "yaw": 0.0
                    },
                    {
                        "frame_map": "map"
                    },
                    {
                        "frame_imu": "imu"
                    },
                    {
                        "frame_world": "world"
                    },
                    {
                        "frame_camera": "camera"
                    },
                    {
                        "static_transform": True
                    },
                    {
                        "colored_pointcloud": False
                    },
                    {
                        "publish_pointclouds": True
                    },
                    {
                        "maximum_tracking_rate_hz": 30.0
                    },
                    {
                        "maximum_sensor_buffer_seconds": 3.0
                    },
                    {
                        "direct_gazebo_flu_cloud": LaunchConfiguration(
                            "direct_gazebo_flu_cloud"
                        )
                    },
                    {
                        "test_run_dir": LaunchConfiguration("test_run_dir")
                    },
                    {
                        "sgraph_archive_enabled": LaunchConfiguration(
                            "sgraph_archive_enabled"
                        )
                    },
                    {
                        "sgraph_archive_interval_sec": LaunchConfiguration(
                            "sgraph_archive_interval_sec"
                        )
                    },
                    {
                        "sgraph_archive_max_files": LaunchConfiguration(
                            "sgraph_archive_max_files"
                        )
                    },
                ],
                remappings=[
                    (
                        "/imu",
                        LaunchConfiguration("imu_topic")
                    ),
                    (
                        "/camera/rgb/image_raw",
                        LaunchConfiguration("rgb_image_topic")
                    ),
                    (
                        "/camera/depth_registered/image_raw",
                        LaunchConfiguration("depth_image_topic"),
                    ),
                    (
                        "/camera/depth/points",
                        LaunchConfiguration("pointcloud_topic"),
                    ),
                ],
            ),

            # ---------------------------------------------------------------- #
            # Static Transforms
            # ---------------------------------------------------------------- #

            Node(
                package="tf2_ros",
                name="map_to_map_elevated",  # For Voxblox Skeleton
                executable="static_transform_publisher",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
                arguments=[
                    "--frame-id", "map",
                    "--child-frame-id", "map_elevated",
                ],
            ),

            Node(
                name="bc_to_se",
                package="tf2_ros",
                executable="static_transform_publisher",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
                arguments=[
                    "--z", "3",
                    "--frame-id", "build_comp",
                    "--child-frame-id", "struc_elem",
                ],
            ),

            Node(
                package="tf2_ros",
                name="world_to_bc",
                executable="static_transform_publisher",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
                arguments=[
                    "--z", "3",
                    "--frame-id", "world",
                    "--child-frame-id", "build_comp",
                ],
            ),

            Node(
                package="tf2_ros",
                name="camera_to_imu",
                executable="static_transform_publisher",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
                # Intel RealSense D435i IMU (BMI055) pose in the colour optical
                # frame "camera" (right-down-forward), metres. This is the
                # inverse of the YAML IMU.T_b_c1 (camera pose in the IMU
                # frame), so both come from the same offsets; the IMU frame
                # shares the optical axes, hence the identity rotation.
                arguments=[
                    "--x", "0.02052",
                    "--y", "-0.0051",
                    "--z", "-0.01174",
                    "--frame-id", "camera",
                    "--child-frame-id", "imu",
                ],
            ),

            Node(
                package="tf2_ros",
                name="camera_to_camera_optical",
                executable="static_transform_publisher",
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
                arguments=[
                    "--frame-id", "camera",
                    "--child-frame-id", "camera_color_optical_frame",
                    # RealSense: camera_color_optical_frame, OpenLoris: d400_color
                ],
            ),

            # ---------------------------------------------------------------- #
            # RViz
            # ---------------------------------------------------------------- #

            Node(
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "-d",
                    [
                        get_package_share_directory("vs_graphs"),
                        "/config/Visualization/vsgraphs_rgbd.rviz",
                    ],
                ],
                parameters=[{
                    "use_sim_time": LaunchConfiguration("use_sim_time")
                }],
                output="screen",
            ),

            # ---------------------------------------------------------------- #
            # Nodelete
            # ---------------------------------------------------------------- #

            ComposableNodeContainer(
                condition=IfCondition(
                    LaunchConfiguration("generate_pointcloud")
                ),
                name="depth_image_proc_container",
                package="rclcpp_components",
                namespace="",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="depth_image_proc",
                        name="point_cloud_xyzrgb_node",
                        parameters=[
                            {
                                "use_sim_time": LaunchConfiguration(
                                    "use_sim_time"
                                )
                            },
                            {
                                "target_frame": "map"
                            }
                        ],
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        remappings=[
                            (
                                "rgb/camera_info",
                                LaunchConfiguration("rgb_camera_info_topic"),
                            ),
                            (
                                "rgb/image_rect_color",
                                LaunchConfiguration("rgb_image_topic"),
                            ),
                            (
                                "depth_registered/image_rect",
                                LaunchConfiguration("depth_image_topic"),
                            ),
                            (
                                "points",
                                LaunchConfiguration("pointcloud_topic")
                            ),
                        ],
                    ),
                ],
            ),

            # Semantic Scene Segmenter Node (based on semantic_scene_segmenter argument)
            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("semantic_scene_segmenter"),
                        "yoso"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_yoso.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time")
                    },
                    {
                        "visualize": LaunchConfiguration(
                            "visualize_segmented_scene"
                        )
                    }
                ],

                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yoso.yaml",
                    ],
                ],
            ),

            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration(
                            "semantic_scene_segmenter"
                        ),
                        "pfcn"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_pFCN.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time")
                    },
                    {
                        "visualize": LaunchConfiguration(
                            "visualize_segmented_scene"
                        )
                    }
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_pFCN.yaml",
                    ],
                ],
            ),

            Node(
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration(
                            "semantic_scene_segmenter"
                        ),
                        "yolo26"
                    )
                ),
                name="segmenter_ros",
                package="segmenter_ros",
                executable="segmenter_yolo26.py",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time")
                    },
                    {
                        "visualize": LaunchConfiguration(
                            "visualize_segmented_scene"
                        )
                    }
                ],
                arguments=[
                    "--ros-args",
                    "--params-file",
                    [
                        get_package_share_directory("segmenter_ros"),
                        "/config/cfg_yolo26.yaml",
                    ],
                ],
            ),
            # Structural Element Detectors
            # Node(
            #     name="situational_graphs_reasoning",
            #     package="situational_graphs_reasoning",
            #     executable="situational_graphs_reasoning",
            #     output="screen",
            #     # parameters=[
            #     #     os.path.join(
            #     #         get_package_share_directory("situational_graphs_reasoning"),
            #     #         "config",
            #     #         "params.yaml",
            #     #     )
            #     # ],
            #     # remappings=[
            #     #     (
            #     #         "situational_graphs_reasoning/graphs",
            #     #         "/s_graphs/graph_structure",
            #     #     ),
            #     # ],
            # ),
        ]
    )

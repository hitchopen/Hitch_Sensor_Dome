#
#   Copyright (c)
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#
#   This file has been adapted for the Hitch Sensor Dome project.
#   Sensor topics, frames, and URDF discovery target Robin W LiDARs +
#   Point One Nav Atlas Duo INS instead of the original DLIO defaults.
#

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare('gicp_localization')

    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/robin_w_front/points')
    imu_topic = LaunchConfiguration('imu_topic', default='/imu/data')
    # The adapter publishes this topic only for genuine SolutionType::RtkFixed.
    # Float/no-fix periods therefore fall back to LiDAR+IMU localization.
    gt_odom_topic = LaunchConfiguration(
        'gt_odom_topic', default='/gps_p1/filtered_odom_rtk_fixed')
    enu_origin_topic = LaunchConfiguration(
        'enu_origin_topic', default='/gps_p1/local_enu_origin')
    imu_only = LaunchConfiguration('imu_only', default='false')
    urdf_path = LaunchConfiguration(
        'urdf_path',
        default='')
    parent_frame = LaunchConfiguration('parent_frame', default='base_link')
    child_frame = LaunchConfiguration('child_frame', default='lidar_front_link')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value=rviz, description='Launch RViz')
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic', default_value=pointcloud_topic,
        description='Primary Robin W point cloud topic (the front sensor by default).')
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic', default_value=imu_topic,
        description='Atlas Duo IMU topic. /imu/data is the low-latency live '
                    'default; /gps_p1/imu is supported for normalized replay.')
    declare_gt_odom_topic_arg = DeclareLaunchArgument(
        'gt_odom_topic', default_value=gt_odom_topic,
        description='Adapter RTK-fixed-only INS odometry. Used for initial '
                    'pose, cross-check, calibration, and recovery.')
    declare_enu_origin_topic_arg = DeclareLaunchArgument(
        'enu_origin_topic', default_value=enu_origin_topic,
        description='Transient-local adapter datum metadata topic.')
    declare_imu_only_arg = DeclareLaunchArgument(
        'imu_only', default_value=imu_only,
        description='If true, disable GICP and run IMU-only propagation.')
    declare_urdf_path_arg = DeclareLaunchArgument(
        'urdf_path', default_value=urdf_path,
        description='Absolute path to sensor_dome.urdf used by '
                    'robot_state_publisher (provides base_link → imu_link / '
                    'lidar_*_link TFs). When empty, the launcher auto-walks '
                    'up from this file looking for '
                    'GLIM_plusplus/config/sensor_dome.urdf.')
    declare_parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value=parent_frame,
        description='Parent frame of the LiDAR sensor in the URDF.')
    declare_child_frame_arg = DeclareLaunchArgument(
        'child_frame', default_value=child_frame,
        description='Primary LiDAR sensor frame (must match the incoming '
                    'PointCloud2 header.frame_id and the URDF link).')
    declare_map_path_arg = DeclareLaunchArgument(
        'map_path', default_value='',
        description='Path to PCD map file for localization (overrides '
                    'localization.yaml when non-empty). Use the PCD produced '
                    'by GLIM_plusplus offline mapping.')
    declare_local_enu_origin_arg = DeclareLaunchArgument(
        'local_enu_origin', default_value='',
        description='Optional explicit "lat_deg,lon_deg,alt_m" assertion '
                    'against the map manifest. Live adapter metadata is still '
                    'validated by default.')
    declare_require_live_enu_origin_arg = DeclareLaunchArgument(
        'require_live_enu_origin', default_value='true',
        description='Require transient-local datum metadata from the live '
                    'adapter before accepting point clouds or GT odometry.')

    # Hitch Sensor Dome — two-mode selector.
    # race (default): front-only LiDAR, tight crop, fewer GICP iters,
    #   yaw-rate Kp/Kq attenuation, debug publishers off. Tuned for
    #   lowest latency on a pre-built race-track map.
    # safe: 3× LiDAR concat, full 100 m crop, upstream GICP iter count,
    #   tighter convergence, all debug topics on, no yaw-rate attenuation.
    #   Tuned for maximum sensor coverage and robustness.
    # custom: skips the mode overlay entirely; the base localization.yaml
    #   plus any operator-supplied --params-file is what loads.
    # The selected overlay file is layered AFTER cfg/localization.yaml in
    # the parameter chain, so its entries override the race-mode defaults
    # without touching the base file.
    declare_mode_arg = DeclareLaunchArgument(
        'mode', default_value='race',
        description='Localization mode: "race" (front-only, low-latency), '
                    '"safe" (3× LiDARs, max coverage, all debug on), or '
                    '"custom" (no overlay; base YAML only). Default: race.')

    # Hitch Sensor Dome: RTK-gating republisher controls.
    declare_run_rtk_gate_arg = DeclareLaunchArgument(
        'run_rtk_gate', default_value='false',
        description='Compatibility mode: spawn nav_sat_gated_odom. The '
                    'helper subscribes to ins_odom_topic + ins_fix_topic and '
                    'republishes RTK-class samples on gt_odom_topic. Keep '
                    'false with the adapter fixed-only topic.')
    declare_ins_odom_topic_arg = DeclareLaunchArgument(
        'ins_odom_topic', default_value='/gps_p1/filtered_odom',
        description='Atlas Duo INS odometry topic (input to nav_sat_gated_odom).')
    declare_ins_fix_topic_arg = DeclareLaunchArgument(
        'ins_fix_topic', default_value='/gps_p1/fix',
        description='Atlas Duo NavSatFix topic — the RTK quality signal.')
    declare_rtk_max_pos_sigma_arg = DeclareLaunchArgument(
        'rtk_max_position_stddev', default_value='0.10',
        description='Reject NavSatFix samples whose position covariance σ '
                    'exceeds this (m). This compatibility bridge always '
                    'requires RTK-class status.')

    localization_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'localization.yaml'])

    # Publish the full vehicle URDF via robot_state_publisher. This provides
    # the real base_link → lidar_*_link and base_link → imu_link transforms
    # from sensor_dome.urdf, generated by
    # GLIM_plusplus/config/generate_sensor_dome_urdf.py.
    def make_robot_state_publisher(context):
        urdf_file = LaunchConfiguration('urdf_path').perform(context).strip()
        # If no explicit path was given, walk up from this launch file
        # looking for GLIM_plusplus/config/sensor_dome.urdf (the canonical
        # location in this repo).
        if not urdf_file:
            d = os.path.dirname(os.path.abspath(__file__))
            for _ in range(10):
                candidate = os.path.join(d, 'GLIM_plusplus', 'config', 'sensor_dome.urdf')
                if os.path.isfile(candidate):
                    urdf_file = candidate
                    break
                # Also accept a sibling layout where sensor_dome.urdf sits
                # next to the launch file (e.g. inside an install share dir).
                fallback = os.path.join(d, 'sensor_dome.urdf')
                if os.path.isfile(fallback):
                    urdf_file = fallback
                    break
                d = os.path.dirname(d)
        if not os.path.isfile(urdf_file):
            raise RuntimeError(
                f"sensor_dome.urdf not found at '{urdf_file}'. Generate it "
                f"with `cd GLIM_plusplus/config && python3 "
                f"generate_sensor_dome_urdf.py`, or pass an explicit path "
                f"with urdf_path:=<abs-path>.")
        with open(urdf_file, 'r') as f:
            robot_description = f.read()
        node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )
        return [node]

    # GICP Localization Node
    def make_localization_node(context):
        map_path_value = LaunchConfiguration('map_path').perform(context).strip()
        local_enu_origin_value = LaunchConfiguration(
            'local_enu_origin').perform(context).strip()
        child_frame_value = LaunchConfiguration('child_frame').perform(context).strip()
        mode_value = LaunchConfiguration('mode').perform(context).strip().lower()

        params = [localization_yaml_path]

        # Mode overlay: ros2's --params-file chain applies later files'
        # values on top of earlier ones, so loading the overlay AFTER the
        # base yaml lets the overlay's keys override.
        if mode_value == 'safe':
            overlay = PathJoinSubstitution(
                [current_pkg, 'cfg', 'localization_safe.yaml']).perform(context)
            params.append(overlay)
            print(f"[gicp_localization launch] mode = SAFE — layering overlay: {overlay}")
        elif mode_value == 'race':
            print("[gicp_localization launch] mode = RACE — base localization.yaml (no overlay)")
        elif mode_value == 'custom':
            print("[gicp_localization launch] mode = CUSTOM — no overlay applied; "
                  "pass your own --params-file with --ros-args")
        else:
            print(f"[gicp_localization launch] WARN: unknown mode '{mode_value}' "
                  f"— falling back to RACE defaults")

        params.append({'localization/lidar_frame': child_frame_value})
        params.append({
            'localization/expected_enu_origin': local_enu_origin_value,
            'localization/require_live_enu_origin': ParameterValue(
                LaunchConfiguration('require_live_enu_origin'),
                value_type=bool),
        })
        params.append({
            'localization/imu_only': ParameterValue(
                LaunchConfiguration('imu_only'), value_type=bool),
        })
        if map_path_value:
            params.append({'localization/map_path': map_path_value})

        node = Node(
            package='gicp_localization',
            executable='gicp_localization_node',
            output='screen',
            parameters=params,
            remappings=[
                ('pointcloud', pointcloud_topic),
                ('imu', imu_topic),
                ('gt_odom', gt_odom_topic),
                ('enu_origin', enu_origin_topic),
                ('localized_pose', 'gicp/localization/pose'),
                ('localized_odom', 'gicp/localization/odom'),
                ('localized_path', 'gicp/localization/path'),
                ('map', 'gicp/localization/map'),
            ],
        )
        return [node]

    def make_rtk_gate(context):
        if LaunchConfiguration('run_rtk_gate').perform(context).lower() != 'true':
            return []
        ins_odom = LaunchConfiguration('ins_odom_topic').perform(context)
        ins_fix  = LaunchConfiguration('ins_fix_topic').perform(context)
        out_odom = LaunchConfiguration('gt_odom_topic').perform(context)
        params = [{
            'require_rtk_fixed': True,
            'max_position_stddev': float(LaunchConfiguration('rtk_max_position_stddev').perform(context)),
            'max_fix_age_s': 0.5,
            'report_interval_s': 10.0,
        }]
        return [Node(
            package='gicp_localization',
            executable='nav_sat_gated_odom',
            name='nav_sat_gated_odom',
            output='screen',
            parameters=params,
            remappings=[
                ('odom', ins_odom),
                ('fix',  ins_fix),
                ('odom_rtk_only', out_odom),
            ],
        )]

    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'localization.rviz'])

    def make_rviz_node(context):
        yaml_path = PathJoinSubstitution(
            [FindPackageShare('gicp_localization'), 'cfg', 'localization.yaml']
        ).perform(context)
        with open(yaml_path, 'r') as f:
            ros_params = yaml.safe_load(f).get('/**', {}).get('ros__parameters', {})
        map_frame = ros_params.get('localization/map_frame', 'map')
        base_frame = ros_params.get('localization/base_frame', 'base_link')

        template_path = rviz_config_path.perform(context)
        with open(template_path, 'r') as f:
            rviz_content = f.read()
        rviz_content = rviz_content.replace('__MAP_FRAME__', map_frame)
        rviz_content = rviz_content.replace('__BASE_FRAME__', base_frame)

        tmp = tempfile.NamedTemporaryFile(suffix='.rviz', mode='w', delete=False)
        tmp.write(rviz_content)
        tmp.close()

        return [Node(
            package='rviz2',
            executable='rviz2',
            name='gicp_localization_rviz',
            arguments=['-d', tmp.name],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        )]

    return LaunchDescription([
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_gt_odom_topic_arg,
        declare_enu_origin_topic_arg,
        declare_imu_only_arg,
        declare_urdf_path_arg,
        declare_parent_frame_arg,
        declare_child_frame_arg,
        declare_map_path_arg,
        declare_local_enu_origin_arg,
        declare_require_live_enu_origin_arg,
        declare_mode_arg,
        declare_run_rtk_gate_arg,
        declare_ins_odom_topic_arg,
        declare_ins_fix_topic_arg,
        declare_rtk_max_pos_sigma_arg,
        OpaqueFunction(function=make_robot_state_publisher),
        OpaqueFunction(function=make_rtk_gate),
        OpaqueFunction(function=make_localization_node),
        OpaqueFunction(function=make_rviz_node),
    ])

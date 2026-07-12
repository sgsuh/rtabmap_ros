#
# To avoid log buffering:
# "stdbuf -o L ros2 launch rtabmap_launch rtabmap_amr_preprocessors.launch.py ..."
#

import os

from launch import LaunchDescription, Substitution, LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
from typing import Text
from ament_index_python.packages import get_package_share_directory

#Based on https://answers.ros.org/question/363763/ros2-how-best-to-conditionally-include-a-prefix-in-a-launchpy-file/
class ConditionalText(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> Text:
        if self.condition == True or self.condition == 'true' or self.condition == 'True':
            return self.text_if
        else:
            return self.text_else
            
class ConditionalBool(Substitution):
    def __init__(self, text_if, text_else, condition):
        self.text_if = text_if
        self.text_else = text_else
        self.condition = condition

    def perform(self, context: 'LaunchContext') -> bool:
        if self.condition:
            return self.text_if
        else:
            return self.text_else

def launch_setup(context, *args, **kwargs):
    # conditionals can only be evaluated in an OpaqueFunction
    config_rtab_mapping = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rtabmap_config_mapping.ini')
    config_rtab_localization = os.path.join(get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rtabmap_config_localization.ini')
    config_rtab = ConditionalText(config_rtab_mapping, config_rtab_localization, LaunchConfiguration('mapping').perform(context)).perform(context)

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    tf_remapping = ''.join(["/", LaunchConfiguration('namespace').perform(context), "/tf"])
    tf_static_remapping = ''.join(["/", LaunchConfiguration('namespace').perform(context), "/tf_static"])
    transform_remappings = [('/tf', tf_remapping), ('/tf_static', tf_static_remapping)]
               
    group_action = [
        DeclareLaunchArgument('full_namespace', default_value=''.join([LaunchConfiguration('namespace').perform(context), "/rtabmap"]), description='namespace for rtabmap related nodes and topics'),

        DeclareLaunchArgument('rtabmap_config', default_value=config_rtab, description='rtabmap configuration ini file'),

        DeclareLaunchArgument('depth',           default_value=ConditionalText('false', 'true', IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' == 'true'"]))._predicate_func(context)), description=''),
        DeclareLaunchArgument('subscribe_rgb',   default_value=LaunchConfiguration('depth'),           description=''),
        DeclareLaunchArgument('args',            default_value=LaunchConfiguration('args'),            description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('sync_queue_size', default_value=LaunchConfiguration('sync_queue_size'), description='Queue size of topic synchronizers.'),
        DeclareLaunchArgument('qos_image',       default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for image input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_camera_info', default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for camera info input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_scan',        default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for scan input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_odom',        default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for odometry input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_user_data',   default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for user input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_imu',         default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for imu input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        DeclareLaunchArgument('qos_gps',         default_value=LaunchConfiguration('sensor_data_qos'), description='Specific QoS used for gps input data: 0=system default, 1=Reliable, 2=Best Effort.'),
        
        # These arguments should not be modified, see referred topics without "_relay" suffix above
        DeclareLaunchArgument('rgb_topic_relay',         default_value=ConditionalText(''.join([LaunchConfiguration('rgb_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('rgb_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('depth_topic_relay',       default_value=ConditionalText(''.join([LaunchConfiguration('depth_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('depth_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('left_image_topic_relay',  default_value=ConditionalText(''.join([LaunchConfiguration('left_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('left_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('right_image_topic_relay', default_value=ConditionalText(''.join([LaunchConfiguration('right_image_topic').perform(context), "_relay"]), ''.join(LaunchConfiguration('right_image_topic').perform(context)), LaunchConfiguration('compressed').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('rgbd_topic_relay',        default_value=ConditionalText(''.join(LaunchConfiguration('rgbd_topic').perform(context)), ''.join([LaunchConfiguration('rgbd_topic').perform(context), "_relay"]), LaunchConfiguration('rgbd_sync').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('scan_topic_relay',        default_value=ConditionalText(LaunchConfiguration('multi_lidar_merge_scan_topic').perform(context), LaunchConfiguration('scan_topic').perform(context), LaunchConfiguration('multi_lidar_preprocess').perform(context)), description='Should not be modified manually!'),
        DeclareLaunchArgument('cloud_topic_relay',       default_value=ConditionalText(LaunchConfiguration('multi_lidar_merge_cloud_topic').perform(context), LaunchConfiguration('scan_cloud_topic').perform(context), LaunchConfiguration('multi_lidar_preprocess').perform(context)), description='Should not be modified manually!'),
    
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
    ]

    standalone_nodes = GroupAction(
        condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('use_composition'), "' == 'true'"])),
        actions=[
            Node(
                package='rtabmap_util', executable='lidar_deskewing', name='lidar_deskewer_1', output="screen",
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('multi_lidar_preprocess'), "' == 'true'"])),
                parameters=[{
                    "queue_size": LaunchConfiguration('topic_queue_size'),
                    "qos": LaunchConfiguration('qos_scan'),
                    "fixed_frame_id": LaunchConfiguration('multi_lidar_fixed_frame'),
                    "wait_for_transform": LaunchConfiguration('multi_lidar_wait_for_tf'),
                    "slerp": LaunchConfiguration('multi_lidar_deskew_use_slerp'),
                    "is2d": LaunchConfiguration('multi_lidar_is_2d'),
                    "clockwise_scan": LaunchConfiguration('multi_lidar_clockwise_scan')}],
                remappings=transform_remappings + [
                    ("input_scan", LaunchConfiguration('multi_lidar_scan_topic_1')),
                    ("input_cloud", LaunchConfiguration('multi_lidar_cloud_topic_1'))],
                arguments=[LaunchConfiguration("args"), '--ros-args', '--log-level', LaunchConfiguration('log_level')],
                namespace=LaunchConfiguration('full_namespace')),
            Node(
                package='rtabmap_util', executable='lidar_deskewing', name='lidar_deskewer_2', output="screen",
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('multi_lidar_preprocess'), "' == 'true'"])),
                parameters=[{
                    "queue_size": LaunchConfiguration('topic_queue_size'),
                    "qos": LaunchConfiguration('qos_scan'),
                    "fixed_frame_id": LaunchConfiguration('multi_lidar_fixed_frame'),
                    "wait_for_transform": LaunchConfiguration('multi_lidar_wait_for_tf'),
                    "slerp": LaunchConfiguration('multi_lidar_deskew_use_slerp'),
                    "is2d": LaunchConfiguration('multi_lidar_is_2d'),
                    "clockwise_scan": LaunchConfiguration('multi_lidar_clockwise_scan')}],
                remappings=transform_remappings + [
                    ("input_scan", LaunchConfiguration('multi_lidar_scan_topic_2')),
                    ("input_cloud", LaunchConfiguration('multi_lidar_cloud_topic_2'))],
                arguments=[LaunchConfiguration("args"), '--ros-args', '--log-level', LaunchConfiguration('log_level')],
                namespace=LaunchConfiguration('full_namespace')),
            Node(
                package='rtabmap_util', executable='point_cloud_aggregator', name='lidar_merger', output="screen",
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('multi_lidar_preprocess'), "' == 'true'"])),
                parameters=[{
                    "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                    "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                    "qos": LaunchConfiguration('qos_scan'),
                    "fixed_frame_id": LaunchConfiguration('multi_lidar_fixed_frame'),
                    "frame_id": LaunchConfiguration('multi_lidar_merge_result_frame'),
                    "approx_sync": LaunchConfiguration('multi_lidar_merge_approx_sync'),
                    "approx_sync_max_interval": LaunchConfiguration('multi_lidar_merge_approx_sync_threshold'),
                    "inter_message_lower_bound": LaunchConfiguration('multi_lidar_merge_inter_message_lower_bound'),
                    "age_penalty": LaunchConfiguration('multi_lidar_merge_age_penalty'),
                    "count": LaunchConfiguration('multi_lidar_num_lidars'),
                    "wait_for_transform": LaunchConfiguration('multi_lidar_wait_for_tf'),
                    "xyz_output": LaunchConfiguration('multi_lidar_merge_xyz_output'),
                    "2d_output": LaunchConfiguration('multi_lidar_is_2d'),
                    "scan_angle_min": LaunchConfiguration('multi_lidar_merge_scan_angle_min'),
                    "scan_angle_max": LaunchConfiguration('multi_lidar_merge_scan_angle_max'),
                    "scan_angle_increment": LaunchConfiguration('multi_lidar_merge_scan_angle_increment'),
                    "scan_range_min": LaunchConfiguration('multi_lidar_merge_scan_range_min'),
                    "scan_range_max": LaunchConfiguration('multi_lidar_merge_scan_range_max')}],
                remappings=transform_remappings + [
                    ("cloud1", ConditionalText(
                        ''.join([LaunchConfiguration('multi_lidar_scan_topic_1').perform(context), "/deskewed"]),
                        ''.join([LaunchConfiguration('multi_lidar_cloud_topic_1').perform(context), "/deskewed"]),
                        LaunchConfiguration('multi_lidar_is_2d').perform(context))),
                    ("cloud2", ConditionalText(
                        ''.join([LaunchConfiguration('multi_lidar_scan_topic_2').perform(context), "/deskewed"]),
                        ''.join([LaunchConfiguration('multi_lidar_cloud_topic_2').perform(context), "/deskewed"]),
                        LaunchConfiguration('multi_lidar_is_2d').perform(context))),
                    ("combined_scan", LaunchConfiguration('scan_topic_relay')),
                    ("combined_cloud", LaunchConfiguration('cloud_topic_relay'))],
                arguments=[LaunchConfiguration("args"), '--ros-args', '--log-level', LaunchConfiguration('log_level')],
                namespace=LaunchConfiguration('full_namespace')),

            # Relays RGB-Depth
            Node(
                package='image_transport', executable='republish', name='republish_rgb',
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' != 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
                remappings=transform_remappings + [
                    (['in/', LaunchConfiguration('rgb_image_transport')], [LaunchConfiguration('rgb_topic'), '/', LaunchConfiguration('rgb_image_transport')]),
                    ('out', LaunchConfiguration('rgb_topic_relay'))], 
                arguments=[LaunchConfiguration('rgb_image_transport'), 'raw'],
                namespace=LaunchConfiguration('full_namespace')),
            Node(
                package='image_transport', executable='republish', name='republish_depth',
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' != 'true' and ('", LaunchConfiguration('subscribe_rgbd'), "' != 'true' or '", LaunchConfiguration('rgbd_sync'),"'=='true') and '", LaunchConfiguration('compressed'), "' == 'true'"])),
                remappings=transform_remappings + [
                    (['in/', LaunchConfiguration('depth_image_transport')], [LaunchConfiguration('depth_topic'), '/', LaunchConfiguration('depth_image_transport')]),
                    ('out', LaunchConfiguration('depth_topic_relay'))], 
                arguments=[LaunchConfiguration('depth_image_transport'), 'raw'],
                namespace=LaunchConfiguration('full_namespace')),
            Node(
                package='rtabmap_sync', executable='rgbd_sync', name="rgbd_sync", output="screen",
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('stereo'), "' != 'true' and '", LaunchConfiguration('rgbd_sync'), "' == 'true'"])),
                parameters=[{
                    "approx_sync": LaunchConfiguration('approx_rgbd_sync'),
                    "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                    "approx_sync_inter_message_lower_bound": LaunchConfiguration('rgbd_sync_inter_message_lower_bound'),
                    "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                    "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                    "qos": LaunchConfiguration('qos_image'),
                    "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                    "depth_scale": LaunchConfiguration('depth_scale')}],
                remappings=transform_remappings + [
                    ("rgb/image", LaunchConfiguration('rgb_topic_relay')),
                    ("depth/image", LaunchConfiguration('depth_topic_relay')),
                    ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                    ("rgbd_image", LaunchConfiguration('rgbd_topic_relay'))],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                namespace=LaunchConfiguration('full_namespace'))
        ]
    )
    group_action.append(standalone_nodes)

    composed_nodes = LoadComposableNodes(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_composition'), "' == 'true'"])),
        target_container=LaunchConfiguration('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::LidarDeskewing',
                namespace=LaunchConfiguration('full_namespace'),
                name='lidar_deskewer_1',
                parameters=[{
                    "queue_size": LaunchConfiguration('topic_queue_size'),
                    "qos": LaunchConfiguration('qos_scan'),
                    "fixed_frame_id": LaunchConfiguration('multi_lidar_fixed_frame'),
                    "wait_for_transform": LaunchConfiguration('multi_lidar_wait_for_tf'),
                    "slerp": LaunchConfiguration('multi_lidar_deskew_use_slerp'),
                    "is2d": LaunchConfiguration('multi_lidar_is_2d'),
                    "clockwise_scan": LaunchConfiguration('multi_lidar_clockwise_scan')}],
                remappings=transform_remappings + [
                    ("input_scan", LaunchConfiguration('multi_lidar_scan_topic_1')),
                    ("input_cloud", LaunchConfiguration('multi_lidar_cloud_topic_1'))],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::LidarDeskewing',
                namespace=LaunchConfiguration('full_namespace'),
                name='lidar_deskewer_2',
                parameters=[{
                    "queue_size": LaunchConfiguration('topic_queue_size'),
                    "qos": LaunchConfiguration('qos_scan'),
                    "fixed_frame_id": LaunchConfiguration('multi_lidar_fixed_frame'),
                    "wait_for_transform": LaunchConfiguration('multi_lidar_wait_for_tf'),
                    "slerp": LaunchConfiguration('multi_lidar_deskew_use_slerp'),
                    "is2d": LaunchConfiguration('multi_lidar_is_2d'),
                    "clockwise_scan": LaunchConfiguration('multi_lidar_clockwise_scan')}],
                remappings=transform_remappings + [
                    ("input_scan", LaunchConfiguration('multi_lidar_scan_topic_2')),
                    ("input_cloud", LaunchConfiguration('multi_lidar_cloud_topic_2'))],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='rtabmap_util',
                plugin='rtabmap_util::PointCloudAggregator',
                name='lidar_merger',
                namespace=LaunchConfiguration('full_namespace'),
                parameters=[{
                    "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                    "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                    "qos": LaunchConfiguration('qos_scan'),
                    "fixed_frame_id": LaunchConfiguration('multi_lidar_fixed_frame'),
                    "frame_id": LaunchConfiguration('multi_lidar_merge_result_frame'),
                    "approx_sync": LaunchConfiguration('multi_lidar_merge_approx_sync'),
                    "approx_sync_max_interval": LaunchConfiguration('multi_lidar_merge_approx_sync_threshold'),
                    "inter_message_lower_bound": LaunchConfiguration('multi_lidar_merge_inter_message_lower_bound'),
                    "age_penalty": LaunchConfiguration('multi_lidar_merge_age_penalty'),
                    "count": LaunchConfiguration('multi_lidar_num_lidars'),
                    "wait_for_transform": LaunchConfiguration('multi_lidar_wait_for_tf'),
                    "xyz_output": LaunchConfiguration('multi_lidar_merge_xyz_output'),
                    "2d_output": LaunchConfiguration('multi_lidar_is_2d'),
                    "scan_angle_min": LaunchConfiguration('multi_lidar_merge_scan_angle_min'),
                    "scan_angle_max": LaunchConfiguration('multi_lidar_merge_scan_angle_max'),
                    "scan_angle_increment": LaunchConfiguration('multi_lidar_merge_scan_angle_increment'),
                    "scan_range_min": LaunchConfiguration('multi_lidar_merge_scan_range_min'),
                    "scan_range_max": LaunchConfiguration('multi_lidar_merge_scan_range_max')}],
                remappings=transform_remappings + [
                    ("cloud1", ConditionalText(
                        ''.join([LaunchConfiguration('multi_lidar_scan_topic_1').perform(context), "/deskewed"]),
                        ''.join([LaunchConfiguration('multi_lidar_cloud_topic_1').perform(context), "/deskewed"]),
                        LaunchConfiguration('multi_lidar_is_2d').perform(context))),
                    ("cloud2", ConditionalText(
                        ''.join([LaunchConfiguration('multi_lidar_scan_topic_2').perform(context), "/deskewed"]),
                        ''.join([LaunchConfiguration('multi_lidar_cloud_topic_2').perform(context), "/deskewed"]),
                        LaunchConfiguration('multi_lidar_is_2d').perform(context))),
                    ("combined_scan", LaunchConfiguration('scan_topic_relay')),
                    ("combined_cloud", LaunchConfiguration('cloud_topic_relay'))],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # TODO: compressed republish, or stuff like that
            ComposableNode(
                package='rtabmap_sync',
                plugin='rtabmap_sync::RGBDSync',
                name='rgbd_sync',
                namespace=LaunchConfiguration('full_namespace'),
                parameters=[{
                    "approx_sync": LaunchConfiguration('approx_rgbd_sync'),
                    "approx_sync_max_interval": LaunchConfiguration('approx_sync_max_interval'),
                    "approx_sync_inter_message_lower_bound": LaunchConfiguration('rgbd_sync_inter_message_lower_bound'),
                    "topic_queue_size": LaunchConfiguration('topic_queue_size'),
                    "sync_queue_size": LaunchConfiguration('sync_queue_size'),
                    "qos": LaunchConfiguration('qos_image'),
                    "qos_camera_info": LaunchConfiguration('qos_camera_info'),
                    "depth_scale": LaunchConfiguration('depth_scale')}],
                remappings=transform_remappings + [
                    ("rgb/image", LaunchConfiguration('rgb_topic_relay')),
                    ("depth/image", LaunchConfiguration('depth_topic_relay')),
                    ("rgb/camera_info", LaunchConfiguration('camera_info_topic')),
                    ("rgbd_image", LaunchConfiguration('rgbd_topic_relay'))],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    group_action.append(composed_nodes)
    
    return group_action

def generate_launch_description():
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )
    config_libpm = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'pm_config.yaml'
    )
    
    return LaunchDescription([
        # Composition
        DeclareLaunchArgument('use_composition', default_value='false',                   description='Whether to use composition or not.'),
        DeclareLaunchArgument('container_name',  default_value='preprocessors_container', description='Name of container to load nodes into.'),

        # Configs
        DeclareLaunchArgument('libpm_cfg',   default_value=config_libpm,             description='Configuration file path for libpointmatcher(ICP backend).'),
        DeclareLaunchArgument('gui_cfg',     default_value='~/.ros/rtabmap_gui.ini', description='Configuration file path for rtabmap_viz(standalone GUI).'),
        DeclareLaunchArgument('rviz_cfg',    default_value=config_rviz,              description='Configuration file path for rviz.'),
        DeclareLaunchArgument('camera_name', default_value='camera',                 description='Camera name'),

        # Arguments
        DeclareLaunchArgument('use_sim_time',  default_value='false',    description='Use simulation clock(topic /clock).'),
        DeclareLaunchArgument('args',          default_value='--uerror', description='Can be used to pass RTAB-Map\'s parameters or other flags like --udebug and --delete_db_on_start/-d'),
        DeclareLaunchArgument('launch_prefix', default_value='',         description='For debugging purpose, it fills prefix tag of the nodes, e.g., "xterm -e gdb -ex run --args".'),
        DeclareLaunchArgument('output',        default_value='screen',   description='Controls node output (screen or log).'),

        # Node settings
        DeclareLaunchArgument('namespace',     default_value='amr',  description='Namespace prifix. \'/rtabmap\' is always appended. Required.'),
        DeclareLaunchArgument('log_level',     default_value='error', description="ROS logging level (debug, info, warn, error). For RTAB-Map\'s logger level, use \"args\" argument."),

        # Visualization
        DeclareLaunchArgument('rtabmap_viz', default_value='false', description='Launch rtabmap_viz(standalone GUI).'),
        DeclareLaunchArgument('rviz',        default_value='true',  description='Launch rviz.'),
        
        # Map database
        DeclareLaunchArgument('rtabmap_db', default_value='~/.ros/rtabmap.db', description='Rtabmap database file path.'),

        # Localization initial pose
        DeclareLaunchArgument('initial_pose', default_value='', description='Set an initial pose (only in localization mode). Format: "x y z roll pitch yaw" or "x y z qx qy qz qw". Default: see "RGBD/StartAtOrigin" doc'),

        # Mapping/Localization mode
        DeclareLaunchArgument('mapping', default_value='false', description='Launch mapping or localization mode.'),

        # ROS data Queue
        DeclareLaunchArgument('topic_queue_size', default_value='5', description='Queue size of individual topic subscribers.'),
        DeclareLaunchArgument('sensor_data_qos',  default_value='2', description='QoS used for sensor input data: 0=system default, 1=Reliable, 2=Best Effort.'),

        # Sensors sync policy
        DeclareLaunchArgument('approx_sync',              default_value='true', description='Sync sensor inputs using approximate vs exact time sync policy.'),
        DeclareLaunchArgument('approx_sync_max_interval', default_value='0.1',  description='If approx_sync is true, this parameter in seconds is the maximum time window length for sync. 0==Inf.'),
        DeclareLaunchArgument('sync_queue_size',          default_value='5',    description='Queue size of topic synchronizers.'),

        # RGBD sync
        DeclareLaunchArgument('rgbd_sync',                           default_value='true',                           description='Pre-sync rgb_topic, depth_topic, camera_info_topic.'),
        DeclareLaunchArgument('rgbd_sync_inter_message_lower_bound', default_value='0.033',                          description='Approximate sync policy inter-message lower bound. Tune this parameter starting from 1.0 / camera rate / 2.'),
        DeclareLaunchArgument('approx_rgbd_sync',                    default_value='false',                          description='false=exact synchronization. false for realsense(with correct setups).'),
        DeclareLaunchArgument('subscribe_rgbd',                      default_value=LaunchConfiguration('rgbd_sync'), description='Already synchronized RGB-D related topic, e.g., with rtabmap_sync/rgbd_sync nodelet.'),
        DeclareLaunchArgument('depth_scale',                         default_value='1.0',                            description='Scale factor for depth images.'),
        
        # Image topic compression
        DeclareLaunchArgument('compressed',            default_value='false',           description='Subscribe to compressed image topics.'),
        DeclareLaunchArgument('rgb_image_transport',   default_value='compressed',      description='Common types: compressed, theora (see "rosrun image_transport list_transports").'),
        DeclareLaunchArgument('depth_image_transport', default_value='compressedDepth', description='Compatible types: compressedDepth (see "rosrun image_transport list_transports").'),
 
        # Camera topics
        DeclareLaunchArgument('stereo',                  default_value='false',                                                                                                         description='Use stereo camera instead of RGB-D.'),
        DeclareLaunchArgument('rgb_topic',               default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('camera_name'), '/color/image'],                description=''),
        DeclareLaunchArgument('depth_topic',             default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('camera_name'), '/depth/image'],                description=''),
        DeclareLaunchArgument('camera_info_topic',       default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('camera_name'), '/color/camera_info'],          description=''),
        DeclareLaunchArgument('rgbd_topic',              default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('camera_name'), '/rgbd_image'],                 description=''),
        DeclareLaunchArgument('stereo_namespace',        default_value='stereo_camera',                                                                                                 description=''),
        DeclareLaunchArgument('left_image_topic',        default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('stereo_namespace'), '/left/image_rect_color'], description=''),
        DeclareLaunchArgument('right_image_topic',       default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('stereo_namespace'), '/right/image_rect'],      description=''),
        DeclareLaunchArgument('left_camera_info_topic',  default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('stereo_namespace'), '/left/camera_info'],      description=''),
        DeclareLaunchArgument('right_camera_info_topic', default_value=['/', LaunchConfiguration('namespace'), '/', LaunchConfiguration('stereo_namespace'), '/right/camera_info'],     description=''),
       
        # LiDAR topics (for single lidar only. for multi-lidat setups, see below "Multi-lidar deskew and merge" section.)
        DeclareLaunchArgument('scan_topic',       default_value=['/', LaunchConfiguration('namespace'), '/lidar/scan/front'], description='LaserScan message topic(e.g. 2D lidar).'),
        DeclareLaunchArgument('scan_cloud_topic', default_value=['/', LaunchConfiguration('namespace'), '/lidar/cloud'],      description='PointCloud2 message topic(e.g. 3D lidar).'),
        
        # Coordinate frames
        DeclareLaunchArgument('frame_id',               default_value='base_link',          description='Fixed frame ID of the robot (base frame), you may set "base_link" or "base_footprint" if they are published. For camera-only config, this could be "camera_link".'),
        DeclareLaunchArgument('odom_frame_id',          default_value='odom',               description='If set, TF is used(lookupTransform) to get odometry instead of pose topics.'),
        DeclareLaunchArgument('map_frame_id',           default_value='map',                description='Map frame ID.'),
        DeclareLaunchArgument('map_topic',              default_value='occupancy_grid_map', description='Map topic name.'),
        DeclareLaunchArgument('publish_tf_map',         default_value='true',               description='Publish map->odom TF.'),
        DeclareLaunchArgument('map_odom_tf_pub_period', default_value='0.02',               description='Period to publish map->odom TF.'),
        DeclareLaunchArgument('tf_tolerance',           default_value='0.1',                description='Add this value in seconds to the stamp of the map->odom TF.'),
        DeclareLaunchArgument('wait_for_transform',     default_value='0.1',                description='Time threshold to wait for transform between frames to be available.'),
        
        # Odometry
        DeclareLaunchArgument('visual_odometry',            default_value='false',      description='Launch rtabmap visual odometry node.'),
        DeclareLaunchArgument('icp_odometry',               default_value='true',       description='Launch rtabmap icp odometry node.'),
        DeclareLaunchArgument('subscribe_scan',             default_value='true',       description='Use 2d lidar.'),
        DeclareLaunchArgument('subscribe_scan_cloud',       default_value='false',      description='Use 3d lidar.'),
        DeclareLaunchArgument('odom_topic',                 default_value='odom',       description='Odometry topic name.'),
        DeclareLaunchArgument('vo_frame_id',                default_value='',           description='Visual odometry frame ID.'),
        DeclareLaunchArgument('publish_tf_odom',            default_value='true',       description='Publish odom->base_link TF.'),
        DeclareLaunchArgument('odom_tf_angular_variance',   default_value='0.01',       description='If TF is used to get odometry, this is the default angular variance.'),
        DeclareLaunchArgument('odom_tf_linear_variance',    default_value='0.001',      description='If TF is used to get odometry, this is the default linear variance.'),
        DeclareLaunchArgument('odom_sensor_sync',           default_value='true',       description=''),
        DeclareLaunchArgument('odom_guess_frame_id',        default_value='wheel_odom', description='Frame ID for external odometry guess, such as wheel odometry.'),
        DeclareLaunchArgument('odom_guess_min_translation', default_value='0.0',        description=''),
        DeclareLaunchArgument('odom_guess_min_rotation',    default_value='0.0',        description=''),
        DeclareLaunchArgument('scan_normal_k',              default_value='0',          description=''),
        DeclareLaunchArgument('odom_deskewing',             default_value='false',      description='Deskew laser scans internally. WARNING: Do not enable at the same time with \'multi_lidar_preprocess\'.'),
        DeclareLaunchArgument('odom_args',                  default_value='',           description='More arguments for odometry (overwrite same parameters in args).'),

        # Multi-lidar deskew and merge
        DeclareLaunchArgument('multi_lidar_preprocess',                      default_value='true',                                                         description='Deskew and merge scans from multiple laser scanners using external odometry source.'),
        DeclareLaunchArgument('multi_lidar_num_lidars',                      default_value='2',                                                            description='If you don\'t know what this is, you probably shouldn\'t use this.'),
        DeclareLaunchArgument('multi_lidar_scan_topic_1',                    default_value=['/', LaunchConfiguration('namespace'), '/lidar/scan/front'],   description='Topic for first 2D laser scanner.'),
        DeclareLaunchArgument('multi_lidar_scan_topic_2',                    default_value=['/', LaunchConfiguration('namespace'), '/lidar/scan/rear'],    description='Topic for second 2D laser scanner.'),
        DeclareLaunchArgument('multi_lidar_cloud_topic_1',                   default_value=['/', LaunchConfiguration('namespace'), '/lidar/cloud/front'],  description='Topic for first 3D laser scanner.'),
        DeclareLaunchArgument('multi_lidar_cloud_topic_2',                   default_value=['/', LaunchConfiguration('namespace'), '/lidar/cloud/rear'],   description='Topic for second 3D laser scanner.'),
        DeclareLaunchArgument('multi_lidar_clockwise_scan',                  default_value='true',                                                         description='Set to true if your 2d lidar scanner sweeps clockwise direction(e.g. RPlidars).'),
        DeclareLaunchArgument('multi_lidar_deskew_use_slerp',                default_value='true',                                                         description='Use SLERP interpolation when deskewing points. If set to false, tf buffer lookup happens for every scan point which adds considerable overhead.'),
        DeclareLaunchArgument('multi_lidar_fixed_frame',                     default_value='wheel_odom',                                                   description='Fixed frame to use for laser scan deskew and merge(e.g. wheel odometry frame ID, such as wheel_odom).'),
        DeclareLaunchArgument('multi_lidar_wait_for_tf',                     default_value='0.02',                                                         description='Acceptable getTransform delay for external odometry. (1.0/external odometry rate) is appropriate.'),
        DeclareLaunchArgument('multi_lidar_merge_result_frame',              default_value='base_link',                                                    description='Desired frame ID of merged scan.'),
        DeclareLaunchArgument('multi_lidar_merge_approx_sync',               default_value='true',                                                         description='Whether to approximately combine non-synced lidars.'),
        DeclareLaunchArgument('multi_lidar_merge_approx_sync_threshold',     default_value='0.1',                                                          description='Acceptable approximate sync time threshold between lidars. (1.0/lidar scan rate) is appropriate.'),
        DeclareLaunchArgument('multi_lidar_merge_inter_message_lower_bound', default_value='0.05',                                                         description='Approximate sync policy inter-message lower bound. Tune this parameter starting from (1.0/lidar scan rate) / 2.'),
        DeclareLaunchArgument('multi_lidar_merge_age_penalty',               default_value='1.0',                                                          description='Approximate sync policy age penalty.'),
        DeclareLaunchArgument('multi_lidar_merge_xyz_output',                default_value='true',                                                         description='Whether to discard fields other than xyz while merging.'),
        DeclareLaunchArgument('multi_lidar_is_2d',                           default_value='true',                                                         description='If set to true, combination of two 2d scans are converted to laserscan instead of pointcloud.'),
        DeclareLaunchArgument('multi_lidar_merge_scan_angle_min',            default_value='-3.14159265359',                                               description='Desired minimum scan angle(in radians) of merged laserscan. Effective only if multi_lidar_is_2d is true.'),
        DeclareLaunchArgument('multi_lidar_merge_scan_angle_max',            default_value='3.14159265359',                                                description='Desired maximum scan angle(in radians) of merged laserscan. Effective only if multi_lidar_is_2d is true.'),
        DeclareLaunchArgument('multi_lidar_merge_scan_angle_increment',      default_value='0.005',                                                        description='Desired scan increment angle(in radians) of merged laserscan. Effective only if multi_lidar_is_2d is true.'),
        DeclareLaunchArgument('multi_lidar_merge_scan_range_min',            default_value='0.3',                                                          description='Desired minimum scan range of merged laserscan. Effective only if multi_lidar_is_2d is true.'),
        DeclareLaunchArgument('multi_lidar_merge_scan_range_max',            default_value='20.0',                                                         description='Desired maximum scan range of merged laserscan. Effective only if multi_lidar_is_2d is true.'),
        DeclareLaunchArgument('multi_lidar_merge_scan_topic',                default_value=['/', LaunchConfiguration('namespace'), '/lidar/scan/merged'],  description='Merged scan output topic'),
        DeclareLaunchArgument('multi_lidar_merge_cloud_topic',               default_value=['/', LaunchConfiguration('namespace'), '/lidar/cloud/merged'], description='Merged cloud output topic'),
        
        # IMU
        DeclareLaunchArgument('imu_topic',        default_value=['/', LaunchConfiguration('namespace'), '/imu/data'], description='Used with VIO approaches and for SLAM graph optimization (gravity constraints).'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='false',                                              description=''),
        
        # GPS
        DeclareLaunchArgument('gps_topic',  default_value=['/', LaunchConfiguration('namespace'), '/gps/fix'], description='GPS async subscription. This is used for SLAM graph optimization and loop closure candidates selection.'),

        # Tag/Landmark
        DeclareLaunchArgument('tag_topic',            default_value=['/', LaunchConfiguration('namespace'), '/apriltag_detections'], description='AprilTag topic async subscription. This is used for SLAM graph optimization and loop closure detection. Landmark poses are also published accordingly to current optimized map. Required: Remove optional frame name parameters from apriltag\'s cfg file so that TF frame can be deducted from topic\'s family and id.'),
        DeclareLaunchArgument('tag_linear_variance',  default_value='0.0001',                                                        description=''),
        DeclareLaunchArgument('tag_angular_variance', default_value='9999.0',                                                        description='>=9999 means rotation is ignored in optimization, when rotation estimation of the tag is not reliable or not computed.'),
        DeclareLaunchArgument('fiducial_topic',       default_value='/fiducial_transforms',                                          description='aruco_detect async subscription, use tag_linear_variance and tag_angular_variance to set covariance.'),
                
        # User Data
        DeclareLaunchArgument('subscribe_user_data',   default_value='false',            description='User data synchronized subscription.'),
        DeclareLaunchArgument('user_data_topic',       default_value='/user_data',       description=''),
        DeclareLaunchArgument('user_data_async_topic', default_value='/user_data_async', description='User data async subscription (rate should be lower than map update rate).'),
        
        # Optional
        DeclareLaunchArgument('output_goal_topic',          default_value='/goal_pose', description='Output goal topic (can be connected to nav2).'),
        DeclareLaunchArgument('use_action_for_goal',        default_value='false',      description='Connect to nav2\'s navigate_to_pose action server instead of publishing the output goal topic.'),
        DeclareLaunchArgument('ground_truth_frame_id',      default_value='',           description='e.g., "world"'),
        DeclareLaunchArgument('ground_truth_base_frame_id', default_value='',           description='e.g., "tracker", a fake frame matching the frame "frame_id" (but on different TF tree)'),
        
        OpaqueFunction(function=launch_setup),

    ])


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters = [{
        'frame_id': 'base_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'database_path': '/root/ros2_ws/src/pkg/Drone-Manager/drone_manager/config/rtabmap.db',
        'subscribe_depth': True,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'qos_image': 2,
        'qos': 2,
        'odom_topic': '/model/x500_depth_0/odometry',
        'rgbd_cameras': 1,
        'rgbd_camera_frame_id': 'x500_depth_0/OakD-Lite/base_link/IMX214',
        'wait_for_transform': 10.5,
        
        # LOCALIZATION MODE SETTINGS
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',  # Changed to true for better localization
        'Mem/STMSize': '10',  # Increased slightly for better matching
        
        # INITIAL POSE AND LOCALIZATION SETTINGS
        'initial_pose': '0.0 0.0 0.48 0.0 0.0 0.0',  # ADJUST THESE VALUES TO YOUR DRONE'S START POSITION
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        'Mem/UseOdomFeatures': 'false',
        
        # IMPROVED LOCALIZATION PARAMETERS
        'RGBD/LocalizationSmoothing': 'true',
        'Rtabmap/LocalizationLinearUpdate': '0.1',  # Update every 10cm movement
        'Rtabmap/LocalizationAngularUpdate': '0.1',  # Update every ~6 degree rotation
        'Kp/MaxFeatures': '400',
        'Vis/MaxFeatures': '1000',
        'Vis/MinInliers': '15',  # Minimum inliers for loop closure
        'RGBD/MaxLocalRetrieved': '2',  # Limit local retrievals
        'Mem/RehearsalSimilarity': '0.6',  # Similarity threshold for rehearsal
        'Rtabmap/MemoryThr': '0.0',  # Memory threshold (0=disabled)
        
        # FRAME AND TIMING SETTINGS
        'publish_tf_odom': False,
        'use_sim_time': True,
        'wait_imu_to_init': False,
        
        # ADDITIONAL LOCALIZATION IMPROVEMENTS
        'Optimizer/Strategy': '1',  # TORO optimizer
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/AngularUpdate': '0.1',
        'RGBD/LinearUpdate': '0.1',
    }]

    remappings=[
          ('rgb/image', '/camera'),
          ('rgb/camera_info', '/camera_info'),
          ('depth/image', '/depth_camera')]

    return LaunchDescription([

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings),

    ])
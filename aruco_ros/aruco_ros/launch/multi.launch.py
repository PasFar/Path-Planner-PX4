from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

def parse_csv_to_int_list(s):
    if s is None or s.strip() == "":
        return []
    parts = [p.strip() for p in s.split(',') if p.strip() != ""]
    return [int(p) for p in parts]

def parse_csv_to_float_list(s):
    if s is None or s.strip() == "":
        return []
    parts = [p.strip() for p in s.split(',') if p.strip() != ""]
    return [float(p) for p in parts]

def launch_setup(context, *args, **kwargs):

    # get raw strings from LaunchConfiguration
    marker_ids_raw = perform_substitutions(context, [LaunchConfiguration('marker_ids')])
    marker_sizes_raw = perform_substitutions(context, [LaunchConfiguration('marker_sizes')])
    output_file = perform_substitutions(context, [LaunchConfiguration('output_file')])

    marker_ids = parse_csv_to_int_list(marker_ids_raw)
    marker_sizes = parse_csv_to_float_list(marker_sizes_raw)

    aruco_single_params = {
        'image_is_rectified': True,
        # keep single-value params for backward compatibility (node will prefer marker_ids if provided)
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': 'x500_depth_0/OakD-Lite/base_link/IMX214',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
        # new multi-marker params (passed as python lists)
        'marker_ids': marker_ids,
        'marker_sizes': marker_sizes,
        'output_file': output_file,
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='multi',  # rimane 'single' se quello è il nome del binario
        parameters=[aruco_single_params],
        # remap camera topics if necessario (scommenta e adatta)
        # remappings=[('/camera', '/camera/color/image_raw'),
        #             ('/camera_info', '/camera/color/camera_info')],
    )

    return [aruco_single]


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='582',
        description='Marker ID (single, backward compatibility). '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.34',
        description='Marker size in m (single, backward compatibility). '
    )

    eye_arg = DeclareLaunchArgument(
        'eye', default_value='left',
        description='Eye. ',
        choices=['left', 'right'],
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be refered. '
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
    )

    marker_ids_arg = DeclareLaunchArgument(
        'marker_ids', default_value='201,202,203',
        description='Comma-separated list of marker IDs to track (e.g. "201,202,203").'
    )

    marker_sizes_arg = DeclareLaunchArgument(
        'marker_sizes', default_value='0.1,0.05,0.07',
        description='Comma-separated list of marker sizes in meters corresponding to marker_ids.'
    )

    output_file_arg = DeclareLaunchArgument(
        'output_file', default_value='/root/ros2_ws/src/pkg/Drone-Manager/results/aruco_poses.txt',
        description='File path where first-time detections are appended.'
    )

    # Build launch description
    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(eye_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame)
    ld.add_action(corner_refinement_arg)
    ld.add_action(marker_ids_arg)
    ld.add_action(marker_sizes_arg)
    ld.add_action(output_file_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
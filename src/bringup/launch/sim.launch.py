import logging
import os
import re
import tempfile
import yaml
import numpy as np
from lxml import etree
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from robot_model import RobotModel

from constraint_to_gazebo.constraint_to_detachable_joint import addDetachableJoints
from constraint_to_gazebo.add_masses import addMassesToDummyLinks
from constraint_to_gazebo.add_world_link import addWorldLink
from constraint_to_gazebo.close_loop import close_loop
from constraint_to_gazebo.add_ros2_control import add_ros2_control, add_gz_ros2_control


def resolve_urdf(urdf_file, moveit_config_package):
    """
    Resolves the raw robot URDF/ARDF model file.
    If 'urdf_file' is explicitly given, returns it.
    Otherwise, inspects 'config/*.urdf.xacro' in the MoveIt configuration package,
    finds the <xacro:include> referencing the .urdf or .ardf model, and resolves its path.
    """
    if urdf_file and os.path.exists(urdf_file):
        return urdf_file

    if not moveit_config_package:
        raise ValueError(
            "Either 'urdf_file' or 'moveit_config_package' must be provided."
        )

    pkg_share = get_package_share_directory(moveit_config_package)
    config_dir = os.path.join(pkg_share, 'config')

    if not os.path.exists(config_dir):
        raise FileNotFoundError(
            f"Config directory '{config_dir}' not found in package '{moveit_config_package}'."
        )

    # 1. Look for config/*.urdf.xacro and extract the .urdf / .ardf include
    xacro_files = [f for f in os.listdir(config_dir) if f.endswith('.urdf.xacro')]
    if xacro_files:
        xacro_path = os.path.join(config_dir, xacro_files[0])
        xacro_tree = etree.parse(xacro_path)

        for elem in xacro_tree.getroot().iter():
            if not isinstance(elem.tag, str):
                continue
            if 'include' in elem.tag:
                filename = elem.get('filename', '')
                # Find the import of the robot model (.urdf or .ardf)
                if any(ext in filename.lower() for ext in ['.urdf', '.ardf']):
                    # Resolve $(find <package_name>) if present
                    match = re.search(r'\$\(find\s+([^\)]+)\)', filename)
                    if match:
                        ref_pkg = match.group(1)
                        ref_pkg_share = get_package_share_directory(ref_pkg)
                        resolved_path = filename.replace(match.group(0), ref_pkg_share)
                    elif not os.path.isabs(filename):
                        resolved_path = os.path.normpath(os.path.join(config_dir, filename))
                    else:
                        resolved_path = filename

                    # If the included file is a xacro, process it to clean URDF
                    if resolved_path.endswith('.xacro'):
                        doc = xacro.process_file(resolved_path)
                        temp_urdf = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf')
                        temp_urdf.write(doc.toxml())
                        temp_urdf_path = temp_urdf.name
                        temp_urdf.close()
                        return temp_urdf_path

                    if os.path.exists(resolved_path):
                        return resolved_path
                    else:
                        raise FileNotFoundError(f"Included robot model file not found: '{resolved_path}'.")

    # 2. Fallback: check for standalone *.urdf or *.ardf directly in config/
    direct_files = [f for f in os.listdir(config_dir) if f.endswith(('.urdf', '.ardf'))]
    if direct_files:
        return os.path.join(config_dir, direct_files[0])

    raise FileNotFoundError(
        f"Could not locate robot .urdf/.ardf file from package '{moveit_config_package}'."
    )


def createRosControlConfig(active_joints):
    """Create the ros2_control YAML configuration for Gazebo simulation."""
    joint_list = [str(j) for j in active_joints]

    controller_config = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                "forward_position_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        },
        "forward_position_controller": {
            "ros__parameters": {
                "joints": joint_list,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"]
            }
        }
    }

    temp_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
    yaml.dump(controller_config, temp_yaml)
    temp_yaml_path = temp_yaml.name
    temp_yaml.close()
    return temp_yaml_path


def createDescriptions(urdf_file):
    """
    Process the constraint-based URDF:
    - Removes mock ros2_control hardware tags from MoveIt config (e.g. FakeSystem).
    - Numerically closes loops and shifts joint origins/limits so 0 matches the closed pose.
    - Injects Gazebo ros2_control and gz_ros2_control tags for the active motor joints.
    - Injects Gazebo-specific physics (detachable joints, dummy masses, world link).
    """
    logging.info('bringup: load robot model')
    robot_model = RobotModel(urdf_file)
    urdf_tree = etree.parse(urdf_file)

    robot_name = urdf_tree.getroot().get('name')
    active_joints = np.array(robot_model.joint_names)[robot_model.joints.active_joints.astype(bool)]
    temp_yaml_path = createRosControlConfig(active_joints)

    # Remove any pre-existing mock ros2_control elements (e.g. FakeSystem with mock_components)
    
    for elem in urdf_tree.getroot().findall('ros2_control'):
        urdf_tree.getroot().remove(elem)

    # Solve loops and update joint zero positions
    logging.info('bringup: close kinematic tree structure in description to comply with the constraints')
    urdf_tree, violations = close_loop(urdf_tree, robot_model)
    urdf_tree = add_ros2_control(urdf_tree, active_joints)
    urdf_tree = add_gz_ros2_control(urdf_tree, temp_yaml_path)

    # ros_desc: the kinematically closed model for ROS (robot_state_publisher & MoveIt)
    logging.info('bringup: extract description for ROS')
    ros_desc = etree.tostring(urdf_tree, encoding='unicode')

    # gazebo_desc: adds simulation physics elements for Gazebo Sim
    logging.info('bringup: add gazebo elements')
    urdf_tree = addDetachableJoints(urdf_tree, violations)
    urdf_tree = addMassesToDummyLinks(urdf_tree)
    urdf_tree = addWorldLink(urdf_tree, offset=[0.0, 0.0, 1.5])
    gazebo_desc = etree.tostring(urdf_tree, encoding='unicode')

    return ros_desc, gazebo_desc, robot_name


def launch_setup(context, *args, **kwargs):
    urdf_file_arg = LaunchConfiguration('urdf_file').perform(context)
    moveit_config_package = LaunchConfiguration('moveit_config_package').perform(context)
    actions = []

    # Resolve URDF file: uses urdf_file if given, or extracts config/*.urdf.xacro from the package
    urdf_file = resolve_urdf(urdf_file_arg, moveit_config_package)

    # Generate altered descriptions
    ros_desc, gazebo_desc, robot_name = createDescriptions(urdf_file)

    if not moveit_config_package:
        moveit_config_package = f"{robot_name}_moveit_config"

    # Dump descriptions to temporary URDF files
    logging.info('bringup: create dummy urdf files')
    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf') as ros_urdf_file:
        ros_urdf_file.write(ros_desc)
        ros_desc_path = ros_urdf_file.name

    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf') as gz_urdf_file:
        gz_urdf_file.write(gazebo_desc)
        gazebo_desc_path = gz_urdf_file.name

    # 1. Use MoveItConfigsBuilder to load package configs with the altered URDF injected!
    logging.info('bringup: create moveit config with updated urdf')
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_package)
        .robot_description(file_path=ros_desc_path)
        .to_moveit_configs()
    )

    # 2. Environment Variable for Gazebo mesh resolution
    logging.info('bringup: update gazbo environment variables')
    descriptions_share = get_package_share_directory('descriptions')
    gazebo_model_path = os.path.dirname(descriptions_share)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{gazebo_model_path}:{existing_gz_path}" if existing_gz_path else gazebo_model_path
    actions.append(
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path,
        )
    )

    # 2. Robot State Publisher (using altered ros_desc)
    logging.info('bringup: start robot_state_publisher')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ros_desc, 'use_sim_time': True}]
    )

    # 3. Gazebo Sim
    logging.info('bringup: start gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_bringup_2 = get_package_share_directory('bringup')
    world_path = os.path.join(pkg_bringup_2, 'worlds', 'world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 4. Spawn Robot in Gazebo (loads from temp URDF file)
    logging.info('bringup: spawn robot in gazebo')
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-v', '4',
            '-file', gazebo_desc_path,
            '-name', robot_name,
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    # 5. Controllers
    logging.info('bringup: start joint state broadcaster')    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    logging.info('bringup: start position controller')    
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='position_controller_spawner',
        arguments=[
            'forward_position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen'
    )

    # Sequence position controller spawner after joint_state_broadcaster finishes loading
    logging.info('bringup: delay the position controller')
    delay_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    # 6. Clock Bridge (Gazebo -> ROS)
    logging.info('bringup: bridge the clock from gazebo to ROS')    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 7. Joint State Transformer (Maps MoveIt trajectories to actuated joints)
    logging.info('bringup: start joint_state_transformer')
    joint_state_transformer_node = Node(
        package='joint_state_transformer',
        executable='joint_state_transformer',
        name='joint_state_transformer',
        output='screen',
        parameters=[{
            'max_iterations': 100,
            'tolerance': 1e-5,
            'input_action_name': LaunchConfiguration('joint_state_transformer_input_action'),
            'output_action_name': '/forward_position_controller/follow_joint_trajectory',
            'use_sim_time': True,
        }],
        condition=IfCondition(LaunchConfiguration('launch_joint_state_transformer'))
    )

    # 8. MoveGroup (configured via MoveItConfigsBuilder with injected altered URDF)
    logging.info('bringup: start moveit')
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
    )

    # 9. RViz2 (optional)
    if LaunchConfiguration('launch_rviz'):
        logging.info('bringup: start rviz, when selected')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    actions.extend([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        joint_state_broadcaster_spawner,
        delay_position_controller_spawner,
        clock_bridge,
        joint_state_transformer_node,
        move_group_node,
        rviz_node,
    ])

    return actions


def generate_launch_description():
    """Generate the launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='',
            description='MoveIt configuration package name (e.g. a_00036)'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            default_value='',
            description='Path to raw URDF (optional, auto-extracted from moveit_config_package/config/*.urdf.xacro if omitted)'
        ),
        DeclareLaunchArgument(
            'launch_joint_state_transformer',
            default_value='true',
            description='Launch joint_state_transformer to map MoveIt trajectories to actuated joints'
        ),
        DeclareLaunchArgument(
            'joint_state_transformer_input_action',
            default_value='/ee_controller/follow_joint_trajectory',
            description='Input action name for joint_state_transformer'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Whether to launch RViz2'
        ),
        OpaqueFunction(function=launch_setup)
    ])

import logging
import os
from lxml import etree
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from robot_model import RobotModel

from constraint_to_gazebo.constraint_to_detachable_joint import addDetachableJoints
from constraint_to_gazebo.add_masses import addMassesToDummyLinks
from constraint_to_gazebo.close_loop import close_loop


def createDescriptions(urdf_desc):
    """
    Process the constraint-based URDF:
    - Removes mock ros2_control hardware tags from MoveIt config (e.g. FakeSystem).
    - Numerically closes loops and shifts joint origins/limits so 0 matches the closed pose.
    - Injects Gazebo ros2_control and gz_ros2_control tags for the active motor joints.
    - Injects Gazebo-specific physics (detachable joints, dummy masses, world link).
    """
    logging.info('bringup: load robot model')
    robot_model = RobotModel(urdf_desc)
    urdf_tree = etree.ElementTree(etree.fromstring(urdf_desc))

    robot_name = urdf_tree.getroot().get('name')

    # close loop for gazebo simulation. IMPORTANT: Since the active joints are controlled via rs2_control, the staate of these must remain the same as in the description
    logging.info('bringup: close kinematic tree structure in description to comply with the constraints')
    urdf_tree, violations = close_loop(urdf_tree, robot_model)

    # gazebo_desc: adds simulation physics elements and detachable joints to close the loop for Gazebo Sim
    logging.info('bringup: add gazebo elements')
    urdf_tree = addDetachableJoints(urdf_tree, violations)
    urdf_tree = addMassesToDummyLinks(urdf_tree)
    gazebo_desc = etree.tostring(urdf_tree, encoding='unicode')

    return gazebo_desc, robot_name


def launch_setup(context, *args, **kwargs):
    moveit_config_package = LaunchConfiguration('moveit_config_package').perform(context)
    
    config_dir = os.path.join(get_package_share_directory(moveit_config_package), 'config')
    # Resolve URDF file: uses urdf_file if given, or extracts config/*.urdf.xacro from the package
    urdf_file = os.path.join(config_dir, LaunchConfiguration('urdf_file').perform(context))
    # If the included file is a xacro, process it to clean URDF
    if urdf_file.endswith('.xacro'):
        ros_desc = xacro.process_file(urdf_file,  
                mappings={
                "hardware": "gazebo",}).toxml()
    else:
        with open(urdf_file, "r", encoding="utf-8") as f:
            ros_desc = f.read()
    actions = []

    # Generate altered descriptions
    gazebo_desc, robot_name = createDescriptions(ros_desc)

    # 1. Use MoveItConfigsBuilder to load package configs with the altered URDF injected!
    logging.info('bringup: create moveit config with updated urdf')
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_package)
        .robot_description(file_path=urdf_file,  mappings={"hardware": "gazebo"})
        .to_moveit_configs()
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
    pkg_bringup = get_package_share_directory('bringup')
    world_path = os.path.join(pkg_bringup, 'worlds', 'world.sdf')

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
            '-string', gazebo_desc,
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

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
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
            'output_action_name': LaunchConfiguration('joint_state_transformer_output_action'),
            'use_sim_time': True
        }]
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
        ]
    )

    actions.extend([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        delay_joint_state_broadcaster,
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
            description='MoveIt configuration package name (e.g. a_00036)'
        ),
        DeclareLaunchArgument(
            'urdf_file',
            description='URDF file or Xacro file for the choosen robot in the config packages config directory'
        ),
        DeclareLaunchArgument(
            'joint_state_transformer_input_action',
            default_value='/joint_state_transformer/follow_joint_trajectory',
            description='Input action name for joint_state_transformer'
        ),        
        DeclareLaunchArgument(
            'joint_state_transformer_output_action',
            default_value='/forward_position_controller/follow_joint_trajectory',
            description='Output action name for joint_state_transformer'
        ),
        OpaqueFunction(function=launch_setup)
    ])

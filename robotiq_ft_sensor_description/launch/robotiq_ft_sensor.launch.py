from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='robotiq_ft_sensor_description',
            description='Package with the controller\'s configuration in "config" folder. \
                         Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'controllers_file',
            default_value='controllers.yaml',
            description='YAML file with the controllers configuration.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='robotiq_ft_sensor_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='ft300.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_world_file',
            default_value='empty.world',
            description='gazebo world file with the robot',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='""',
            description='Prefix of the joint names, useful for multi-robot setup. \
                         If changed than also joint names in the controllers \
                         configuration have to be updated. Expected format "<prefix>/"',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim_gazebo',
            default_value='true',
            description='Start robot in Gazebo simulation.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'command_interface',
            default_value='position',
            description='Robot command interface [position|velocity|effort].',
        )
    )

    # Initialize Arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    gazebo_world_file = LaunchConfiguration('gazebo_world_file')
    prefix = LaunchConfiguration('prefix')
    namespace = LaunchConfiguration('namespace')
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    start_rviz = LaunchConfiguration('start_rviz')

    # File path
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), 'rviz', 'robotiq_ft_sensor.rviz']
    )
    gazebo_world_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'gazebo', gazebo_world_file]
    )

    # Get URDF/XACRO file path
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'urdf', description_file]
            ), 
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
        # it is the save way to wrap the xacro output
        # ref: https://answers.ros.org/question/417369/caught-exception-in-launch-see-debug-for-traceback-unable-to-parse-the-value-of-parameter-robot_description-as-yaml/
    }
    
    # Nodes and Launch

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": sim_gazebo},
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            {"use_sim_time": sim_gazebo},
        ],
        condition=IfCondition(start_rviz)
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'),
                    'launch', 'gazebo.launch.py']
            )]
        ),
        launch_arguments={'verbose': 'false', 'world': gazebo_world_file}.items(),
        condition=IfCondition(sim_gazebo),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name="spawn_robotiq_ft_sensor",
        arguments=['-topic', [namespace, 'robot_description'], '-entity', [namespace, 'robotiq_ft_sensor']],
        output='screen',
        condition=IfCondition(sim_gazebo),
    )

    nodes = [
        gazebo,
        spawn_entity,
        robot_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
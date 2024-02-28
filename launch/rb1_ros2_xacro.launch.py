import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction, LogInfo, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    description_package_name = "rb1_ros2_description"
    install_dir = get_package_prefix(description_package_name)

    # This is to find the models inside the models folder in rb1_ros2_description package
    gazebo_models_path = os.path.join(description_package_name, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Define the launch arguments for the Gazebo launch file
    gazebo_launch_args = {
        'verbose': 'false',
        'pause': 'false',
    }

   # Include the Gazebo launch file with the modified launch arguments
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments=gazebo_launch_args.items(),
    )

    # Define the robot model files to be used
    robot_desc_file = "rb1_ros2_base.urdf.xacro"
    robot_desc_path = os.path.join(get_package_share_directory("rb1_ros2_description"), "xacro", robot_desc_file)
    robot_name = "rb1_robot"
    robot_namespace = "rb1_robot"


    # robot state publisher node
    robot_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace=robot_name,
        parameters=[{'frame_prefix': robot_name + '/', 'use_sim_time': use_sim_time,
                     'robot_description': ParameterValue(Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name]), value_type=str)}],
        output="both"
    )

    # spawning the robot
    robot_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name, '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-topic', '/robot_description']
    )

    # joint state broadcaster node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # differential drive control node
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "-c", "/controller_manager"],
    )

     # elevator lift control node
    lift_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lift_controller", "-c", "/controller_manager"],
    )


    # adding timer to some nodes. These will load after the gazebo is loaded and the robot is spawnwed properly.
    Timer_Actions = [TimerAction(
            period=10.0,
            actions=[LogInfo(msg= "starting robot spawning."),
                robot_spawn_node,
                LogInfo(msg= "please wait for the controller node to start........")]
        ),
        TimerAction(
            period=105.0,
            actions=[LogInfo(msg= "starting joint state broadcaster."),
                joint_state_broadcaster_spawner]
        ),
        TimerAction(
            period=105.0,
            actions=[LogInfo(msg= "starting differential drive controller."),
                diff_drive_controller_spawner,
                lift_controller_spawner]
        )
    ]



    return LaunchDescription([
       
        gazebo,
        robot_rsp_node,
        *Timer_Actions 
    ])

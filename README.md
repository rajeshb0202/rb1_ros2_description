README
-----------------------------------------------------------


# The rb1_ros2_description package
-------------------------------------------------------------------------------------------------------------

This package contains a ROS2 robot model description for Robotnik's RB-1 mobile base and controller manager configurations to interact with the robot in the Gazebo simulation.

Through this package, one can move the robot and its elevator can be used by publishing commands to its topics. Below are the steps mentioned of its usage.



## Disclaimer: 
-------------------------------------------------------------------------------------------------------------
This package only modifies/adapts files from these repositories/packages:  
- [RobotnikAutomation/rb1_base_sim](https://github.com/RobotnikAutomation/rb1_base_sim) licensed under the BSD 2-Clause "Simplified" License
- [RobotnikAutomation/rb1_base_common/rb1_base_description](https://github.com/RobotnikAutomation/rb1_base_common/tree/melodic-devel/rb1_base_description), licensed under the BSD License
- [RobotnikAutomation/robotnik_sensors],(https://github.com/RobotnikAutomation/robotnik_sensors) licensed under the BSD License




## Important files in the package regarding controllers perpective 
-------------------------------------------------------------------------------------------------------------

rb1_controller.yaml                 #Inside /config directory. This file contains the implementation of mentioned controllers.

rb1_ros2_xacro.launch.py            #Inside /launch directory. This file launches Gazebo, robot state publisher, spawning the robot in Gazebo, and starts mentioned controllers.

ros2_control.urdf.xacro             #Inside /urdf/ros2_control directory This file contains all mentioned controller plugins and the gazebo_ros2_control plugin.

rb1_ros2_base.urdf.xacro            #Inside /xacro directory This is the main robot xacro file which contains all the xacro definitions of the robot's linkages and joints.





## Controllers used in this package:
-------------------------------------------------------------------------------------------------------------
In this package, there are two controllers and one broadcaster implemented.

        1. The diff_drive_controller: 
                It is used to move the robot. The DiffDriveController is implemmented in this case.

        2. The lift_controller: 
                This is to make the robot's elevator up and down. Here JointGoupEffortController is implemented.

        3. joint_state_broadcaster: This is to publish the joints information.
                In this case, the JointStateBroadcaster is used.







######## Getting started
-------------------------------------------------------------------------------------------------------------
step 1: Installation and Building the package:
----------------------------------------------------
Clone this package into your workspace: 
        git clone https://github.com/rajeshb0202/rb1_ros2_description.git

Build and install this package:
        cd ~/ros2_ws && colcon build --packages-select rb1_ros2_description



Step 2: Starting the simulation and the controllers:
------------------------------------------------------            
For to start simulation in gazebo and the above mentioned controllers. This will take around 1 to 1.5 minutes to load all the controllers.
        ros2 launch rb1_base_description rb1_ros2_xacro.launch.py 

        "Don't close/stop the controller in this terminal"

Once the above started, check for the controllers loaded in the another terminal
        ros2 control list_controllers



step-3:  Moving the robot
-------------------------------------------------------------------------------------------------------------
(The topic available for this task is : /diff_drive_controller/cmd_vel_unstamped)

For to move the robot in the x-axis: 
            ros2 topic pub /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "linear:
            x: 0.2
            y: 0.0
            z: 0.0
            angular:
            x: 0.0
            y: 0.0
            z: 0.0"

For to teleoperate the robot using keyboard
            ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped




Step-4: Move the elevator up and down
-------------------------------------------------------------------------------------------------------------
(The topic available for this task is : /lift_controller/commands)

For to lift the elevator:
            ros2 topic pub /lift_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]" -1

For to bring the elevator down:
            ros2 topic pub /lift_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]" -1

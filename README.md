# my-openmanipulator

First, create your workspace
```
$ mkdir ~/ws-manipulator/src -p
``` 

Inside your workspace, on the src folder
```
$ cd ~/ws-manipulator/src
$ git clone https://github.com/gabrielcalmon/my-openmanipulator.git
``` 

## Simulation
After build and **source**, use the following commands.
Note this commands are used to simulate the robot.
### Forward kinematics
Allow the user to control the angle of each individual joint
```bash
$ roslaunch open_manipulator_p_gazebo open_manipulator_p_gazebo.launch
$ roslaunch open_manipulator_p_controller open_manipulator_p_controller.launch use_platform:=false
$ roslaunch open_manipulator_p_control_gui open_manipulator_p_control_gui.launch
```

### Inverse kinematics
Use the moveit package to get a input position, calculate the target and move the manipulator.
*Note the 'play' button of the Gazebo must be pressed.*
```bash
$ roslaunch open_manipulator_p_controllers joint_trajectory_controller.launch sim:=true
```

## Operation
After build and **source**, use the following commands.
*Note this commands are used to control the robot itself, and not simulate.*
### Forward kinematics
Allow the user to control the angle of each individual joint

```bash
$ roslaunch open_manipulator_p_gazebo open_manipulator_p_gazebo.launch
$ roslaunch open_manipulator_p_controller open_manipulator_p_controller.launch use_platform:=false
$ roslaunch open_manipulator_p_control_gui open_manipulator_p_control_gui.launch
```

### Inverse kinematics
Use the moveit package to get a input position, calculate the target and move the manipulator

```bash
$ roslaunch open_manipulator_p_controllers joint_trajectory_controller.launch sim:=false
```

## Data collection
The package *node_listener* contain a script used to listen to the */joint_states* topic, which uses the *sensor_msgs/JointState* message type.

```bash
$ rosrun node_listener listener.py
```

After run this command, the time series will be saved as .csv at src/time_series_csv with the name joint_states_{YYMMDD_HourMinuteSecond}.csv
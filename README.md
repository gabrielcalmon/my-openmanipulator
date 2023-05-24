# my-openmanipulator

## Setup
First, create your workspace
```bash
$ mkdir ~/ws-manipulator/src -p
``` 

Inside your workspace, on the src folder
```bash
$ cd ~/ws-manipulator/src
$ git clone https://github.com/gabrielcalmon/my-openmanipulator.git
``` 

Install the requirements:
```bash
$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-gazebo* ros-noetic-moveit* ros-noetic-industrial-core
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

First of all, run the inverse kinematics node, even to simulation or operation case. So, after that, run:

```bash
$ rosrun node_listener listener.py
```

After run this command, the time series will be saved as .csv at src/time_series_csv with the name joint_states_{YYMMDD_HourMinuteSecond}.csv

At the top of listener.py, the subsampling and point float precision can be changed
*Note that the csv file will apply the comma as decimal marker instead of dot*
*Note that it may be necessary to change the number format to scientific notation when importing the csv on your editing software*
# UGV trajectory tracking

Position control and trajectory tracking algorithm for an UGV.

## Instalation

Just clone the repository inside the ros workspace and use catkin build command to build it. See example below:

```
cd ~/catkin_ws/src/
git clone https://github.com/larics/ugv_trajectory_tracking.git
catkin build
```

## Basic Usage

To run the trajectory tracking algorithm write your own launch file and place it inside `launch` folder. We provide you with an example of the launch file which is located in folder launch and named `ugv.launch`. To launch the launch file simply run command:

```
roslaunch ugv_trajectory_tracking gvr_bot.launch
```

## Nodes

### ugv_control

This node provides position control for an UGV. It has 2 control types:

* `0` MANUAL - the velocities from `/cmd_vel_ref` topic are just forwarded to output. Mode without position control.
* `1` POSITION CONTROL - position control mode.  

**Subscriptions:**

- ``` /pose  [geometry_msgs/PoseStamped]``` -> measured pose of an UGV. (if `sim == 0`)
- ``` /gazebo/model_states  [gazebo_msgs/ModelStates]``` -> measured pose of an UGV. (if `sim == 1`)
- ``` /reference  [trajectory_msgs/MultiDOFJointTrajectory]``` -> position reference of an UGV
- ``` /cmd_vel_ref  [geometry_msgs/Twist]``` -> if position control is disabled this value will be forwarded to output.
- ``` /control_type  [std_msgs/Int16]``` -> if `0` position control is disabled, if `1` position control is enabled.

**Published topics:**

- ``` /computed_control_actions [geometry_msgs/Twist]``` -> computed commands for an UGV

**Parameters:**

- ``` Kp_v``` -> Gain for the linear velocity controller
- ``` Kp_w ``` -> Gain for the angular velocity controller
- ``` v_limit``` -> maximal linear velocity
- ``` w_limit ``` -> maximal angular velocity
- ``` sensitivity ``` -> allowed position error
- ``` rate ``` -> rate of the controller in hz
- ``` sim ``` -> parameter which has to be set to `1` if you are using node in simulation or `0` if you are using in real life experiments.

## ugv_commander

This node provides you with ugv control using gamepad. It has four diferent modes:

* `0` MANUAL mode - this mode mappes the gamepad inputs to linear and angular velocities. (default mode)
* `1` CARROT mode - this mode generates the reference position for an UGV based on measured position and parameter `carrot_v`. By moving the sticks you control the position of the vehicle relative to its measured position.
* `2` TRAJECTORY mode - in this mode the trajectory which is recived on topic `trajectory` will be executed. Node that is necessarily to set correct parameters `trajectory_rate`.
* `3` POINT mode - this mode sends an UGV in position recieved on topic `pose_ref`.

**Subscriptions:**

- ``` /pose  [geometry_msgs/PoseStamped]``` -> measured pose of an UGV. (if `sim == 0`)
- ``` /gazebo/model_states  [gazebo_msgs/ModelStates]``` -> measured pose of an UGV. (if `sim == 1`)
- ``` /joy  [sensor_msgs/Joy]``` -> inputs from a gamepad
- ``` /trajectory  [trajectory_msgs/MultiDOFJointTrajectory]``` -> trajectory for an UGV.
- ``` /pose_ref  [geometry_msgs/Pose]``` -> reference position of an UGV.

**Published topics:**

- ``` /cmd_vel_ref [geometry_msgs/Twist]``` -> reference linear and angular velocities for an UGV.
- ``` /control_type [std_msgs/Int16]``` -> mode of the ugv control
- ``` /reference [trajectory_msgs/MultiDOFJointTrajectory]``` -> position reference of an UGV.

**Parameters:**
- ``` carrot_v ``` -> parameter for the `CARROT` mode. 
- ``` trajectory_rate ``` -> sampling rate of the input trajectory.
- ``` rate ``` -> rate of the controller in hz
- ``` sim ``` -> parameter which has to be set to `1` if you are using node in simulation or `0` if you are using in real life experiments.

### Gamepad commands
The ugv_commander modes are mapped to the following elements in `buttons` list:

* `0` - POINT mode
* `4` - CARROT mode
* `5` - MANUAL mode (default mode)
* `8` - TRAJECTORY mode

If you are in manual mode the linear and angular velocities are mapped to the following elements in `axes` list:

* `0` - angular velocity
* `4` - linear velovity

For CARROT mode the x and y positions are mapped to the following elements in `axes` list:

* `0` - y position
* `4` - x position






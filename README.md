# UGV trajectory tracking

This repository contains code for position control of an UGV. 

## Instalation

Just clone the repository inside the ros workspace and use catkin build command to build it. See example below:

```
cd ~/catkin_ws/src/
git clone https://github.com/larics/ugv_trajectory_tracking.git
catkin build
```

## Basic Usage

To run the trajectory tracking algorithm write your own launch file and place it inside `launch` folder. We provide you with an example of the launch file which is located in folder launch and named `gvr_bot.launch`. To launch the launch file simply run command:

```
roslaunch ugv_trajectory_tracking gvr_bot.launch
```

## Nodes

### gvr_bot_control

This node provides position control for an UGV. It has 2 control types:

* `0` - the velocities from `/cmd_vel_ref` topic are just forwarded to output. Mode without position control.
* `1` - position control mode.  

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

# ROS_UnityArm
ROS connection between simulation and manipulator arm 

(Currently working)

Launch UR3 arm in Gazebo
```
(Change to Catkin Workspace)
$ source devel/setup.bash
$ roslaunch ur3_gazebo ur_cubes.launch ur_robot:=ur3 grasp_plugin:=1
```


# Subscribe to the Unity messages
The simulation publishes the Joint angles and now we subscribe to them for controlling a real robot
```python
import rospy
import time
from unity_robotics_demo_msgs.msg import ActionValue
g_list = []
def callback(msg):
    lis = []
    lis.append(msg.a)
    lis.append(msg.b-1.57)
    lis.append(msg.c)
    lis.append(msg.d-1.57)
    lis.append(msg.e)
    lis.append(msg.f)
    g_list.append(lis)
    print(lis)
    
    #print('BIG LIST',g_list))
    

rospy.init_node("extractactions")
sub = rospy.Subscriber('Action_Val', ActionValue, callback)

rospy.spin()
```
The Gazebo simulation has the following rostopics
```
/clock
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/gripper_controller/gripper_cmd/cancel
/gripper_controller/gripper_cmd/feedback
/gripper_controller/gripper_cmd/goal
/gripper_controller/gripper_cmd/result
/gripper_controller/gripper_cmd/status
/joint_states
/rosout
/rosout_agg
/scaled_pos_joint_traj_controller/command
/scaled_pos_joint_traj_controller/follow_joint_trajectory/cancel
/scaled_pos_joint_traj_controller/follow_joint_trajectory/feedback
/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal
/scaled_pos_joint_traj_controller/follow_joint_trajectory/result
/scaled_pos_joint_traj_controller/follow_joint_trajectory/status
/scaled_pos_joint_traj_controller/state
/tf
/tf_static
```
We then select the following rostopic to simulate the robotic arm using the joint angles derived from the simulation
```
arm_client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
```

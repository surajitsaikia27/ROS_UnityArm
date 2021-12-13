# ROS_UnityArm
ROS connection between simulation and manipulator arm


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

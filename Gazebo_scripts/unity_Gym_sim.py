"""
Testing program to check the robot movement using actionlib
"""

import rospy
import copy
import time
import math
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
traj = [[2.3168, -1.7997, -1.7772, 0.3415, 2.3669, 3.1805],
        [2.3168, -1.9113, -1.8998, 0.5756, 2.3669, 3.1805],
        [2.4463, -1.9799, -1.7954, 0.5502, 2.2378, 3.1960],
         [2.5501, -2.0719, -1.6474, 0.5000, 2.1344, 3.2062],
        [1.57000018,-1.57000679,1.26000021,-1.57000555,-1.56999797,-314.15924777],
         [3.1400018,-1.57000679,1.26000021,-1.57000555,-1.56999797,-314.15924777], 
        [-3.1400018,-1.57000679,1.26000021,-1.57000555,-1.56999797,-314.15924777]]
i_gazebo  = [0, -1.57, 0.0, -1.57, 0.0, 0.0]
#i_gazebo = [0, 0, 0.0, 0, 0.0, 0.0]
# RESET POSITION

rospy.init_node("controller")

reset = rospy.get_param('~reset', False)

arm_joints=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

rospy.loginfo('Waiting for right arm trajectory controller...')
    

# rospy.loginfo('...connected.')
arm_client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

arm_client.wait_for_server()
done=False

f = open("Angles.txt",'r')
lines = f.readlines()
# while not done and not rospy.is_shutdown():
for points in lines:
        points = points.strip()
        points = points.split(',')
        print('1',points)
        points = [ (math.radians(float(x))+i_gazebo[i]) for i,x in enumerate(points)]
        print('2',points)
   
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = points
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
        
            
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
                
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
                
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(3.0)
            
        # Send the goal to the action server
        arm_client.send_goal(arm_goal)
        time.sleep(5)
        rospy.loginfo('...done')

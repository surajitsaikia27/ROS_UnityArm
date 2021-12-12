import math


f= open("Angles.txt",'r')
lines = f.readlines()
# while not done and not rospy.is_shutdown():
for points in lines:
        points = points.strip()
        points = points.split(',')
        points = [ math.radians(float(x))for i,x in enumerate(points)]
        print(points)
       
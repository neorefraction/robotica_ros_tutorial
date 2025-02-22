#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Global variables
robot_x, robot_y = 5.544445, 5.544445 # origin position

def update_pose(pose: Pose) -> None:
    # Retrieves global variables
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Robot X = %f\t Robot Y = %f\n",pose.x, pose.y)

def isValidPosition(horizontal_limit: float, vertical_limit: float) -> bool:
    if robot_x > (5.544445 + horizontal_limit):
        return False
    if robot_x < (5.544445 - horizontal_limit):
        return False
    if robot_y > (5.544445 + vertical_limit):
        return False
    if robot_y < (5.544445 - vertical_limit):
        return False
    return True

def move_turtle_controlled(
    linear_velocity: float,
    angular_velocity: float,
    horizontal_limit: float,
    vertical_limit: float
) -> None:

    # Retrieves global variables
    global robot_x, robot_y
    
    # Set up node
    publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    rate = rospy.Rate(10) # 10hz
    
    # Message to send the linear and angular speed to the turtlesim_node
    velocity = Twist()

    while not rospy.is_shutdown():
        velocity.linear.x = -linear_velocity
        velocity.linear.y = 0
        velocity.linear.z = 0
        velocity.angular.x = 0
        velocity.angular.y = 0
        velocity.angular.z = angular_velocity

        if not isValidPosition(horizontal_limit, vertical_limit):
            rospy.loginfo('Robot hits a wall')
            rospy.logwarn('Stopping robot')
            break

        publisher.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtle_controlled')
        v = rospy.get_param('~v')
        w = rospy.get_param('~w')
        x = rospy.get_param('~x')
        y = rospy.get_param('~y')
        move_turtle_controlled(v, w, x, y)
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'An unexpected error happend: {e}')
    
 

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

ORIGIN = 5.544445

# Global variables
robot_x, robot_y = ORIGIN, ORIGIN

def update_pose(pose: Pose) -> None:
    '''
    Updates turtle's position

    Params:
    -------
    pose: turtlesim.msg.Pose
        ROS turtlesim Pose mesage (https://docs.ros.org/en/jade/api/turtlesim/html/msg/Pose.html)
    '''

    # retrieves and updates global variables
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y


def isValidPosition(horizontal_limit: float, vertical_limit: float) -> bool:
    '''
    Check if turtle is within boundaries

    Params:
    -------
    horizontal_limit: float
        Distance from center to X axis limits (forward and backward)
    vertical_limit: float
        Distance from center to Y axis limits (up, down)
    '''
    
    if robot_x > (ORIGIN + horizontal_limit):
        return False
    if robot_x < (ORIGIN - horizontal_limit):
        return False
    if robot_y > (ORIGIN + vertical_limit):
        return False
    if robot_y < (ORIGIN - vertical_limit):
        return False
    return True

def move_turtle_controlled(velocity: dict, limits: dict) -> None:
    '''
    Moves the turtle randomly utill reach a limit

    Params:
    -------
    velocity: dict
        Dicctionary with the values of the linear and angular velocities
        - linear: float
            Value for turtle linear velocity
        - angular: float
            Value for turtle angular velocity

    limits: dict
        Dicctionary with the values of the horizontal and vertical limits
        - horizontal: float
            Value with X axis limits from center (forward and backward)
        - vertical: float
            Value with Y axis limits from center (up and down)
    '''

    # Retrieves global variables
    global robot_x, robot_y
    
    # Set up node
    publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    rate = rospy.Rate(10) # 10hz
    
    # Sets velocity message for the node
    move = Twist()
    move.linear.y = -velocity['linear']
    move.angular.z = velocity['angular']

    while not rospy.is_shutdown():

        if not isValidPosition(limits['horizontal'], limits['vertical']):
            publisher.publish(Twist())
            rospy.loginfo('Robot hits a wall')
            rospy.logwarn('Stopping robot')
            break

        publisher.publish(move)
        rate.sleep()
        
    # If we press control + C, the node will stop.
    rospy.spin()

if __name__ == '__main__':
    try:
        # Creates the node
        rospy.init_node('move_turtle_controlled')

        # Fetches variables from launch file
        v = rospy.get_param('~v', 1)
        w = rospy.get_param('~w', 1)
        x = rospy.get_param('~x', 2)
        y = rospy.get_param('~y', 2)

        # Ease the params use
        velocity = {'linear': v, 'angular': w}
        limits = {'horizontal': x, 'vertical': y}

        # Starts node functionality
        move_turtle_controlled(velocity, limits)

    except rospy.ROSInterruptException as e:
        rospy.logerr(f'An unexpected error happend: {e}')
    
 

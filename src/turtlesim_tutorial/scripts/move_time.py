#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import string

# Global variables
robot_x, robot_y = 5.544445, 5.544445 # Origin position


def update_pose(pose: Pose) -> None:
    '''
    Updates turtle's global pose variables

    Params:
    -------
    pose: turtlesim.msg.Pose
        ROS turtlesim Pose mesage (https://docs.ros.org/en/jade/api/turtlesim/html/msg/Pose.html)
    '''

    # retrieves and updates global variables
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y


def update_velocity(velocity: Twist, linear_velocity: float, angular_velocity: float) -> None:
    '''
    Updates turtle's velocity

    '''

    velocity.linear.x = linear_velocity
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = angular_velocity


def move_turtle_timed(linear_velocity: float, angular_velocity: float, time_interval: int) -> None:
    '''
    Moves the turtle for a time interval

    Params:
    -------
    linear_velocity: float
        Value for turtlesim linear velocity for the x axis
    angular_velocity: float
        Value for turtlesim angular velocity for the z axis
    time: int
        Time interval in secods
    '''

    # retrieves global variables
    global robot_x, robot_y

    # connects the node to turtle1/cmd_vel topic to send Twist messages for turtlesim
    publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # connects the node to turtle1/pose topic to read Pose messages from turtlesim
    rospy.Subscriber('/turtle1/pose', Pose, update_pose)
    # sets node read/publish process to 10Hz
    rate = rospy.Rate(10)

    # defines a Twist message to update turtlesim velocity
    velocity = Twist()
    c = 1
    # defines a timer start
    start = rospy.Time.now()

    # node main loop
    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start).to_sec()
        
        if(elapsed_time > time_interval):
            rospy.loginfo(f'Time elapsed: {elapsed_time}')
            rospy.loginfo('Timer finished\nStopping robot')
            return
        
        rospy.loginfo(f'Time elapsed: {elapsed_time}')
        update_velocity(velocity, linear_velocity, angular_velocity * c)
        publisher.publish(velocity)
        c += 10
        c *= -1
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node(f'move_time', anonymous=False)
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        t= rospy.get_param("~t")
        move_turtle_timed(v,w,t)
    except rospy.ROSInterruptException:
        pass
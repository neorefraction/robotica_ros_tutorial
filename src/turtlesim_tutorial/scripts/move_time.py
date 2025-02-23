#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Global variables
robot_x, robot_y = 5.544445, 5.544445 # Origin position


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


def draw_heart(velocity: Twist, t: int) -> None:
    '''
    Calculates turtle's velocity to draw a Heart

    Params:
    -------
    velocity: geometry_msgs.msg
        ROS velocity message (https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
    t: int
        Elapsed time in secods
    '''
    
    # Calculates the linear velocity for the X and Y axes based on the derivatives of the parametric heart equations:
    # x(t) = 16sin(t)^3
    # y(t) = 13cos(t) - 5cos(2t) - 2cos(3t) - cos(4t)
    v_x = 48 * math.sin(t)**2 * math.cos(t)
    v_y = -13 * math.sin(t) + 10 * math.sin(2*t) + 6 * math.sin(3*t) + math.sin(4*t)

    # used to adjust units to turtlesim window
    c = 0.2

    # Updates velocity
    velocity.linear.x = v_x * c
    velocity.linear.y = v_y * c


def move_turtle_timed(time_interval: int) -> None:
    '''
    Moves the turtle given a linear and angular velocity for a time interval

    Params:
    -------
    linear_velocity: float
        Turtlesim linear velocity for the X axis
    angular_velocity: float
        Turtlesim angular velocity for the Z axis
    time: int
        Time interval in seconds
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
    # defines a timer start
    start = rospy.Time.now()

    # node main loop
    while not rospy.is_shutdown():

        elapsed_time = (rospy.Time.now() - start).to_sec()
        
        # Stops robot
        if(elapsed_time > time_interval):
            publisher.publish(Twist())
            rospy.loginfo(f'Time elapsed: {round(elapsed_time, 2)}')
            rospy.loginfo('Timer finished\nStopping robot')
            return
        
        # Updates robot velocity
        rospy.loginfo(f'Time elapsed: {round(elapsed_time, 2)}')
        draw_heart(velocity, elapsed_time)
        publisher.publish(velocity)
        rate.sleep()

    # If we press control + C, the node will stop.
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node(f'move_time', anonymous=False)
        t= rospy.get_param("~t")
        move_turtle_timed(t)
    except rospy.ROSInterruptException:
        pass
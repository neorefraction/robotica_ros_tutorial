#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from geometry_msgs.msg import Twist


class TurtleBot:

    def __init__(self):
        '''
        Initialize turtle bot
        '''

        self.init_node()
        self.init_attributes()


    def init_attributes(self) -> None:
        '''
        Initialize bot attributes

        Attributes:
        -----------
        bot_pose: turtlesim.msg.Pose
            Tracks bot's position
        goal_pose: turtlesim.msg.Pose
            Defines bot goal position and orientation
        rate: rospy.Rate
            Defines the bot reading/writing rate
        
        '''

        # Poses values
        self.bot_pose = Pose()
        self.goal_pose = Pose()
        self.goal_pose.x = rospy.get_param('~x', 6)
        self.goal_pose.y = rospy.get_param('~y', 8)
        self.goal_pose.theta = rospy.get_param('~theta', 20)

        # Tolerances value
        self.distance_tolerance = rospy.get_param("~l_tol", 0.1)
        self.angle_tolerance = rospy.get_param("~a_tol", 0.1)


    def init_node(self) -> None:
        '''
        Initialize node
        '''

        # Creates the turtle bot node
        rospy.init_node('move_turtle', anonymous=True)
        self.rate = rospy.Rate(10)
        # Creates a publisher object to send Twist messages to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # Subscribe the node to the '/turtle1/pose' topic to read turtle's position
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)


    def update_pose(self, pose: Pose):
        '''
        Updates turtle pose and round values to 4 decimals

        Params:
        -------
        pose: turtlesim.msg.Pose
            ROS turtlesim Pose mesage (https://docs.ros.org/en/jade/api/turtlesim/html/msg/Pose.html)
        '''

        self.bot_pose = pose
        self.bot_pose.x = round(self.bot_pose.x, 4)
        self.bot_pose.y = round(self.bot_pose.y, 4)

    def euclidean_distance(self, goal_pose):
        '''
        Calculates Euclidean distance between current pose and the goal one

        Params:
        -------
        goal_pose: turtlesim.msg.Pose (https://docs.ros.org/en/jade/api/turtlesim/html/msg/Pose.html)
            Turtle's final pose
        
        Returns:
        --------
        float:
            Value with the euclidean distance between turtle's current pose and goal one
        '''

        return sqrt((goal_pose.x - self.bot_pose.x)**2 + (goal_pose.y - self.bot_pose.y)**2)

    def linear_vel(self, goal_pose, constant=1.5):
        '''
        Calculates turtle's linear velocity taking euclidean distance as unit and a constant to calibrate
        See video: https://www.youtube.com/watch?v=Qh15Nol5htM.

        Params:
        -------
        goal_pose: turtlesim.msg.Pose (https://docs.ros.org/en/jade/api/turtlesim/html/msg/Pose.html)
            Turtle's final pose
        constant: float
            Value used to calibrate velocity within turtle's field

        Returns:
        --------
        float:
            Value with turtle's linear velocity
        '''

        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.bot_pose.y, goal_pose.x - self.bot_pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.bot_pose.theta)

    def move2goal(self):
        '''
        Moves the turtle to the goal position and turn it to the goal direction
        '''

        velocity_message = Twist()

        while self.euclidean_distance(self.goal_pose) >= self.distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            velocity_message.linear.x = self.linear_vel(self.goal_pose)

            # Angular velocity in the z-axis.
            velocity_message.angular.z = self.angular_vel(self.goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(velocity_message)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping robot at linear velcoity
        velocity_message.linear.x = 0
        self.velocity_publisher.publish(velocity_message)

        while abs(self.goal_pose.theta - self.bot_pose.theta) >= self.angle_tolerance:
            velocity_message.angular.z = 1
            # Publishing our vel_msg
            self.velocity_publisher.publish(velocity_message)
            # Publish at the desired rate.
            self.rate.sleep()
        
        # Stopping robot at angular velocity
        velocity_message.angular.z = 0
        self.velocity_publisher.publish(velocity_message)

        # Log info
        rospy.loginfo("Robot Reached destination")
        rospy.logwarn("Stopping robot")

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        turtle = TurtleBot()
        turtle.move2goal()
    except rospy.ROSInterruptException:
        pass
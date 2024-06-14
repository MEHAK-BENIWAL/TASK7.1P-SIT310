#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import AprilTagDetectionArray

class RobotFollower:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('robot_follower_node', anonymous=True)

        # Register clean-up function for node shutdown
        rospy.on_shutdown(self.clean_shutdown)

        # Initialize publisher for velocity commands
        self.velocity_publisher = rospy.Publisher('/duckie/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)

        # Initialize subscriber for AprilTag detections
        self.tag_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.on_tag_detected)
        
        rospy.spin() # Keep the node running and responsive to callbacks

    # Callback function for AprilTag detections
    def on_tag_detected(self, msg):
        self.navigate_robot(msg.detections)

    # Clean shutdown procedure
    def clean_shutdown(self):
        rospy.loginfo("Shutting down. Stopping robot...")
        self.halt_robot()

    # Function to halt the robot
    def halt_robot(self):
        stop_msg = Twist2DStamped()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.v = 0.0
        stop_msg.omega = 0.0
        self.velocity_publisher.publish(stop_msg)

    # Main robot navigation logic based on tag detections
    def navigate_robot(self, detections):
        if not detections:
            self.halt_robot()
            return
        
        self.halt_robot()
        
        position = detections[0].transform.translation
        x = position.x
        y = position.y
        z = position.z

        rospy.loginfo("Position - x: %f, y: %f, z: %f", x, y, z)
        rospy.sleep(1)

        if z > 0.15:
            self.send_velocity(0.2, 0.0)
        elif z < 0.20:
            self.send_velocity(-0.4, 0.0)
        elif x > 0.05:
            self.send_velocity(0.0, -0.4)
        elif x < -0.05:
            self.send_velocity(0.0, 0.4)

    # Function to send velocity commands
    def send_velocity(self, linear_vel, angular_vel):
        vel_msg = Twist2DStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.v = linear_vel
        vel_msg.omega = angular_vel
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(0.2)
        self.halt_robot()

if __name__ == '__main__':
    try:
        robot_follower = RobotFollower()
    except rospy.ROSInterruptException:
        pass

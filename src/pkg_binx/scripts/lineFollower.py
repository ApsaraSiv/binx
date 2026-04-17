#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# subscribe to the camera image that gazebo publishes using the gz ros bridge
# performs an opencv process and gives and output
# publishes velocity commands to move forwards while simultaenously correcting trajectory

# ways this script could be improved: PID control, bounding boxes or any visual cue. 

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image', 
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def listener_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        error = self.detect_line(frame)
        self.velocity_commands(error)
        cv2.imshow("Sim Camera", frame)
        cv2.waitKey(1)


    # operncv stuff
    def detect_line(self,frame):

        h, w = frame.shape[:2]
        roi = frame[int(h*0.65):h, :]
        hsv = cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)

        lower = np.array([0,0,0])
        upper = np.array([180,255,80])

        mask = cv2.inRange(hsv, lower, upper)

        M = cv2.moments(mask)
        if M['m00'] > 500:
            cx = int(M['m10']/M['m00'])
            error = cx - (w//2)
            return error
        
        return None


    # velocity commands
    def velocity_commands(self, error):
        twist = Twist()
        kp = 0.01

        if error is not None:
            twist.linear.x = 0.5
            twist.angular.z = -kp*error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 3.5

        self.publisher.publish(twist)


        
def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    try:
        rclpy.spin(line_follower)
    finally:
        line_follower.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows() 

if __name__ == '__main__':
    main()
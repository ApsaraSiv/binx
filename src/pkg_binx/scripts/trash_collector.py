#!/usr/bin/env python3
import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


BIN_POSES = {
    'red':   (11.4, -3.0),
    'blue':  (11.4,  0.0),
    'green': (11.4,  3.0),
}
DETECTION_POINT = (0.5, 0.0)   #robot stops here, camera looks forward at trash


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.color = None
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image', self._cb, 10)

    def _cb(self, msg):
        if self.color is not None:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        masks = {
            'red':   cv2.bitwise_or(
                        cv2.inRange(hsv, np.array([0,   150, 70]), np.array([10,  255, 255])),
                        cv2.inRange(hsv, np.array([170, 150, 70]), np.array([180, 255, 255]))),
            'blue':  cv2.inRange(hsv, np.array([100, 150, 70]), np.array([130, 255, 255])),
            'green': cv2.inRange(hsv, np.array([40,  150, 70]), np.array([80,  255, 255])),
        }
        counts = {c: cv2.countNonZero(m) for c, m in masks.items()}
        best = max(counts, key=counts.get)

        if counts[best] > 300:
            self.color = best
            self.get_logger().info(f'Detected: {best} ({counts[best]} px)')


def make_pose(nav, x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def navigate_to(nav, pose, label):
    nav.get_logger().info(f'Navigating to {label}...')
    nav.goToPose(pose)
    while not nav.isTaskComplete():
        pass
    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        nav.get_logger().info(f'Reached {label}.')
        return True
    nav.get_logger().error(f'Failed to reach {label}.')
    return False


def main():
    rclpy.init()
    nav = BasicNavigator()
    detector = ColorDetector()

    det_executor = SingleThreadedExecutor()
    det_executor.add_node(detector)
    det_thread = threading.Thread(target=det_executor.spin, daemon=True)
    det_thread.start()

    nav.get_logger().info('Waiting for Nav2...')
    nav.waitUntilNav2Active()

    # Stop at a placeholder detection point and trash is ahead of the camera 
    if not navigate_to(nav, make_pose(nav, *DETECTION_POINT), 'detection point'):
        rclpy.shutdown()
        return

    # wait up to 3s for color detection
    deadline = time.time() + 3.0
    while detector.color is None and time.time() < deadline:
        time.sleep(0.1)

    if detector.color is None:
        nav.get_logger().error('Could not detect trash color — aborting')
        rclpy.shutdown()
        return

    color = detector.color
    nav.get_logger().info(f'Trash is {color.upper()} — collecting...')
    time.sleep(1.0)

    bin_x, bin_y = BIN_POSES[color]
    if not navigate_to(nav, make_pose(nav, bin_x, bin_y), f'{color} bin'):
        rclpy.shutdown()
        return

    nav.get_logger().info(f'Mission complete — deposited in {color.upper()} bin.')
    rclpy.shutdown()


if __name__ == '__main__':
    main()

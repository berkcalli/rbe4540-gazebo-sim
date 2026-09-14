"""Simple example: receive camera images and send poses and velocities."""

import time

import rclpy
from common_interfaces_merlab.srv import SendPose, SendTwist
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SimpleRun(Node):
    def __init__(self):
        super().__init__('simple_run')
        self.cartesian_client = self.create_client(SendPose, '/cartesian_ref')
        self.ee_velocity_client = self.create_client(SendTwist, '/set_ee_velocity')
        self.bridge = CvBridge()
        self.palm_image = None  # Latest OpenCV image; None until one arrives.
        self.palm_camera_subscriber = self.create_subscription(
            Image,
            '/palm_camera/image',
            self.palm_image_callback,
            qos_profile_sensor_data,
        )

    def palm_image_callback(self, msg):
        """Convert the ROS image to an OpenCV image (BGR NumPy array)."""
        self.palm_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Add your image processing here.

    def move_cartesian(self, x, y, z):
        """Move tool0 to a position in base_link (meters), pointing downward."""
        request = SendPose.Request()
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        # Quaternion (x, y, z, w) = (1, 0, 0, 0).
        request.pose.orientation.x = 1.0
        request.pose.orientation.w = 0.0
        request.pose.orientation.y = 0.0
        request.pose.orientation.z = 0.0

        self.get_logger().info(f'Moving to ({x:.2f}, {y:.2f}, {z:.2f})')
        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)
        if not future.done():
            self.get_logger().error('No motion response; stopping the motion sequence')
            return False
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error('Cartesian motion failed; stopping the motion sequence')
            return False
        return True

    def set_ee_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                        wx=0.0, wy=0.0, wz=0.0):
        """Set tool-frame linear (m/s) and angular (rad/s) velocities."""
        request = SendTwist.Request()
        request.twist.linear.x = float(vx)
        request.twist.linear.y = float(vy)
        request.twist.linear.z = float(vz)
        request.twist.angular.x = float(wx)
        request.twist.angular.y = float(wy)
        request.twist.angular.z = float(wz)
        future = self.ee_velocity_client.call_async(request)
        # The first command also starts Servo through the motion interface.
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().error('No velocity response; stopping the motion sequence')
            return False
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else 'No response'
            self.get_logger().error(f'Velocity command failed: {message}')
            return False
        return True

    def move_ee_velocity(self, vx=0.0, vy=0.0, vz=0.0,
                         wx=0.0, wy=0.0, wz=0.0, duration=1.0):
        """Refresh a tool-frame velocity for duration seconds; caller stops it."""
        self.get_logger().info(
            f'Tool-frame velocity: linear=({vx}, {vy}, {vz}) m/s, '
            f'angular=({wx}, {wy}, {wz}) rad/s for {duration:.1f} s'
        )
        if not self.set_ee_velocity(vx, vy, vz, wx, wy, wz):
            return False
        end_time = time.monotonic() + duration
        while rclpy.ok():
            remaining = end_time - time.monotonic()
            if remaining <= 0.0:
                return True
            # Refresh before the interface's default 0.5-second watchdog expires.
            time.sleep(min(0.1, remaining))
            if time.monotonic() >= end_time:
                return True
            if not self.set_ee_velocity(vx, vy, vz, wx, wy, wz):
                return False
        return False

    def run(self):
        self.get_logger().info('Waiting for /cartesian_ref...')
        while rclpy.ok():
            if self.cartesian_client.wait_for_service(timeout_sec=1.0):
                break
        if not rclpy.ok():
            return

        # TODO: Implement your homework here. Edit or extend these two moves.
        # Each call waits for the robot to finish before continuing.
        # Images are received while the motion methods spin waiting for replies.
        # To receive images outside those methods, call rclpy.spin_once(self).
        if not self.move_cartesian(0.45, 0.0, 0.54):
            return
        if not self.move_cartesian(0.45, -0.15, 0.54):
            return

        self.get_logger().info('Waiting for /set_ee_velocity...')
        while rclpy.ok():
            if self.ee_velocity_client.wait_for_service(timeout_sec=1.0):
                break
        if not rclpy.ok():
            return

        # Move along tool0's +X, then -X (about 9 cm each at 0.03 m/s).
        # Switch directly between velocities; send zero when the sequence ends.
        try:
            if not self.move_ee_velocity(vx=0.03, duration=3.0):
                return
            if not self.move_ee_velocity(vx=-0.03, duration=3.0):
                return
        finally:
            # Also request a stop if a command fails or the user interrupts.
            # If ROS has shut down, the interface watchdog stops stale commands.
            stopped = self.set_ee_velocity() if rclpy.ok() else False
        if not stopped:
            return
        self.get_logger().info('Motion sequence complete')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleRun()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
vo_odom_relay
-------------
Sits between the visual-odometry front-end (rtabmap_ros rgbd_odometry,
publishing on /rtabmap/odom) and robot_localization's ekf_node.

Two jobs:
  1. Attach realistic, non-zero covariance to the VO pose/twist so the EKF
     weights it sensibly instead of treating an unspecified covariance as
     perfect certainty.
  2. Detect likely tracking loss (a sudden large jump between consecutive
     VO poses, or a snap back to the origin, which is how rtabmap reports
     "lost track" on some versions) and inflate covariance to effectively
     tell the EKF "ignore this sample" rather than let a bad VO pose yank
     the filter state around.

This keeps the EKF itself simple (delegated to robot_localization) while
still doing meaningful signal-conditioning work in rclpy.
"""
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

POS_VAR_DEFAULT = 0.01      # m^2  (~10 cm std dev - reasonable for RGB-D VO)
YAW_VAR_DEFAULT = 0.02      # rad^2
UNTRUSTED_VAR = 1.0e6       # effectively tells the EKF to ignore this axis/sample

MAX_JUMP_M = 0.5            # max plausible per-message VO position jump


class VoOdomRelay(Node):
    def __init__(self):
        super().__init__('vo_odom_relay')
        self.declare_parameter('input_topic', '/rtabmap/odom')
        self.declare_parameter('output_topic', '/vo/odom')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.sub = self.create_subscription(Odometry, in_topic, self.callback, 10)
        self._last_xy = None
        self.get_logger().info(f'Relaying {in_topic} -> {out_topic}')

    def callback(self, msg: Odometry):
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.child_frame_id

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        lost = False

        if self._last_xy is not None:
            dx, dy = x - self._last_xy[0], y - self._last_xy[1]
            if math.hypot(dx, dy) > MAX_JUMP_M:
                lost = True
            # rtabmap can report an identity/zero pose on total tracking loss;
            # treat a snap back to the origin after having moved as a loss too.
            if x == 0.0 and y == 0.0 and (abs(self._last_xy[0]) > 1e-3 or abs(self._last_xy[1]) > 1e-3):
                lost = True

        pos_var = UNTRUSTED_VAR if lost else POS_VAR_DEFAULT
        yaw_var = UNTRUSTED_VAR if lost else YAW_VAR_DEFAULT

        # Row-major 6x6 covariance: [x,y,z,roll,pitch,yaw]
        cov = [0.0] * 36
        cov[0] = pos_var          # x
        cov[7] = pos_var          # y
        cov[14] = pos_var         # z
        cov[21] = UNTRUSTED_VAR   # roll  - not reliably observable from a ground-robot RGB-D VO
        cov[28] = UNTRUSTED_VAR   # pitch
        cov[35] = yaw_var         # yaw
        msg.pose.covariance = cov
        msg.twist.covariance = cov  # reuse the same weighting for velocity estimates

        if not lost:
            self._last_xy = (x, y)
        elif self._last_xy is None:
            self._last_xy = (x, y)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

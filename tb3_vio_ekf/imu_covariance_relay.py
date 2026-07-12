#!/usr/bin/env python3
"""
imu_covariance_relay
---------------------
Relays /imu -> /imu/data, filling in covariance matrices whenever the
Gazebo IMU plugin publishes them as all-zero.

Why this exists: robot_localization's ekf_node uses the covariance on
each measurement to weight it in the fusion. Gazebo's imu sensor plugin
frequently publishes all-zero covariance blocks, which some consumers
interpret as "perfectly known" (infinite confidence) rather than
"unknown". Left uncorrected, that can make the filter trust noisy raw
IMU data far more than it should. This node fills in a reasonable static
covariance when it detects an all-zero block.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Tune these to your simulated IMU's actual noise characteristics if known.
ORIENTATION_COV = [0.001, 0.0, 0.0,
                    0.0, 0.001, 0.0,
                    0.0, 0.0, 0.001]
ANGULAR_VEL_COV = [0.0003, 0.0, 0.0,
                    0.0, 0.0003, 0.0,
                    0.0, 0.0, 0.0003]
LINEAR_ACC_COV = [0.01, 0.0, 0.0,
                   0.0, 0.01, 0.0,
                   0.0, 0.0, 0.01]


class ImuCovarianceRelay(Node):
    def __init__(self):
        super().__init__('imu_covariance_relay')
        self.declare_parameter('input_topic', '/imu')
        self.declare_parameter('output_topic', '/imu/data')

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(Imu, out_topic, 10)
        self.sub = self.create_subscription(Imu, in_topic, self.callback, 10)
        self.get_logger().info(f'Relaying {in_topic} -> {out_topic} with sanitized covariances')

    def callback(self, msg: Imu):
        if all(c == 0.0 for c in msg.orientation_covariance):
            msg.orientation_covariance = ORIENTATION_COV
        if all(c == 0.0 for c in msg.angular_velocity_covariance):
            msg.angular_velocity_covariance = ANGULAR_VEL_COV
        if all(c == 0.0 for c in msg.linear_acceleration_covariance):
            msg.linear_acceleration_covariance = LINEAR_ACC_COV
        if not msg.header.frame_id:
            msg.header.frame_id = 'imu_link'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceRelay()
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

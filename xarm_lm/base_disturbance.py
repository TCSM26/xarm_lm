#!/usr/bin/env python3
"""
Base Disturbance — simulates a moving robot base by publishing a
time-varying world->link_base TF transform.

Motion pattern:
  XY plane : lemniscate of Bernoulli (infinity / figure-8 shape)
      x(t) = A   * cos(θ)          / (1 + sin²(θ))
      y(t) = A   * sin(θ)*cos(θ)   / (1 + sin²(θ))      θ = ω·t
  Z axis   : sinusoidal oscillation
      z(t) = A_z * sin(ω_z·t)

Starts disabled. Enable/disable at runtime:
  ros2 topic pub --once /base_disturbance/enable std_msgs/msg/Bool "{data: true}"
  ros2 topic pub --once /base_disturbance/enable std_msgs/msg/Bool "{data: false}"

The dynamic TF on /tf overrides the static world->link_base published by the
moveit_fake launch, so no upstream files need to be modified.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
import tf2_ros


class BaseDisturbance(Node):
    def __init__(self):
        super().__init__('base_disturbance')

        self.declare_parameter('amplitude',    0.08)   # XY half-width  (m)
        self.declare_parameter('z_amplitude',  0.04)   # Z  half-height (m)
        self.declare_parameter('period',       8.0)    # XY loop period (s)
        self.declare_parameter('z_period',     4.0)    # Z  osc. period (s)
        self.declare_parameter('publish_rate', 50.0)   # TF publish rate (Hz)
        self.declare_parameter('world_frame',  'world')
        self.declare_parameter('base_frame',   'link_base')

        self.A       = self.get_parameter('amplitude').value
        self.A_z     = self.get_parameter('z_amplitude').value
        self.omega   = 2.0 * np.pi / self.get_parameter('period').value
        self.omega_z = 2.0 * np.pi / self.get_parameter('z_period').value
        self.world   = self.get_parameter('world_frame').value
        self.base    = self.get_parameter('base_frame').value

        self.enabled = False
        self.t0      = None

        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Bool, '~/enable', self.enable_callback, 1)

        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self.publish_tf)

        self.get_logger().info(
            'Base disturbance ready (disabled). '
            'Publish True to /base_disturbance/enable to start.'
        )

    def enable_callback(self, msg: Bool):
        if msg.data and not self.enabled:
            self.enabled = True
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            self.get_logger().info(
                f'Disturbance ENABLED — '
                f'xy_amp={self.A} m  z_amp={self.A_z} m  '
                f'xy_period={2*np.pi/self.omega:.1f} s  '
                f'z_period={2*np.pi/self.omega_z:.1f} s'
            )
        elif not msg.data and self.enabled:
            self.enabled = False
            self.get_logger().info('Disturbance DISABLED.')

    def publish_tf(self):
        now = self.get_clock().now()
        x, y, z = 0.0, 0.0, 0.0

        if self.enabled and self.t0 is not None:
            t     = now.nanoseconds * 1e-9 - self.t0
            theta = self.omega * t
            denom = 1.0 + np.sin(theta) ** 2
            x = self.A   * np.cos(theta) / denom
            y = self.A   * np.sin(theta) * np.cos(theta) / denom
            z = self.A_z * np.sin(self.omega_z * t)

        tf_msg = TransformStamped()
        tf_msg.header.stamp    = now.to_msg()
        tf_msg.header.frame_id = self.world
        tf_msg.child_frame_id  = self.base
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z
        tf_msg.transform.rotation.w    = 1.0

        self.broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BaseDisturbance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

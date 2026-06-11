#!/usr/bin/env python3
"""
Gaze Stabilizer — keeps the xArm6 end-effector fixed in world frame
while the robot base undergoes arbitrary disturbances.

Control loop (closed-loop, purely reactive):
  1. Lock desired EE pose T_des in world frame (on /gaze_stabilizer/start).
  2. Each tick: read current EE pose T_cur from TF.
  3. Compute pose error e = T_des ⊖ T_cur  (position + axis-angle orientation).
  4. Proportional controller: v_cmd = Kp * e  (Cartesian velocity in world frame).
  5. Rotate v_cmd into link_base frame.
  6. Publish TwistStamped to MoveIt Servo.

Levenberg-Marquardt IK (inside MoveIt Servo):
  The servo node solves the velocity-level IK at each tick using damped least-squares:
      dq = J^T (J J^T + lambda^2 I)^{-1} * v_cmd
  where lambda grows near singularities, ensuring stable (approximate) solutions
  instead of joint-velocity blow-up.

Topics
------
  Subscribe:  ~/start          (std_msgs/Bool)  — True = latch & start, False = stop
  Publish:    /servo_server/delta_twist_cmds  (geometry_msgs/TwistStamped)
  Publish:    /base_disturbance/enable        (std_msgs/Bool)

Services called
---------------
  /servo_server/start_servo  (std_srvs/Trigger)  — called automatically at startup
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation


class GazeStabilizer(Node):
    def __init__(self):
        super().__init__('gaze_stabilizer')

        self.declare_parameter('kp_linear',       16.0)
        self.declare_parameter('kp_angular',      12.0)
        self.declare_parameter('max_linear_vel',   0.5)
        self.declare_parameter('max_angular_vel',  2.0)
        self.declare_parameter('world_frame',     'world')
        self.declare_parameter('ee_frame',        'link_eef')
        self.declare_parameter('base_frame',      'link_base')
        self.declare_parameter('control_rate',    50.0)

        self.kp_lin  = self.get_parameter('kp_linear').value
        self.kp_ang  = self.get_parameter('kp_angular').value
        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.world   = self.get_parameter('world_frame').value
        self.ee      = self.get_parameter('ee_frame').value
        self.base    = self.get_parameter('base_frame').value
        rate         = self.get_parameter('control_rate').value

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.twist_pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.disturbance_pub = self.create_publisher(
            Bool, '/base_disturbance/enable', 1)

        self.enabled      = False
        self.desired_pos  = None
        self.desired_rot  = None
        self._servo_ready = False

        self._servo_client = self.create_client(Trigger, '/servo_server/start_servo')
        self._servo_timer  = self.create_timer(1.0, self._try_start_servo)

        self.create_subscription(Bool, '~/start', self.start_callback, 1)
        self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('=' * 55)
        self.get_logger().info('Gaze Stabilizer ready.')
        self.get_logger().info('Move the arm to the desired pose in RViz, then:')
        self.get_logger().info(
            'ros2 topic pub --once /gaze_stabilizer/start '
            'std_msgs/msg/Bool "{data: true}"'
        )
        self.get_logger().info('=' * 55)

    # ── Servo startup ────────────────────────────────────────────────────────

    def _try_start_servo(self):
        if self._servo_ready:
            self._servo_timer.cancel()
            return
        if not self._servo_client.wait_for_service(timeout_sec=0.1):
            return
        future = self._servo_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_servo_started)

    def _on_servo_started(self, future):
        try:
            result = future.result()
            if result.success:
                self._servo_ready = True
                self._servo_timer.cancel()
                self.get_logger().info('MoveIt Servo started.')
            else:
                self.get_logger().warn(f'start_servo: {result.message}')
        except Exception as e:
            self.get_logger().error(f'start_servo failed: {e}')

    # ── Start / stop ─────────────────────────────────────────────────────────

    def start_callback(self, msg: Bool):
        if msg.data and not self.enabled:
            pos, rot = self._lookup_tf(self.world, self.ee)
            if pos is None:
                self.get_logger().warn('Cannot latch: TF not available yet.')
                return
            self.desired_pos = pos.copy()
            self.desired_rot = rot
            self.enabled = True
            self.disturbance_pub.publish(Bool(data=True))
            self.get_logger().info(
                'Stabilization STARTED. EE locked at world xyz={}'.format(
                    np.round(self.desired_pos, 4))
            )
        elif not msg.data and self.enabled:
            self.enabled     = False
            self.desired_pos = None
            self.desired_rot = None
            self.disturbance_pub.publish(Bool(data=False))
            self.get_logger().info('Stabilization STOPPED.')

    # ── TF helper ────────────────────────────────────────────────────────────

    def _lookup_tf(self, parent, child):
        try:
            t   = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            pos = np.array([t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z])
            rot = Rotation.from_quat([t.transform.rotation.x,
                                      t.transform.rotation.y,
                                      t.transform.rotation.z,
                                      t.transform.rotation.w])
            return pos, rot
        except TransformException:
            return None, None

    # ── Control loop ─────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.enabled:
            return

        pos, rot = self._lookup_tf(self.world, self.ee)
        if pos is None:
            return

        # Pose error in world frame
        pos_error = self.desired_pos - pos
        rot_error = self.desired_rot * rot.inv()
        ang_error = rot_error.as_rotvec()

        # Proportional velocity command (world frame)
        v_lin_world = np.clip(self.kp_lin * pos_error, -self.max_lin, self.max_lin)
        v_ang_world = np.clip(self.kp_ang * ang_error, -self.max_ang, self.max_ang)

        # Rotate into link_base frame (required by MoveIt Servo config)
        _, R_base_world = self._lookup_tf(self.base, self.world)
        if R_base_world is None:
            return

        v_lin = R_base_world.apply(v_lin_world)
        v_ang = R_base_world.apply(v_ang_world)

        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base
        msg.twist.linear.x  = float(v_lin[0])
        msg.twist.linear.y  = float(v_lin[1])
        msg.twist.linear.z  = float(v_lin[2])
        msg.twist.angular.x = float(v_ang[0])
        msg.twist.angular.y = float(v_ang[1])
        msg.twist.angular.z = float(v_ang[2])
        self.twist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

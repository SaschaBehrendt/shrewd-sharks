#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""

import rclpy
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_msgs.msg import DepthStamped, Float64Stamped
from rclpy.node import Node


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        # Initialize setpoint and depth variables
        self.current_setpoint = 0.0
        self.current_depth = 0.0

        # PID control variables
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        # Buoyancy compensation (this must be tuned)
        self.buoyancy_offset = -0.1  # Adjust this to neutralize buoyancy

        # Publisher for thrust setpoint
        self.thrust_pub = self.create_publisher(ActuatorSetpoint,
                                                'thrust_setpoint',
                                                qos_profile=1)

        # Publishers for PID debugging
        self.depth_error_pub = self.create_publisher(Float64Stamped,
                                                     'depth_error',
                                                     qos_profile=1)
        self.p_gain_pub = self.create_publisher(Float64Stamped,
                                                'p_gain',
                                                qos_profile=1)
        self.i_gain_pub = self.create_publisher(Float64Stamped,
                                                'i_gain',
                                                qos_profile=1)
        self.d_gain_pub = self.create_publisher(Float64Stamped,
                                                'd_gain',
                                                qos_profile=1)

        # Subscriptions
        self.setpoint_sub = self.create_subscription(
            Float64Stamped,
            'depth_setpoint',
            self.on_setpoint,
            qos_profile=1,
        )
        self.depth_sub = self.create_subscription(
            DepthStamped,
            'depth',
            self.on_depth,
            qos_profile=1,
        )

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # Save the received setpoint for use in control calculations
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # Process new depth data
        current_depth = depth_msg.depth
        self.get_logger().info(
            f"Hi! I'm your controller running. I received a depth of {current_depth} m.",
            throttle_duration_sec=1,
        )

        # Compute control output (thrust)
        thrust = self.compute_control_output(current_depth)

        # Use the depth message timestamp for publishing
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        # Prepare and publish the thrust setpoint message
        msg = ActuatorSetpoint()
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False
        msg.z = thrust
        msg.header.stamp = timestamp.to_msg()
        self.thrust_pub.publish(msg)

    def compute_control_output(self, current_depth: float) -> float:
        # PID control calculations
        msg_e = Float64Stamped()
        msg_p = Float64Stamped()
        msg_i = Float64Stamped()
        msg_d = Float64Stamped()

        # Proportional term
        error_p = self.current_setpoint - current_depth
        msg_e.data = error_p
        self.depth_error_pub.publish(msg_e)

        # PID gains
        k_p = 2  # Proportional gain
        k_i = 0  # Integral gain
        k_d = 3  # Derivative gain

        # Time difference for integral and derivative terms
        now = self.get_clock().now()
        delta_t = (now.nanoseconds * 1e-9) - (self.last_time.nanoseconds * 1e-9)
        self.last_time = now

        # Integral term
        self.integral += error_p * delta_t
        msg_i.data = self.integral
        self.i_gain_pub.publish(msg_i)

        # Derivative term (avoid division by zero)
        if delta_t > 0:
            error_d = (error_p - self.last_error) / delta_t
        else:
            error_d = 0.0
        self.last_error = error_p
        msg_d.data = error_d
        self.d_gain_pub.publish(msg_d)

        # Thrust calculation (apply gains)
        if -0.8 <= self.current_setpoint <= -0.1:
            thrust_z = (k_p * error_p) + (k_i * self.integral) + (
                k_d * error_d) + self.buoyancy_offset
            msg_p.data = error_p
            self.p_gain_pub.publish(msg_p)
        else:
            # Reset integral if setpoint is out of bounds, set thrust to zero
            self.integral = 0.0
            thrust_z = 0.0

            # Optional: Limit thrust_z to actuator capabilities
        thrust_z = max(min(thrust_z, 1.0),
                       -1.0)  # Assuming thrust range is [-1, 1]

        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

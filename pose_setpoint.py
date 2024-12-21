#!/usr/bin/env python3
"""
This node computes a square-wave setpoint for the position controller, i.e.
the setpoint jumps between two different position values with a set duration.
You can change this code to try out other setpoint functions, e.g. a sin wave.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
import math

class PositionSetpointNode(Node):
    def __init__(self):
        super().__init__(node_name='position_setpoint_publisher')

        self.start_time = self.get_clock().now()

        # Setpoints für x, y, z
        self.setpoint_1 = {'x': -1.5, 'y': -3.5, 'z': -0.5}  # in m
        self.setpoint_2 = {'x': 0, 'y': -1, 'z': -0.6}  # in m
        self.duration = 120.0  # in seconds

        self.position_setpoint_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic='position_setpoint',
            qos_profile=1,
        )
        self.timer = self.create_timer(
            timer_period_sec=1 / 50,
            callback=self.on_timer,
        )

    def on_timer(self) -> None:
        now = self.get_clock().now()
        time = now - self.start_time
        i = time.nanoseconds * 1e-9 % (self.duration * 2)
        
        if i > self.duration:
            setpoint = self.setpoint_1
        else:
            setpoint = self.setpoint_2

        # Optional: Implementierung einer Kreisbewegung in der xy-Ebene
        # radius = 1.0  # Radius des Kreises
        # angular_velocity = 2 * math.pi / self.duration  # Winkelgeschwindigkeit
        # setpoint['x'] = radius * math.cos(angular_velocity * i)
        # setpoint['y'] = radius * math.sin(angular_velocity * i)

        self.publish_setpoint(setpoint=setpoint, now=now)

    def publish_setpoint(self, setpoint: dict, now: rclpy.time.Time) -> None:
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = setpoint['x']
        msg.pose.position.y = setpoint['y']
        msg.pose.position.z = setpoint['z']
        self.position_setpoint_pub.publish(msg)

def main():
    rclpy.init()
    node = PositionSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()


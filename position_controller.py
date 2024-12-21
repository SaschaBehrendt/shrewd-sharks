#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_msgs.msg import DepthStamped, Float64Stamped
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64

class PositionControlNode(Node):
    def __init__(self):
        super().__init__(node_name='position_controller')
        
        # Initialisierung der Setpoint- und Positions-Variablen
        self.current_setpoint = {'x': -1, 'y': -7, 'z': -0.5}
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # PID-Kontrollvariablen für jede Achse
        self.integral = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_error = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_time = self.get_clock().now()
        
        # Auftriebskompensation (muss angepasst werden)
        self.buoyancy_offset = 0
        
        # Publisher für Schub-Setpoints
        self.thrust_pub = self.create_publisher(ActuatorSetpoint, 'thrust_setpoint', qos_profile=1)
        
        # Publisher für PID-Debugging
        self.error_pubs = {
            'x': self.create_publisher(Float64Stamped, 'x_error', qos_profile=1),
            'y': self.create_publisher(Float64Stamped, 'y_error', qos_profile=1),
            'z': self.create_publisher(Float64Stamped, 'z_error', qos_profile=1)
        }
        
        # Subscriptions
        self.setpoint_sub = self.create_subscription(
            PoseStamped, 'position_setpoint', self.on_setpoint, qos_profile=1)
        self.position_sub = self.create_subscription(
            PoseStamped, 'position_estimate', self.on_position, qos_profile=1)
        self.depth_sub = self.create_subscription(
            DepthStamped, 'depth', self.on_depth, qos_profile=1)

    def on_setpoint(self, setpoint_msg: PoseStamped):
        # Aktualisiere die Setpoints für alle Achsen
        self.current_setpoint['x'] = setpoint_msg.pose.position.x
        self.current_setpoint['y'] = setpoint_msg.pose.position.y
        self.current_setpoint['z'] = setpoint_msg.pose.position.z

    def on_position(self, position_msg: PoseStamped):
        # Aktualisiere die aktuellen x- und y-Positionen aus dem Kalman-Filter
        self.current_position['x'] = position_msg.pose.position.x
        self.current_position['y'] = position_msg.pose.position.y
        
        # Berechne den Schub für x und y
        thrust_x = self.compute_control_output('x')
        thrust_y = self.compute_control_output('y')
        
        # Veröffentliche den horizontalen Schub
        self.publish_horizontal_thrust(thrust_x, thrust_y, position_msg.header.stamp)

    def on_depth(self, depth_msg: DepthStamped):
        # Aktualisiere die aktuelle z-Position (Tiefe)
        self.current_position['z'] = depth_msg.depth
        
        # Berechne den Schub für z
        thrust_z = self.compute_control_output('z')
        
        # Veröffentliche den vertikalen Schub
        self.publish_vertical_thrust(thrust_z, depth_msg.header.stamp)

    def publish_horizontal_thrust(self, thrust_x: float, thrust_y: float, timestamp: rclpy.time.Time) -> None:
        # Bereite die Schub-Setpoint-Nachricht für horizontale Bewegung vor und veröffentliche sie
        msg = ActuatorSetpoint()
        msg.ignore_x = False
        msg.ignore_y = False
        msg.ignore_z = True
        msg.x = thrust_x
        msg.y = thrust_y
        msg.header.stamp = timestamp
        self.thrust_pub.publish(msg)

    def publish_vertical_thrust(self, thrust: float, timestamp: rclpy.time.Time) -> None:
        # Bereite die Schub-Setpoint-Nachricht für vertikale Bewegung vor und veröffentliche sie
        msg = ActuatorSetpoint()
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False
        msg.z = thrust
        msg.header.stamp = timestamp
        self.thrust_pub.publish(msg)

    def compute_control_output(self, axis: str) -> float:
        # PID-Kontrollberechnungen für die gegebene Achse
        error = self.current_setpoint[axis] - self.current_position[axis]
        
        # Veröffentliche den Fehler für Debugging
        msg_e = Float64Stamped()
        msg_e.data = error
        self.error_pubs[axis].publish(msg_e)

        # PID-Verstärkungen (müssen für jede Achse angepasst werden)
        k_p = 0.5  # Proportionalverstärkung
        k_i = 0  # Integralverstärkung
        k_d = 0  # Differentialverstärkung

        # Zeitdifferenz für Integral- und Differentialterme
        now = self.get_clock().now()
        delta_t = (now.nanoseconds - self.last_time.nanoseconds) * 1e-9
        self.last_time = now

        # Integralterm
        self.integral[axis] += error * delta_t

        # Differentialterm
        if delta_t > 0:
            error_d = (error - self.last_error[axis]) / delta_t
        else:
            error_d = 0.0
        self.last_error[axis] = error

        # Schubberechnung (Anwendung der Verstärkungen)
        thrust = (k_p * error) + (k_i * self.integral[axis]) + (k_d * error_d)

        # Füge Auftriebskompensation für z-Achse hinzu
        if axis == 'z':
            thrust += self.buoyancy_offset

        # Begrenze den Schub auf den Bereich [-1, 1]
        thrust = max(min(thrust, 1.0), -1.0)

        return thrust

def main():
    rclpy.init()
    node = PositionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

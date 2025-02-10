#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleStatus, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import TimesyncStatus
from rclpy.clock import Clock

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')
        
        # Configure QoS for PX4 compatibility
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # PX4 Status Subscriber
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile
        )
        
        # Setpoint Publisher
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        # Vehicle Command Publisher (for arming/mode changes)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )
        
        # Control timer (20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # State variables
        self.current_status = VehicleStatus()
        self.last_request = self.get_clock().now()
        self.armed = False
        self.offboard_mode = False

    def status_callback(self, msg):
        self.current_status = msg
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def publish_trajectory_setpoint(self, vx=0.0, vy=0.0, vz=0.0):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.yaw = 0.0  # Radians
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def control_loop(self):
        # Send setpoint even when not armed to maintain offboard mode
        self.publish_trajectory_setpoint(vx=0.5, vy=0.0, vz=-1.0)  # NED frame (negative Z = up)

        now = self.get_clock().now()
        
        # Handle mode transitions
        if not self.offboard_mode:
            if (now - self.last_request).nanoseconds > 5e9:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,  # Base mode
                    6.0   # Custom mode: OFFBOARD
                )
                self.last_request = now
                self.get_logger().info("Requesting OFFBOARD mode...")
        
        elif not self.armed:
            if (now - self.last_request).nanoseconds > 5e9:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0  # ARM
                )
                self.last_request = now
                self.get_logger().info("Requesting arming...")

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

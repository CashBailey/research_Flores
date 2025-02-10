#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from px4_msgs.msg import VehicleStatus, TrajectorySetpoint, VehicleCommand
from rclpy.callback_groups import ReentrantCallbackGroup

class OffboardSquareTrajectory(Node):
    def __init__(self):
        super().__init__('offb_square_trajectory')
        
        # Configure QoS for PX4 compatibility
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Current state and trajectory parameters
        self.altura_fija = -1.0  # NED frame (negative Z = up)
        self.velocidad_base = 0.5  # Base linear velocity (m/s)
        self.duracion = 15.0  # Duration per movement (seconds)
        self.movimientos = [
            (self.velocidad_base, 0),    # Forward X
            (0, self.velocidad_base),    # Right Y
            (-self.velocidad_base, 0),   # Backward X
            (0, -self.velocidad_base)    # Left Y
        ]
        self.current_movement = 0
        self.step_counter = 0
        self.last_req = self.get_clock().now()

        # Create callback group for parallel execution
        self.cb_group = ReentrantCallbackGroup()

        # Vehicle status subscriber
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile,
            callback_group=self.cb_group
        )

        # Setpoint publisher
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        # Vehicle command publisher
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # Control timer (20Hz)
        self.timer = self.create_timer(
            0.05,  # 20Hz
            self.control_loop,
            callback_group=self.cb_group
        )

        # State variables
        self.current_status = VehicleStatus()
        self.armed = False
        self.offboard_mode = False

    def status_callback(self, msg):
        self.current_status = msg
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def publish_velocity(self, vx, vy):
        """Publish velocity command with fixed altitude"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = [float(vx), float(vy), 0.0]  # Maintain horizontal velocity
        msg.position = [float('nan'), float('nan'), self.altura_fija]
        msg.yaw = 0.0  # Radians (0 = North)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle commands"""
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
        """Main control loop with state machine"""
        now = self.get_clock().now()
        
        # Maintain continuous setpoint publishing
        self.publish_velocity(0.0, 0.0)  # Default to zero velocity
        
        # Handle mode transitions
        if not self.offboard_mode:
            if (now - self.last_req) > Duration(seconds=5.0):
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,  # Base mode
                    6.0   # Custom mode: OFFBOARD
                )
                self.last_req = now
                self.get_logger().info("Requesting OFFBOARD mode...")
                return
        
        # Handle arming
        if not self.armed:
            if (now - self.last_req) > Duration(seconds=5.0):
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0  # ARM
                )
                self.last_req = now
                self.get_logger().info("Requesting arming...")
                return

        # Execute trajectory if armed and in offboard mode
        if self.armed and self.offboard_mode:
            vx, vy = self.movimientos[self.current_movement]
            
            # Calculate velocity reduction factor
            factor = 1 - (self.step_counter / (self.duracion * 20))
            vx_actual = vx * factor
            vy_actual = vy * factor
            
            self.get_logger().info(
                f"Movement {self.current_movement+1}/4 | "
                f"vx: {vx_actual:.2f}, vy: {vy_actual:.2f} | "
                f"Factor: {factor:.2f}",
                throttle_duration_sec=1.0
            )
            
            # Publish velocity command
            self.publish_velocity(vx_actual, vy_actual)
            
            # Update step counter
            self.step_counter += 1
            
            # Check movement completion
            if self.step_counter >= (self.duracion * 20):
                self.step_counter = 0
                self.current_movement = (self.current_movement + 1) % 4

def main(args=None):
    rclpy.init(args=args)
    
    node = OffboardSquareTrajectory()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

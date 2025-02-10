#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleStatus, TrajectorySetpoint  # Use px4_msgs for ROS2
from px4_msgs.srv import CommandBool, VehicleCommand  # Use px4_msgs for ROS2
from rclpy.clock import Clock
from std_msgs.msg import Float64MultiArray  # Import for Uv and Uw subscriptions


class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.current_status = VehicleStatus()  # Use px4_msgs VehicleStatus
        self.state_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.state_cb, qos) # Use px4_msgs VehicleStatus and correct topic

        self.Uv = [0.0, 0.0, 0.0]  # Initialize Uv as list
        self.Uv_sub = self.create_subscription(
            Float64MultiArray, '/Uv', self.uv_cb, qos) # Subscribe to /Uv topic

        self.Uw = [0.0, 0.0, 0.0]  # Initialize Uw as list
        self.Uw_sub = self.create_subscription(
            Float64MultiArray, '/Uw', self.uw_cb, qos) # Subscribe to /Uw topic


        self.trajectory_pub = self.create_publisher( # Use px4_msgs TrajectorySetpoint and correct topic
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        self.arming_client = self.create_client(CommandBool, '/fmu/cmd/arming') # Use px4_msgs CommandBool and correct topic
        self.set_mode_client = self.create_client(VehicleCommand, '/fmu/cmd/vehicle_command') # Use px4_msgs VehicleCommand and correct topic

        self.timer = self.create_timer(0.04, self.main_loop)  # 25Hz (40ms period), corrected timer frequency
        self.last_req = self.get_clock().now()
        self.start_time = self.get_clock().now() # Initialize start_time
        self.params_set = False # Initialize params_set

    def state_cb(self, msg):
        self.current_status = msg

    def uv_cb(self, msg): # Callback for Uv subscription
        if len(msg.data) == 3:
            self.Uv = msg.data

    def uw_cb(self, msg): # Callback for Uw subscription
        if len(msg.data) == 3:
            self.Uw = msg.data


    def publish_trajectory_setpoint(self, x=float('nan'), y=float('nan'), z=float('nan'), yaw=float('nan'), vx=float('nan'), vy=float('nan'), vz=float('nan'), roll_rate=float('nan'), pitch_rate=float('nan'), yaw_rate=float('nan')):
        """Publish command for trajectory setpoint"""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # in us
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [roll_rate, pitch_rate, yaw_rate] # angular velocity is mapped to acceleration in px4_msgs/TrajectorySetpoint
        self.trajectory_pub.publish(msg)


    def main_loop(self):
        if self.current_status.nav_state == VehicleStatus.NAVIGATION_STATE_UNINITIALIZED: # Use VehicleStatus enum
            return

        if self.current_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and (self.get_clock().now() - self.last_req).nanoseconds > 5e9: # Use VehicleStatus enum
            self.set_mode_offboard()
            self.last_req = self.get_clock().now()

        elif not self.current_status.arming_state.armed and (self.get_clock().now() - self.last_req).nanoseconds > 5e9:
            self.arm_vehicle()
            self.last_req = self.get_clock().now()

        else:
            elapsed_time = self.get_clock().now() - self.start_time
            if elapsed_time.seconds() <= 15.0:
                # Initialization phase: small vertical climb
                self.publish_trajectory_setpoint(z=-1.0, yaw=0.0) # z=-1.0 for NED frame (up)
            else:
                # Convert control inputs from ENU to NED frame
                velocity_body = [
                    -self.Uv[0],  # Forward (NED X)
                    -self.Uv[2],  # Right (NED Y)
                    -self.Uv[1]   # Down (NED Z)
                ]

                angular_velocity = [
                    -self.Uw[0],  # Roll (NED X)
                    -self.Uw[2],  # Pitch (NED Y)
                    -self.Uw[1]   # Yaw (NED Z)
                ]
                self.publish_trajectory_setpoint(
                    vx=velocity_body[0], vy=velocity_body[1], vz=velocity_body[2],
                    roll_rate=angular_velocity[0], pitch_rate=angular_velocity[1], yaw_rate=angular_velocity[2]
                )


    def set_mode_offboard(self):
        """Request to set OFFBOARD mode"""
        req = VehicleCommand.Request() # Use VehicleCommand for set_mode
        req.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        req.param1 = 1.0 # Set flag to indicate base_mode
        req.param2 = 6.0 # OFFBOARD mode is 6 in PX4
        self.set_mode_client.call_async(req)
        self.get_logger().info("Attempting to set OFFBOARD mode")


    def arm_vehicle(self):
        """Request to arm the vehicle"""
        req = CommandBool.Request() # Use CommandBool for arming
        req.value = True
        self.arming_client.call_async(req)
        self.get_logger().info("Attempting to arm vehicle")


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

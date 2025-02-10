
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include <lib/matrix/matrix/math.hpp>

using namespace std::chrono_literals;

// Define data structure to share data between components
struct Context {
    px4_msgs::msg::VehicleStatus current_status;
    Eigen::Vector3d Uv;
    Eigen::Vector3d Uw;
    rclcpp::Time last_request;
    rclcpp::Time start_time;
    bool params_set;
};

class OffboardNode : public rclcpp::Node {
public:
    OffboardNode() : Node("offb_node") {
        // Initialize subscribers
        vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
                context_.current_status = *msg;
            });
            
        Uv_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Uv", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() == 3) {
                    context_.Uv << msg->data[0], msg->data[1], msg->data[2];
                }
            });

        Uw_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/Uw", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() == 3) {
                    context_.Uw << msg->data[0], msg->data[1], msg->data[2];
                }
            });

        // Initialize publishers with appropriate QoS
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        
        trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos_profile);
            
        vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos_profile);

        // Initialize timers
        timer_ = create_wall_timer(40ms, std::bind(&OffboardNode::control_loop, this));

        // Initialize context
        context_.last_request = now();
        context_.start_time = now();
        context_.params_set = false;
        context_.Uv.setZero();
        context_.Uw.setZero();
    }

private:
    void control_loop() {
        // Wait for system to be ready
        if (context_.current_status.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_UNINITIALIZED) {
            return;
        }

        // Handle mode setting and arming
        handle_offboard_arming();

        // Execute control logic
        execute_control();
    }

    void handle_offboard_arming() {
        auto now = this->now();
        
        // Set Offboard mode
        if (context_.current_status.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
            (now - context_.last_request) > 5s) 
        {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 
                                   1.0f, 6.0f);  // 6 = OFFBOARD mode
            context_.last_request = now;
        }
        // Arm vehicle
        else if (!context_.current_status.arming_state.armed &&
                (now - context_.last_request) > 5s) 
        {
            publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                                   1.0f);
            context_.last_request = now;
        }
    }

    void execute_control() {
        auto elapsed_time = now() - context_.start_time;
        auto setpoint = px4_msgs::msg::TrajectorySetpoint();

        if (elapsed_time.seconds() <= 15.0) {
            // Initialization phase: small vertical climb
            setpoint.position = {NAN, NAN, -1.0f};  // NED frame (negative Z = up)
            setpoint.yaw = 0.0f;  // Radians
        } else {
            // Convert control inputs from ENU to NED frame
            matrix::Vector3d velocity_body(
                -context_.Uv(0),  // Forward (NED X)
                -context_.Uv(2),  // Right (NED Y)
                -context_.Uv(1)   // Down (NED Z)
            );
            
            matrix::Vector3d angular_velocity(
                -context_.Uw(0),  // Roll (NED X)
                -context_.Uw(2),  // Pitch (NED Y)
                -context_.Uw(1)   // Yaw (NED Z)
            );

            setpoint.velocity = {velocity_body(0), velocity_body(1), velocity_body(2)};
            setpoint.acceleration = {angular_velocity(0), angular_velocity(1), angular_velocity(2)};
        }

        trajectory_pub_->publish(setpoint);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.command = command;
        msg.param1 = param1;
        msg.param2 = param2;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_pub_->publish(msg);
    }

    // Member variables
    Context context_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Uv_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Uw_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
}

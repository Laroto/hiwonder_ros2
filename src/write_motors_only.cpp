#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sbs_controller.hpp"

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        // Declare parameters
        this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 9600);
        this->declare_parameter<int>("motor_read_rate", 4);

        // Get parameters
        std::string device = this->get_parameter("device").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        int motor_read_rate = this->get_parameter("motor_read_rate").as_int();
        int delay_ms = static_cast<int>(1000.0 / motor_read_rate);

        // Create controller object
        sbs_controller_ = std::make_unique<SBSController>(device, baud_rate);

        // Subscriber to receive motor commands
        joint_states_sub_ = this->create_subscription<actuator_msgs::msg::Actuators>(
            "/actuators", 10,
            std::bind(&MotorController::jointStatesCallback, this, std::placeholders::_1));

        disable_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/motors/disable",
            std::bind(&MotorController::disableMotorsCallback, this, std::placeholders::_1, std::placeholders::_2));
        enable_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/motors/enable",
            std::bind(&MotorController::enableMotorsCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Timer to alternate read/write
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(delay_ms /2),
            std::bind(&MotorController::timedWrite, this));

        RCLCPP_INFO(this->get_logger(), "Motor Controller Node started");

        // Sleep for 2 seconds to wait for messages to arrive
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

private:
    void timedWrite() {
        if (!motors_enabled_) {
            return;
        }

        if (this->latest_joint_state_) {
            sendMotorCommands(std::make_shared<actuator_msgs::msg::Actuators>(this->latest_joint_state_.value()));
        } else {
            RCLCPP_WARN(this->get_logger(), "No message received yet on /actuators");
        }
    }

    void jointStatesCallback(const actuator_msgs::msg::Actuators::SharedPtr msg) {
        this->latest_joint_state_ = *msg; // Store the latest message
    }

    void disableMotorsCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        motors_enabled_ = false;
        try {
            sbs_controller_->cmdMultiServoUnload(servo_ids_);
            response->success = true;
            response->message = "Motors disabled.";
            RCLCPP_WARN(this->get_logger(), "Motors disabled; actuator writes are paused.");
        } catch (const std::exception & exc) {
            response->success = false;
            response->message = std::string("Failed to disable motors: ") + exc.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    void enableMotorsCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        motors_enabled_ = true;
        if (this->latest_joint_state_) {
            sendMotorCommands(std::make_shared<actuator_msgs::msg::Actuators>(this->latest_joint_state_.value()));
        }

        response->success = true;
        response->message = "Motors enabled.";
        RCLCPP_INFO(this->get_logger(), "Motors enabled; actuator writes are active.");
    }

    void sendMotorCommands(const actuator_msgs::msg::Actuators::SharedPtr latest_joint_state) {

        if (latest_joint_state->position.size() != servo_ids_.size()) {
            RCLCPP_ERROR(this->get_logger(), "Received joint state size does not match servo ids size");
            return;
        }

        std::vector<uint16_t> positions;
        for (size_t i = 0; i < latest_joint_state->position.size(); ++i) {
            double degrees = latest_joint_state->position[i] * (180.0 / M_PI);
            double scaled_position = ((degrees + 120.0) / 240.0) * 1000.0;
            positions.push_back(static_cast<uint16_t>(scaled_position));
        }

        try {
            sbs_controller_->cmdServoMove(servo_ids_, positions, 1);
        } catch (const std::exception & exc) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send motor commands: %s", exc.what());
        }
    }

    std::vector<unsigned char> servo_ids_ = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};

    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr joint_states_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<SBSController> sbs_controller_;

    std::optional<actuator_msgs::msg::Actuators> latest_joint_state_;
    bool motors_enabled_ = true;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

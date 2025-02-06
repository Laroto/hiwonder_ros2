#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

        // Publisher for motor state readings
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_read", 1);

        // Subscriber to receive motor commands
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MotorController::jointStatesCallback, this, std::placeholders::_1));

        // Timer to alternate read/write
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(delay_ms),
            std::bind(&MotorController::alternatedReadWrite, this));

        RCLCPP_INFO(this->get_logger(), "Motor Controller Node started");

        // Sleep for 2 seconds to wait for messages to arrive
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

private:
    void alternatedReadWrite() {
        if (this->comm_state == CommState::READ) {
            // this->publishJointStates();
            this->comm_state = CommState::WRITE;
        } else if (this->comm_state == CommState::WRITE) {
            if (this->latest_joint_state_) {
                sendMotorCommands(std::make_shared<sensor_msgs::msg::JointState>(this->latest_joint_state_.value()));
            } else {
                RCLCPP_WARN(this->get_logger(), "No message received yet on /joint_states");
            }
            this->comm_state = CommState::READ;
        }
    }

    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->latest_joint_state_ = *msg; // Store the latest message
    }

    void publishJointStates() {
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = this->now();

        std::vector<std::string> joint_names;
        for (int i = 0; i < servo_ids_.size(); i++) {
            joint_names.push_back("motor_" + std::to_string(servo_ids_[i]));
        }
        joint_state_msg.name = joint_names;

        auto positions = sbs_controller_->cmdMultiServoPosRead(servo_ids_);
        if (!positions) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read motor positions");
            return;
        }

        joint_state_msg.position = std::vector<double>(positions->begin(), positions->end());
        joint_states_pub_->publish(joint_state_msg);
    }

    void sendMotorCommands(const sensor_msgs::msg::JointState::SharedPtr latest_joint_state) {

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

        sbs_controller_->cmdServoMove(servo_ids_, positions, 1);
    }

    std::vector<unsigned char> servo_ids_ = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<SBSController> sbs_controller_;

    std::optional<sensor_msgs::msg::JointState> latest_joint_state_;

    enum class CommState { READ, WRITE };
    CommState comm_state = CommState::READ;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

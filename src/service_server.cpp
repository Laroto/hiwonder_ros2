#include "rclcpp/rclcpp.hpp"
#include "hiwonder_ros2/srv/get_motor_positions.hpp"
#include "sbs_controller.hpp"
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <future>

class MotorServiceServer : public rclcpp::Node {
public:
    MotorServiceServer() : Node("motor_service_server"), running_(true) {
        this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 9600);
        
        std::string device = this->get_parameter("device").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();

        sbs_controller_ = std::make_unique<SBSController>(device, baud_rate);

        read_service_ = this->create_service<hiwonder_ros2::srv::GetMotorPositions>(
            "read_motor_positions",
            [this](const std::shared_ptr<hiwonder_ros2::srv::GetMotorPositions::Request> request,
                   std::shared_ptr<hiwonder_ros2::srv::GetMotorPositions::Response> response) {
                std::promise<void> promise;
                std::future<void> future = promise.get_future();

                // Directly submit the request to be processed
                this->submitRequest([this, response, &promise]() {
                    this->readMotorPositions(response, promise);
                });

                // Wait until the request is processed
                future.wait();
            }
        );
        
        worker_thread_ = std::thread(&MotorServiceServer::processMotorRequests, this);
        RCLCPP_INFO(this->get_logger(), "Motor Service Server started");
    }

    ~MotorServiceServer() {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            running_ = false;
        }
        cv_.notify_one();
        worker_thread_.join();
    }

private:
    void submitRequest(std::function<void()> request) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            request_queue_.push(request);
        }
        cv_.notify_one();
    }

    void processMotorRequests() {
        while (running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_.wait(lock, [this] { return !request_queue_.empty() || !running_; });
            
            if (!running_) break;
            
            auto task = request_queue_.front();
            request_queue_.pop();
            lock.unlock();
            
            task(); // Execute motor request
        }
    }

    void readMotorPositions(std::shared_ptr<hiwonder_ros2::srv::GetMotorPositions::Response> response, 
                            std::promise<void>& promise) {
        auto positions = sbs_controller_->cmdMultiServoPosRead(servo_ids_);
        if (!positions) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read motor positions");
            response->positions.clear();
        } else {
            auto pos_values = positions.value();
            std::vector<float> position_radians;
            position_radians.reserve(pos_values.size());

            for (size_t i = 0; i < pos_values.size(); ++i) {
                uint16_t pos = pos_values[i];
                float degrees = ((static_cast<float>(pos) - 500.0f) / 500.0f) * 120.0f;
                float radians = degrees * (M_PI / 180.0f);
                position_radians.push_back(radians);

                // RCLCPP_INFO(this->get_logger(), "Servo ID %d Position: %.4f rad", 
                //             static_cast<int>(servo_ids_[i]), radians);
            }

            response->positions = position_radians;
        }

        // Notify completion
        promise.set_value();
    }

    // motor controller
    std::unique_ptr<SBSController> sbs_controller_;

    // servos
    std::vector<unsigned char> servo_ids_ = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
    std::vector<std::string> joint_names_ = {
        "leg_0_base__leg_0a", "leg_0a__leg_0b", "leg_0b__leg_0c", 
        "leg_1_base__leg_1a", "leg_1a__leg_1b", "leg_1b__leg_1c", 
        "leg_2_base__leg_2a", "leg_2a__leg_2b", "leg_2b__leg_2c", 
        "leg_3_base__leg_3a", "leg_3a__leg_3b", "leg_3b__leg_3c", 
        "leg_4_base__leg_4a", "leg_4a__leg_4b", "leg_4b__leg_4c", 
        "leg_5_base__leg_5a", "leg_5a__leg_5b", "leg_5b__leg_5c", 
    };

    // ros2 services
    rclcpp::Service<hiwonder_ros2::srv::GetMotorPositions>::SharedPtr read_service_;

    // safety for multithreading
    std::queue<std::function<void()>> request_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    bool running_;
    std::thread worker_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorServiceServer>());
    rclcpp::shutdown();
    return 0;
}

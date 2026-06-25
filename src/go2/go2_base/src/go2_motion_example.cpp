#include "rclcpp/rclcpp.hpp"
#include "go2_msgs/srv/motion_cmd.hpp"
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class Go2MotionExample : public rclcpp::Node
{
public:
    Go2MotionExample()
    : Node("go2_motion_example")
    {
        // go2_driver의 ROS 서비스 클라이언트 생성
        client_ = this->create_client<go2_msgs::srv::MotionCmd>("go2_motion_cmd");
        
        // 서비스 준비 대기
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "go2_motion_cmd service not available");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for go2_motion_cmd service...");
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to go2_motion_cmd service!");
        
        // 상태머신 타이머 시작
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&Go2MotionExample::stateMachineCallback, this)
        );
    }

private:
    rclcpp::Client<go2_msgs::srv::MotionCmd>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    enum State {
        STAND_UP_STATE,      // 일어서기
        STAND_UP_WAIT,       // 일어서기 대기
        STAND_DOWN_STATE,    // 앉기
        STAND_DOWN_WAIT,     // 앉기 대기
        DONE
    };
    
    State current_state_ = STAND_UP_STATE;
    int wait_count_ = 0;
    const int WAIT_DURATION = 30;  // 100ms * 30 = 3초

    void stateMachineCallback()
    {
        switch (current_state_) {
            case STAND_UP_STATE:
                RCLCPP_INFO(this->get_logger(), "[State] STAND_UP_STATE - Standing up...");
                sendMotionCommand(go2_msgs::srv::MotionCmd::Request::STAND_UP);
                current_state_ = STAND_UP_WAIT;
                wait_count_ = 0;
                break;

            case STAND_UP_WAIT:
                wait_count_++;
                if (wait_count_ >= WAIT_DURATION) {
                    RCLCPP_INFO(this->get_logger(), "[State] STAND_UP_WAIT - Done, moving to STAND_DOWN");
                    current_state_ = STAND_DOWN_STATE;
                }
                break;

            case STAND_DOWN_STATE:
                RCLCPP_INFO(this->get_logger(), "[State] STAND_DOWN_STATE - Sitting down...");
                sendMotionCommand(go2_msgs::srv::MotionCmd::Request::STAND_DOWN);
                current_state_ = STAND_DOWN_WAIT;
                wait_count_ = 0;
                break;

            case STAND_DOWN_WAIT:
                wait_count_++;
                if (wait_count_ >= WAIT_DURATION) {
                    RCLCPP_INFO(this->get_logger(), "[State] STAND_DOWN_WAIT - Done!");
                    current_state_ = DONE;
                }
                break;

            case DONE:
                RCLCPP_INFO(this->get_logger(), "[State] DONE - Motion example completed!");
                rclcpp::shutdown();
                break;
        }
    }

    void sendMotionCommand(int32_t task)
    {
        auto request = std::make_shared<go2_msgs::srv::MotionCmd::Request>();
        request->task = task;

        // 비동기 서비스 호출
        auto future = client_->async_send_request(request,
            [this, task](rclcpp::Client<go2_msgs::srv::MotionCmd>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->respond == 0) {
                        RCLCPP_INFO(this->get_logger(), 
                            "Motion command (task=%d) executed successfully", task);
                    } else {
                        RCLCPP_WARN(this->get_logger(), 
                            "Motion command (task=%d) failed with code: %d", task, response->respond);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), 
                        "Service call failed: %s", e.what());
                }
            });
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2MotionExample>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

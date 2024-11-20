#ifndef ACKERMANN_MANAGER_HPP
#define ACKERMANN_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

namespace vesc_ackermann {

class AckermannManager : public rclcpp::Node
{
public:
    // 기본 생성자
    AckermannManager();

    // NodeOptions 인자를 받는 생성자 (컴포넌트로 사용하기 위해 필요)
    explicit AckermannManager(const rclcpp::NodeOptions & options);

    // 모드 enum 정의
    enum class Mode
    {
        MAIN,
        A,
        B,
        X,
        Y,
        JOY
    };

    // 모드 설정 및 반환 함수
    void set_mode(Mode mode);
    Mode get_mode() const;

private:
    // 초기화 함수들
    void initialize_parameters();
    void setup_subscriptions();
    void setup_publisher();

    // 콜백 함수들
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ackermann_main_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void ackermann_a_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void ackermann_b_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void ackermann_x_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
    void ackermann_y_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    // ROS 2 Subscription 및 Publisher
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_main_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_a_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_b_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_x_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_y_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;

    // 파라미터
    double max_speed_;
    double max_steering_angle_;

    // 현재 모드 및 초기화 플래그
    Mode current_mode_;
    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;
    bool joy_received_ = false;
    bool init_ackermann_main_ = false;
    bool init_ackermann_a_ = false;
    bool init_ackermann_b_ = false;
    bool init_ackermann_x_ = false;
    bool init_ackermann_y_ = false;
};

}  // namespace vesc_ackermann

#endif  // ACKERMANN_MANAGER_HPP

#include "vesc_ackermann/ackermann_manager.hpp"

namespace vesc_ackermann {

AckermannManager::AckermannManager()
    : Node("ackermann_manager"), current_mode_(Mode::JOY),
      init_ackermann_a_(false), init_ackermann_b_(false),
      init_ackermann_x_(false), init_ackermann_y_(false)
{
    initialize_parameters();
    setup_subscriptions();
    setup_publisher();
}

AckermannManager::AckermannManager(const rclcpp::NodeOptions & options)
    : Node("ackermann_manager", options), current_mode_(Mode::JOY),
      init_ackermann_a_(false), init_ackermann_b_(false),
      init_ackermann_x_(false), init_ackermann_y_(false)
{
    initialize_parameters();
    setup_subscriptions();
    setup_publisher();
}

void AckermannManager::initialize_parameters()
{
    // Declare parameters with default values
    declare_parameter<double>("max_speed", 1.0);
    declare_parameter<double>("max_steering_angle", 0.5);

    // Retrieve parameter values
    this->get_parameter("max_speed", max_speed_);
    this->get_parameter("max_steering_angle", max_steering_angle_);
}

void AckermannManager::setup_subscriptions()
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&AckermannManager::joy_callback, this, std::placeholders::_1));

    ackermann_main_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "ackermann_cmd_main", 10, std::bind(&AckermannManager::ackermann_main_callback, this, std::placeholders::_1));

    ackermann_a_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "ackermann_cmd_a", 10, std::bind(&AckermannManager::ackermann_a_callback, this, std::placeholders::_1));

    ackermann_b_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "ackermann_cmd_b", 10, std::bind(&AckermannManager::ackermann_b_callback, this, std::placeholders::_1));

    ackermann_x_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "ackermann_cmd_x", 10, std::bind(&AckermannManager::ackermann_x_callback, this, std::placeholders::_1));

    ackermann_y_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "ackermann_cmd_y", 10, std::bind(&AckermannManager::ackermann_y_callback, this, std::placeholders::_1));
}

void AckermannManager::setup_publisher()
{
    ackermann_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 10);
}

void AckermannManager::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    last_joy_msg_ = msg;
    joy_received_ = true;

    if (msg->buttons[4]) current_mode_ = Mode::JOY;
    else if (msg->buttons[5]) current_mode_ = Mode::MAIN;
    else if (msg->buttons[0] && init_ackermann_a_) current_mode_ = Mode::A;
    else if (msg->buttons[1] && init_ackermann_b_) current_mode_ = Mode::B;
    else if (msg->buttons[2] && init_ackermann_x_) current_mode_ = Mode::X;
    else if (msg->buttons[3] && init_ackermann_y_) current_mode_ = Mode::Y;

    if (current_mode_ == Mode::JOY && msg->buttons[4]) {
        auto ackermann_msg = ackermann_msgs::msg::AckermannDriveStamped();
        ackermann_msg.header.stamp = this->now();

        double speed = max_speed_ * msg->axes[1];
        double steering_angle = max_steering_angle_ * msg->axes[3];

        ackermann_msg.drive.speed = speed;
        ackermann_msg.drive.steering_angle = steering_angle;
        ackermann_pub_->publish(ackermann_msg);
    }
}

void AckermannManager::ackermann_main_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (current_mode_ != Mode::MAIN) return;
    ackermann_pub_->publish(*msg);
}

void AckermannManager::ackermann_a_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (!init_ackermann_a_) init_ackermann_a_ = true;
    if (current_mode_ != Mode::A) return;
    ackermann_pub_->publish(*msg);
}

void AckermannManager::ackermann_b_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (!init_ackermann_b_) init_ackermann_b_ = true;
    if (current_mode_ != Mode::B) return;
    ackermann_pub_->publish(*msg);
}

void AckermannManager::ackermann_x_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (!init_ackermann_x_) init_ackermann_x_ = true;
    if (current_mode_ != Mode::X) return;
    ackermann_pub_->publish(*msg);
}

void AckermannManager::ackermann_y_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
{
    if (!init_ackermann_y_) init_ackermann_y_ = true;
    if (current_mode_ != Mode::Y) return;
    ackermann_pub_->publish(*msg);
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannManager)

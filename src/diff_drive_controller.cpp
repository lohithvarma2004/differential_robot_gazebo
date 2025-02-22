#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <algorithm>

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController() : Node("diff_drive_controller") {
        // Declare parameters
        this->declare_parameter("wheelbase", 0.5);
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("max_rpm", 100.0);

        // Get parameters
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // Publishers
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        // Subscriber to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        double v_left = linear_x - (angular_z * wheelbase_ / 2.0);
        double v_right = linear_x + (angular_z * wheelbase_ / 2.0);

        double rpm_left = (v_left / (2 * M_PI * wheel_radius_)) * 60.0;
        double rpm_right = (v_right / (2 * M_PI * wheel_radius_)) * 60.0;

        // Limit RPM to max allowed
        rpm_left = std::clamp(rpm_left, -max_rpm_, max_rpm_);
        rpm_right = std::clamp(rpm_right, -max_rpm_, max_rpm_);

        // Publish RPM values
        std_msgs::msg::Float64 left_msg, right_msg;
        left_msg.data = rpm_left;
        right_msg.data = rpm_right;

        left_wheel_pub_->publish(left_msg);
        right_wheel_pub_->publish(right_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;

    double wheelbase_, wheel_radius_, max_rpm_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveController>());
    rclcpp::shutdown();
    return 0;
}

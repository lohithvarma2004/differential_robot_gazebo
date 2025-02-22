#include <thread>
#include <mutex>
#include <cmath>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class RPMDiffDrivePlugin : public ModelPlugin {
public:
  RPMDiffDrivePlugin() : ModelPlugin(), x_(0.0), y_(0.0), theta_(0.0) {}

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
    // Store the pointer to the model.
    this->model = _model;

    // Retrieve joint names from SDF parameters (or use defaults).
    if (_sdf->HasElement("left_joint"))
      left_joint_name = _sdf->Get<std::string>("left_joint");
    else
      left_joint_name = "wheel2_joint";

    if (_sdf->HasElement("right_joint"))
      right_joint_name = _sdf->Get<std::string>("right_joint");
    else
      right_joint_name = "wheel1_joint";

    left_joint = model->GetJoint(left_joint_name);
    right_joint = model->GetJoint(right_joint_name);
    if (!left_joint || !right_joint) {
      gzerr << "RPMDiffDrivePlugin: One or both wheel joints not found: "
            << left_joint_name << ", " << right_joint_name << std::endl;
      return;
    }

    // Retrieve wheel parameters (with defaults if not provided)
    if (_sdf->HasElement("wheel_radius"))
      wheel_radius_ = _sdf->Get<double>("wheel_radius");
    else
      wheel_radius_ = 0.1;
    if (_sdf->HasElement("wheelbase"))
      wheelbase_ = _sdf->Get<double>("wheelbase");
    else
      wheelbase_ = 0.5;

    // Initialize ROS 2 node (if not already initialized).
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    ros_node = rclcpp::Node::make_shared("rpm_diff_drive_plugin_node");

    // Create subscribers for RPM values.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    left_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
      "/left_wheel_rpm", qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        left_rpm_ = msg->data;
      });
    right_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
      "/right_wheel_rpm", qos,
      [this](std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex);
        right_rpm_ = msg->data;
      });

    // Create an odometry publisher.
    odom_pub = ros_node->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Start a ROS spinning thread.
    ros_spinner = std::thread([this]() { rclcpp::spin(ros_node); });

    // Connect to Gazebo update event.
    update_connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RPMDiffDrivePlugin::OnUpdate, this));

    // Initialize last update time.
    last_update_time = model->GetWorld()->SimTime();
  }

  // Called on every simulation update.
  void OnUpdate() {
    // Copy RPM values.
    double left_rpm, right_rpm;
    {
      std::lock_guard<std::mutex> lock(mutex);
      left_rpm = left_rpm_;
      right_rpm = right_rpm_;
    }
    // Convert RPM to rad/s.
    double left_rad_s = left_rpm * (2.0 * M_PI / 60.0);
    double right_rad_s = right_rpm * (2.0 * M_PI / 60.0);

    // Apply velocities to the wheel joints.
    left_joint->SetVelocity(0, left_rad_s);
    right_joint->SetVelocity(0, right_rad_s);

    // Odometry integration.
    gazebo::common::Time current_time = model->GetWorld()->SimTime();
    double dt = (current_time - last_update_time).Double();
    last_update_time = current_time;
    if (dt <= 0) return;

    // Compute linear and angular velocity using differential drive kinematics.
    double v = wheel_radius_ * (left_rad_s + right_rad_s) / 2.0;
    double w = wheel_radius_ * (right_rad_s - left_rad_s) / wheelbase_;

    // Integrate pose.
    x_ += v * std::cos(theta_) * dt;
    y_ += v * std::sin(theta_) * dt;
    theta_ += w * dt;

    // Prepare and publish the odometry message.
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ros_node->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "body_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    // Convert theta to quaternion.
    double half_theta = theta_ / 2.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = std::sin(half_theta);
    odom.pose.pose.orientation.w = std::cos(half_theta);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub->publish(odom);
  }

  virtual ~RPMDiffDrivePlugin() {
    if (ros_node) {
      rclcpp::shutdown();
    }
    if (ros_spinner.joinable()) {
      ros_spinner.join();
    }
  }

private:
  physics::ModelPtr model;
  physics::JointPtr left_joint;
  physics::JointPtr right_joint;
  std::string left_joint_name, right_joint_name;
  event::ConnectionPtr update_connection;

  rclcpp::Node::SharedPtr ros_node;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  std::thread ros_spinner;
  std::mutex mutex;  // Updated: using "mutex" as the variable name
  double left_rpm_{0.0};
  double right_rpm_{0.0};

  // Odometry state.
  double x_{0.0}, y_{0.0}, theta_{0.0};
  gazebo::common::Time last_update_time;
  
  // Wheel parameters.
  double wheel_radius_;
  double wheelbase_;
};

GZ_REGISTER_MODEL_PLUGIN(RPMDiffDrivePlugin)
}  // namespace gazebo

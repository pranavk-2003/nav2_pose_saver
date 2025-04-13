#include <chrono>
#include <fstream>
#include <memory>
#include <cstdio>  // for std::rename
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "nav2_pose_saver/pose_saver_node.hpp"



using namespace std::chrono_literals;

namespace nav2_pose_saver
{

PoseSaverNode::PoseSaverNode()
: Node("pose_saver_node"), saving_active_(false)
{
  this->declare_parameter<double>("save_interval_sec", 5.0);
  this->declare_parameter<std::string>("pose_file_path", std::string(std::getenv("HOME")) + "/.ros/last_known_pose.yaml");

  pose_file_path_ = this->get_parameter("pose_file_path").as_string();
  double interval_sec = this->get_parameter("save_interval_sec").as_double();

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(interval_sec),
    std::bind(&PoseSaverNode::timer_callback, this));
  timer_->cancel();  // Disabled until service starts

  sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10,
    std::bind(&PoseSaverNode::pose_callback, this, std::placeholders::_1));

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/start_pose_saver",
    std::bind(&PoseSaverNode::start_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/stop_pose_saver",
    std::bind(&PoseSaverNode::stop_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  restore_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/localise_at_last_known_position",
    std::bind(&PoseSaverNode::restore_service_cb, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Pose Saver Node Initialized.");
}

void PoseSaverNode::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_ = msg;
}

void PoseSaverNode::timer_callback()
{
  if (!saving_active_ || !last_pose_)
    return;

  try {
    write_pose_to_file(pose_file_path_);
    write_pose_to_file(get_package_config_path());
    RCLCPP_DEBUG(this->get_logger(), "Saved pose to %s and config dir", pose_file_path_.c_str());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to write pose: %s", e.what());
  }
}

void PoseSaverNode::start_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  saving_active_ = true;
  timer_->reset();
  res->success = true;
  res->message = "Pose saving started.";
  RCLCPP_INFO(this->get_logger(), "Pose saving activated.");
}

void PoseSaverNode::stop_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  saving_active_ = false;
  timer_->cancel();
  res->success = true;
  res->message = "Pose saving stopped.";
  RCLCPP_INFO(this->get_logger(), "Pose saving deactivated.");
}

void PoseSaverNode::restore_service_cb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  try {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg = read_pose_from_file(pose_file_path_);
    auto pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    rclcpp::sleep_for(500ms);  // wait for AMCL to be ready
    pub->publish(pose_msg);
    res->success = true;
    res->message = "Pose restored.";
    RCLCPP_INFO(this->get_logger(), "Published initial pose from file.");
  } catch (const std::exception &e) {
    res->success = false;
    res->message = e.what();
    RCLCPP_ERROR(this->get_logger(), "Failed to restore pose: %s", e.what());
  }
}

void PoseSaverNode::write_pose_to_file(const std::string &filepath)
{
  if (!last_pose_) return;

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "pose" << YAML::BeginMap;
  out << YAML::Key << "position" << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.position.x;
  out << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.position.y;
  out << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.position.z;
  out << YAML::EndMap;
  out << YAML::Key << "orientation" << YAML::BeginMap;
  out << YAML::Key << "x" << YAML::Value << last_pose_->pose.pose.orientation.x;
  out << YAML::Key << "y" << YAML::Value << last_pose_->pose.pose.orientation.y;
  out << YAML::Key << "z" << YAML::Value << last_pose_->pose.pose.orientation.z;
  out << YAML::Key << "w" << YAML::Value << last_pose_->pose.pose.orientation.w;
  out << YAML::EndMap;
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::string tmp_path = filepath + ".tmp";
  std::ofstream tmp_out(tmp_path, std::ios::trunc);
  if (!tmp_out.is_open()) {
    throw std::runtime_error("Unable to open temp file for writing: " + tmp_path);
  }
  tmp_out << out.c_str();
  tmp_out.close();

  if (std::rename(tmp_path.c_str(), filepath.c_str()) != 0) {
    throw std::runtime_error("Failed to rename temp file to target file: " + filepath);
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseSaverNode::read_pose_from_file(const std::string &filepath)
{
  YAML::Node node = YAML::LoadFile(filepath);
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = node["pose"]["position"]["x"].as<double>();
  msg.pose.pose.position.y = node["pose"]["position"]["y"].as<double>();
  msg.pose.pose.position.z = node["pose"]["position"]["z"].as<double>();
  msg.pose.pose.orientation.x = node["pose"]["orientation"]["x"].as<double>();
  msg.pose.pose.orientation.y = node["pose"]["orientation"]["y"].as<double>();
  msg.pose.pose.orientation.z = node["pose"]["orientation"]["z"].as<double>();
  msg.pose.pose.orientation.w = node["pose"]["orientation"]["w"].as<double>();
  return msg;
}

std::string PoseSaverNode::get_package_config_path()
{
  std::string package_share = ament_index_cpp::get_package_share_directory("pose_saver");
  return package_share + "/config/last_known_pose.yaml";
}

}  

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_pose_saver::PoseSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

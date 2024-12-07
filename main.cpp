#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudFilterNode : public rclcpp::Node {
public:
  PointCloudFilterNode(const std::string &camera_name);

private:
  std::string camera_name_;

  // Filter parameters
  int white_threshold_;
  double height_threshold_;
  double height_tolerance_;

  // Publisher for filtered point cloud
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

PointCloudFilterNode::PointCloudFilterNode(const std::string &camera_name)
  : Node("point_cloud_filter_node_" + camera_name),
    camera_name_(camera_name),
    white_threshold_(240),
    height_threshold_(0.55),
    height_tolerance_(0.45)
{
  // Set up subscription to camera topic
  auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/" + camera_name_ + "/depth/color/points",
      10,
      std::bind(&PointCloudFilterNode::callback, this, std::placeholders::_1));

  // Set up publisher for filtered point cloud
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_point_cloud/" + camera_name_,
      10);
}

void PointCloudFilterNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert PointCloud2 message to PCL point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);

  // Filter white points
  pcl::PointIndices::Ptr white_indices(new pcl::PointIndices);
  for (int i = 0; i < cloud->size(); i++) {
    auto point = (*cloud)[i];
    if (point.r >= white_threshold_ && point.g >= white_threshold_ && point.b >= white_threshold_) {
      white_indices->indices.push_back(i);
    }
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr white_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_white;
  extract_white.setInputCloud(cloud);
  extract_white.setIndices(white_indices);
  extract_white.setNegative(false);
  extract_white.filter(*white_cloud);

  // Filter points at half meter height
  pcl::PointIndices::Ptr height_indices(new pcl::PointIndices);
  for (int i = 0; i < cloud->size(); i++) {
    auto point = (*cloud)[i];
    if (std::abs(point.z - height_threshold_) < height_tolerance_) {
      height_indices->indices.push_back(i);
    }
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr height_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract_height;
  extract_height.setInputCloud(cloud);
  extract_height.setIndices(height_indices);
  extract_height.setNegative(false);
  extract_height.filter(*height_cloud);

  // Combine filtered point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  *filtered_cloud = *white_cloud + *height_cloud;

  // Convert filtered point cloud back to PointCloud2 message
  sensor_msgs::msg::PointCloud2 filtered_msg;
  pcl::toROSMsg(*filtered_cloud, filtered_msg);
  filtered_msg.header = msg->header;

  // Publish filtered point cloud
  pub_->publish(filtered_msg);
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <numeric>
#include <iostream>

using std::placeholders::_1;

class PointCloudFilterNode : public rclcpp::Node
{
public:
  PointCloudFilterNode(std::string camera_name)
  : Node("point_cloud_filter_node_" + camera_name),
    camera_name_(camera_name),
    white_threshold_(240),
    height_threshold_(0.55),
    height_tolerance_(0.45)
  {
    // Set up subscription to camera topic
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/" + camera_name + "/depth/color/points",
      10,
      std::bind(&PointCloudFilterNode::callback, this, _1));

    // Set up publisher for filtered point cloud
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_point_cloud/" + camera_name,
      10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void filter_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg, sensor_msgs::msg::PointCloud2& filtered_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  std::string camera_name_;
  int white_threshold_;
  float height_threshold_;
  float height_tolerance_;
};

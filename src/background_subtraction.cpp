#include "background_subtraction/background_subtraction.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/octree/octree_pointcloud_changedetector.h"


namespace background_subtraction
{

BackgroundSubtraction::BackgroundSubtraction()
: Node("background_subtraction")
{
  this->declare_parameter<std::string>("input_topic", "input_point_cloud");
  this->declare_parameter<std::string>("output_topic", "background_subtracted_point_cloud");

  std::string input_topic = this->get_parameter("input_topic").as_string();
  std::string output_topic = this->get_parameter("output_topic").as_string();

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, 10, std::bind(&BackgroundSubtraction::pointCloudCallback, this, std::placeholders::_1));

  background_subtracted_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
}

BackgroundSubtraction::~BackgroundSubtraction()
{
}

void BackgroundSubtraction::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  subtractBackground(msg);
}

void BackgroundSubtraction::subtractBackground(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Implement your background subtraction algorithm here
  // For simplicity, let's assume we just pass the input point cloud through

  auto output_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msg, *input_cloud);

  // Update the background model with the current frame
  if (!background_model_) {
    background_model_ = input_cloud;
  } else {
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(0.05);
    octree.setInputCloud(background_model_);
    octree.addPointsFromInputCloud();

    octree.switchBuffers();

    octree.setInputCloud(input_cloud);
    octree.addPointsFromInputCloud();

    std::vector<int> new_point_indices;
    octree.getPointIndicesFromNewVoxels(new_point_indices);

    for (const auto& idx : new_point_indices) {
      output_cloud->points.push_back(input_cloud->points[idx]);
    }

    // Update the background model
    background_model_ = input_cloud;
  }

  pcl::toROSMsg(*output_cloud, *output_msg);
  output_msg->header = msg->header;
  background_subtracted_pub_->publish(*output_msg);
}

}  // namespace background_subtraction

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<background_subtraction::BackgroundSubtraction>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
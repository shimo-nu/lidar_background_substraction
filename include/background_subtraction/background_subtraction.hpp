#ifndef BACKGROUND_SUBTRACTION_HPP_
#define BACKGROUND_SUBTRACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

namespace background_subtraction
{

class BackgroundSubtraction : public rclcpp::Node
{
public:
  BackgroundSubtraction();
  ~BackgroundSubtraction();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void subtractBackground(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr background_subtracted_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr background_model_;
};

}  // namespace background_subtraction

#endif  // BACKGROUND_SUBTRACTION_HPP_
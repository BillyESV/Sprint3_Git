/**
 * @file cylinder.cpp
 * @brief ROS 2 node that subscribes to /scan topic and visualizes LaserScan data using OpenCV.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <opencv2/opencv.hpp>

/**
 * @class CylinderNode
 * @brief A ROS 2 node that processes LaserScan data and displays it as an image.
 */
class CylinderNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for CylinderNode.
   */
  CylinderNode() : Node("cylinder_node")
  {
    // Subscribe to the /scan topic
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&CylinderNode::scan_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CylinderNode has been started.");
  }

private:
  /**
   * @brief Callback function for processing LaserScan messages.
   * @param scan_msg Shared pointer to the LaserScan message.
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // Convert LaserScan data to an OpenCV image
    cv::Mat scan_image = laserScanToMat(scan_msg);

    // Display the image
    cv::imshow("Laser Scan", scan_image);
    cv::waitKey(1); // Necessary to update the image window
  }

  /**
   * @brief Converts LaserScan data to an OpenCV Mat image.
   * @param scan Shared pointer to the LaserScan message.
   * @return OpenCV Mat image representing the LaserScan data.
   */
  cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
  {
    // Parameters
    int img_size = 500;
    float max_range = scan->range_max;

    cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

    for (size_t i = 0; i < scan->ranges.size(); i++)
    {
      float range = scan->ranges[i];
      if (range > scan->range_min && range < scan->range_max)
      {
        float angle = scan->angle_min + i * scan->angle_increment;
        int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
        int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;
        if (x >= 0 && x < img_size && y >= 0 && y < img_size)
        {
          image.at<uchar>(y, x) = 255;
        }
      }
    }

 

    // Return the image with the largest circle drawn
    return image;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and spin the node
  rclcpp::spin(std::make_shared<CylinderNode>());

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}

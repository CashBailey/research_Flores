#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>

class ImageSaver : public rclcpp::Node
{
public:
  ImageSaver(const std::string & topic_name)
  : Node("image_saver_node"),
    output_path_("/home/c/ros2_ws/src/my_cpp_pkg/image_reference/reference_image.jpg")
  {
    // Create image transport subscription
    // For typical camera data, it's common to use a sensor-data QoS profile:
    rclcpp::SensorDataQoS qos;
    subscriber_ = image_transport::create_subscription(
      this,
      topic_name,
      std::bind(&ImageSaver::image_callback, this, std::placeholders::_1),
      "raw",
      qos.get_rmw_qos_profile());

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on topic: %s. Images will be saved to: %s",
      topic_name.c_str(),
      output_path_.c_str());
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      // Convert ROS image to OpenCV format
      cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;

      // Save image to file
      cv::imwrite(output_path_, image);
      RCLCPP_DEBUG(
        this->get_logger(),
        "Successfully saved image to: %s",
        output_path_.c_str());
    }
    catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const cv::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
    }
  }

  // Member variables
  const std::string output_path_;
  image_transport::Subscriber subscriber_;
};

int main(int argc, char ** argv)
{
  // This initializes OpenCV’s GUI thread. The original code called cv::startWindowThread()
  // so we keep it here. It’s only required if you plan to use cv::imshow and similar.
  cv::startWindowThread();

  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Get topic name from command line or use a default
  std::string topic_name = "/my_image";
  if (argc >= 2) {
    topic_name = argv[1];
  } else {
    std::cout << "Using default topic: " << topic_name << std::endl;
  }

  auto node = std::make_shared<ImageSaver>(topic_name);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



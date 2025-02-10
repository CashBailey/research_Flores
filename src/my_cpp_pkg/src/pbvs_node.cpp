#include "my_cpp_pkg/vision_utils.hpp"
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdlib.h>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

cv::Mat refImage, currentImage;
double t0,t;

static const std::string REF_IMAGE_NAME = "/home/c/ros2_ws/src/my_cpp_pkg/image_reference/reference_image.jpg";

class PBVSNode : public rclcpp::Node {
public:
    PBVSNode() : Node("pbvs_node") {
        refImage = cv::imread(REF_IMAGE_NAME, cv::IMREAD_COLOR);
        if(refImage.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not read the image: %s", REF_IMAGE_NAME.c_str());
            rclcpp::shutdown();
            exit(1);
        } else {
            RCLCPP_INFO(this->get_logger(), "Reference image is ready: %s", REF_IMAGE_NAME.c_str());
        }

        topic_name_ = this->declare_parameter<std::string>("topic_name", "/my_image");
        lambdav_ = this->declare_parameter<double>("lambdav", 1.0);
        lambdaw_ = this->declare_parameter<double>("lambdaw", 1.0);

        homography_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/homograpy_numerical", 10);
        ck_t_ct_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ck_t_ct", 10);
        n_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/n_plane_vector", 10);
        Uv_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Uv", 10);
        Uw_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/Uw", 10);
        d_pub_ = this->create_publisher<std_msgs::msg::Float64>("/d_value", 10);

        image_transport::ImageTransport it(this);
        image_subscription_ = it.subscribe(topic_name_, 1, std::bind(&PBVSNode::imageCallback, this));

        timer_ = this->create_wall_timer(100ms, std::bind(&PBVSNode::controlLoop, this));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
        cv_bridge::CvImagePtr cvPtr;
        try {
            cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cvPtr->image.copyTo(currentImage);
    }

    void controlLoop() {
        if(currentImage.empty()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for current image...");
            return;
        }

        cv::imwrite("/tmp/current_image.jpg", currentImage);

        Matrix_t K(3,3);
        Matrix_t H(3,3);
        Matrix_t R(3,3);
        Vector3D_t t;
        Vector3D_t n;
        Vector3D_t u;
        Vector3D_t Uv;
        Vector3D_t Uw;
        FPTYPE d;
        homographySolution homSolution;
        static int counter = 0;

        K << 554.382713,    0.0     , 320.0,
             0.0         , 554.382713, 240.0,
             0.0         ,     0.0     , 1.0;

        EstimateHomography(refImage, currentImage, K, H, counter );
        RecoverFromHomography(H,R,t,n,d, counter, homSolution);
        Rodriguez(R,u);
        PBVSController(R,t,u,Uv,Uw, lambdav_, lambdaw_);

        std_msgs::msg::Float64MultiArray H_msg, ck_t_ct_msg, n_msg, Uv_msg, Uw_msg;
        std_msgs::msg::Float64 d_msg;

        H_msg.data = std::vector<double>(H.data(), H.data() + H.size());
        ck_t_ct_msg.data = std::vector<double>(t.data(), t.data() + t.size());
        n_msg.data = std::vector<double>(n.data(), n.data() + n.size());
        Uv_msg.data = std::vector<double>(Uv.data(), Uv.data() + Uv.size());
        Uw_msg.data = std::vector<double>(Uw.data(), Uw.data() + Uw.size());

        d_msg.data = d;

        homography_pub_->publish(H_msg);
        ck_t_ct_pub_->publish(ck_t_ct_msg);
        n_pub_->publish(n_msg);
        d_pub_->publish(d_msg);
        Uv_pub_->publish(Uv_msg);
        Uw_pub_->publish(Uw_msg);

        counter++;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr homography_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ck_t_ct_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr n_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Uv_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr Uw_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr d_pub_;
    image_transport::Subscriber image_subscription_;
    std::string topic_name_;
    double lambdav_;
    double lambdaw_;
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PBVSNode>());
    rclcpp::shutdown();
    return 0;
}

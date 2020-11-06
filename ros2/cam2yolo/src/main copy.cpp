// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/highgui/highgui.hpp" // display
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp> // rectangle

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

#include "cam2yolo.hpp"



class Cam2Image : public rclcpp::Node
{
public:
  //IMAGE_TOOLS_PUBLIC
  explicit Cam2Image(const rclcpp::NodeOptions & options)
  : Node("cam2image", options),
    is_flipped_(false),
    publish_number_(1u)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Do not execute if a --help option was provided
    //if (help(options.arguments())) {
      // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
      // from a container.
      //exit(0);
    //}
    parse_parameters();
    initialize();
  }
    void start(){
        cv::Mat frame;
        cap.read(frame);
        std::chrono::nanoseconds time_out(1000);
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->is_bigendian = false;
        msg->height = height_;
        msg->width = width_;
        msg->encoding = mat_type2encoding(frame.type());
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        size_t size = frame.step * frame.rows;
        msg->data.resize(size);
        
        rclcpp::executors::SingleThreadedExecutor exe;
        
        while (true) { //&& cv::waitKey(1)!=27 && !frame.empty()
            //cv::imshow(frame_id_, frame);
            cv::waitKey(1);
            memcpy(&msg->data[0], frame.data, size); // conversion
            cap >> frame;
            
            RCLCPP_INFO(this->get_logger(), "Publishing image #%zd", publish_number_++);
            std::cout<< "before publish" << std::endl;
            pub_->publish(std::move(msg));
            std::cout<< "published" << std::endl;
            exe.spin_once(time_out);
            std::cout<< "spinned" << std::endl;
            if (!rclcpp::ok()) {
                std::cout<< "not ok" << std::endl;
                break;
            }

        }
        std::cout<< "out" << std::endl;
    }

private:
  //IMAGE_TOOLS_LOCAL
  void initialize()
  {
    auto qos = rclcpp::SensorDataQoS(); // https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h

    pub_ = create_publisher<sensor_msgs::msg::Image>("/images", qos);

    // Subscribe to a message that will toggle flipping or not flipping, and manage the state in a
    // callback
    /*auto callback = [this](const std_msgs::msg::Bool::SharedPtr msg) -> void
      {
        this->is_flipped_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Set flip mode to: %s", this->is_flipped_ ? "on" : "off");
      };*/
    // Set the QoS profile for the subscription to the flip message.
    sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/detections", qos, std::bind(&Cam2Image::detection_callback, this, std::placeholders::_1));

    //if (!burger_mode_) {
      // Initialize OpenCV video capture stream.
      //for (int i =0; i<100; i++)
      cap.open(0);

      // Set the width and height based on command line arguments.
      cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
      cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
      if (!cap.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        throw std::runtime_error("Could not open video stream");
      }
    //}
      std::cout << "camera fps : " << cap.get(cv::CAP_PROP_FPS) << std::endl;
      cap >> cam_;
      //im_msg_ = std::make_unique<sensor_msgs::msg::Image>();
      im_msg_.is_bigendian = false;
      im_msg_.height = height_;
      im_msg_.width = width_;
      im_msg_.encoding = mat_type2encoding(cam_.type());
      im_msg_.header.frame_id = frame_id_;
      std::cout << "encoding : " << im_msg_.encoding << std::endl;
      im_msg_.step = static_cast<sensor_msgs::msg::Image::_step_type>(cam_.step);
      std::cout << "rows : " << cam_.rows << " columns : " << cam_.cols << std::endl;
      size_ = cam_.step * height_;
      //std::cout << "height : " << height_ << std::endl;
      std::cout << "size : " << size_ << " step : " << im_msg_.step << std::endl;
      im_msg_.data.resize(size_);
      
    // Start main timer loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
      std::bind(&Cam2Image::timerCallback, this));
  }

  /// Publish camera, or burger, image.
  //IMAGE_TOOLS_LOCAL
  void timerCallback()
  {
      /*cv::Mat frame;

      // Initialize a shared pointer to an Image message.
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->is_bigendian = false;*/
      cap >> cam_;
      to_disp_=cam_.clone();
      draw_detections(to_disp_, D2A_msg_);
      if (is_flipped_) {
          cv::flip(to_disp_,to_pub_,1);
          to_disp_=to_pub_.clone();
      }
      cv::imshow(frame_id_, to_disp_);
      cv::waitKey(1);
      
      cv::cvtColor(cam_, to_pub_, cv::COLOR_BGR2RGB);
      im_msg_.data.resize(size_);
      memcpy(&im_msg_.data[0], to_pub_.data, size_); // conversion
      //convert_frame_to_message(frame, *msg);
      //std::cout << "rows : " << frame_.rows << " columns : " << frame_.cols << std::endl;
      //std::cout << frame_.step * frame_.rows << std::endl;
    // Publish the image message and increment the frame_id.
    //RCLCPP_INFO(get_logger(), "Publishing image #%zd", publish_number_++);
      printf("\33[2K\rPublishing image #%06zd\tMessages received %06zd", publish_number_++, subscribe_number_);
    pub_->publish(std::move(im_msg_));
  }
/*
  IMAGE_TOOLS_LOCAL
  bool help(const std::vector<std::string> args)
  {
    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
      std::find(args.begin(), args.end(), "-h") != args.end())
    {
      std::stringstream ss;
      ss << "Usage: cam2image [-h] [--ros-args [-p param:=value] ...]" << std::endl;
      ss << "Publish images from a camera stream." << std::endl;
      ss << "Example: ros2 run image_tools cam2image --ros-args -p reliability:=best_effort";
      ss << std::endl << std::endl;
      ss << "Options:" << std::endl;
      ss << "  -h, --help\tDisplay this help message and exit";
      ss << std::endl << std::endl;
      ss << "Parameters:" << std::endl;
      ss << "  reliability\tReliability QoS setting. Either 'reliable' (default) or 'best_effort'";
      ss << std::endl;
      ss << "  history\tHistory QoS setting. Either 'keep_last' (default) or 'keep_all'.";
      ss << std::endl;
      ss << "\t\tIf 'keep_last', then up to N samples are stored where N is the depth";
      ss << std::endl;
      ss << "  depth\t\tDepth of the publisher queue. Only honored if history QoS is 'keep_last'.";
      ss << " Default value is 10";
      ss << std::endl;
      ss << "  frequency\tPublish frequency in Hz. Default value is 30";
      ss << std::endl;
      ss << "  burger_mode\tProduce images of burgers rather than connecting to a camera";
      ss << std::endl;
      ss << "  show_camera\tShow camera stream. Either 'true' or 'false' (default)";
      ss << std::endl;
      ss << "  device_id\tDevice ID of the camera. 0 (default) selects the default camera device.";
      ss << std::endl;
      ss << "  width\t\tWidth component of the camera stream resolution. Default value is 320";
      ss << std::endl;
      ss << "  height\tHeight component of the camera stream resolution. Default value is 240";
      ss << std::endl;
      ss << "  frame_id\t\tID of the sensor frame. Default value is 'camera_frame'";
      ss << std::endl << std::endl;
      ss << "Note: try running v4l2-ctl --list-formats-ext to obtain a list of valid values.";
      ss << std::endl;
      std::cout << ss.str();
      return true;
    }
    return false;
  }
    */
    void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg){
        // how to handle Detection2DArray: https://github.com/ros2/openrobotics_darknet_ros/blob/master/src/detector_network.cpp
        //std::cout << "detection received" << std::endl;
        D2A_msg_ = *msg;
        subscribe_number_++;
    }
    void draw_detections(cv::Mat& frame, const vision_msgs::msg::Detection2DArray& msg){
        // https://github.com/AlexeyAB/darknet/blob/91b5dd2da7dcb9acc69d129e504ad9ef83045c4c/src/image_opencv.cpp#L878
        int imax;
        std::string id;
        double score;
        cv::Point pt1, pt2;
        cv::Scalar color;
        //std::cout << "before draw" << std::endl;
        for (auto const& detection: msg.detections) {
            //std::cout << "draw loop" << std::endl;
            imax = results_arg_max(detection.results);
            id = detection.results[imax].id;
            score = detection.results[imax].score;
            color = get_color(classes.at(id) * 123457 % nclasses_, nclasses_);
            pt1.x = detection.bbox.center.x - detection.bbox.size_x/2;
            pt1.y = detection.bbox.center.y - detection.bbox.size_y/2;
            pt2.x = detection.bbox.center.x + detection.bbox.size_x/2;
            pt2.y = detection.bbox.center.y + detection.bbox.size_y/2;
            //std::cout << " pt1 " << pt1 << " pt2 " << pt2 << std::endl;
            cv::rectangle(frame, pt1, pt2, color);
            cv::putText(frame, id+" "+std::to_string(static_cast<int>(score*100))+"%",
                        pt1, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar());
        }
    }
  //IMAGE_TOOLS_LOCAL
  void parse_parameters()
  {/*
    // Parse 'reliability' parameter
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for the image publisher";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_reliability_policy_map) {
      reliability_desc.additional_constraints += entry.first + " ";
    }
    const std::string reliability_param = this->declare_parameter(
      "reliability", "reliable", reliability_desc);
    auto reliability = name_to_reliability_policy_map.find(reliability_param);
    if (reliability == name_to_reliability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS reliability setting '" << reliability_param << "'";
      throw std::runtime_error(oss.str());
    }
    reliability_policy_ = reliability->second;

    // Parse 'history' parameter
    rcl_interfaces::msg::ParameterDescriptor history_desc;
    history_desc.description = "History QoS setting for the image publisher";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry : name_to_history_policy_map) {
      history_desc.additional_constraints += entry.first + " ";
    }
    const std::string history_param = this->declare_parameter(
      "history", name_to_history_policy_map.begin()->first, history_desc);
    auto history = name_to_history_policy_map.find(history_param);
    if (history == name_to_history_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS history setting '" << history_param << "'";
      throw std::runtime_error(oss.str());
    }
    history_policy_ = history->second;

    // Declare and get remaining parameters
    depth_ = this->declare_parameter("depth", 10);
    
    show_camera_ = this->declare_parameter("show_camera", false);
    device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));*/
      freq_ = this->declare_parameter("frequency", 15.0);
    width_ = this->declare_parameter("width", 640);
    height_ = this->declare_parameter("height", 480);
    //rcl_interfaces::msg::ParameterDescriptor burger_mode_desc;
    //burger_mode_desc.description = "Produce images of burgers rather than connecting to a camera";
    //burger_mode_ = this->declare_parameter("burger_mode", false, burger_mode_desc);
    frame_id_ = this->declare_parameter("frame_id", "camera_frame");
  }

  /// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
  /**
   * \param[in] mat_type The OpenCV encoding type.
   * \return A string representing the encoding type.
   */
  //IMAGE_TOOLS_LOCAL
  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
  }

  /// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
  /**
   * \param[in] frame The OpenCV matrix/image to convert.
   * \param[in] frame_id ID for the ROS message.
   * \param[out] Allocated shared pointer for the ROS Image message.
   */
  //IMAGE_TOOLS_LOCAL
  void convert_frame_to_message(
    const cv::Mat & frame, sensor_msgs::msg::Image & msg)
  {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);
    msg.header.frame_id = frame_id_;
  }

  cv::VideoCapture cap;
  cv::Mat cam_, to_disp_, to_pub_;
  //burger::Burger burger_cap;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  vision_msgs::msg::Detection2DArray D2A_msg_;
  //std::unique_ptr<sensor_msgs::msg::Image> im_msg_;
  sensor_msgs::msg::Image im_msg_;

  // ROS parameters
  //bool show_camera_;
  //size_t depth_;
  double freq_;
  //rmw_qos_reliability_policy_t reliability_policy_;
  //rmw_qos_history_policy_t history_policy_;
  size_t width_, height_, size_;
  //bool burger_mode_;
  std::string frame_id_;
  //int device_id_;
    int nclasses_=80;
  /// If true, will cause the incoming camera image message to flip about the y-axis.
  bool is_flipped_;
  /// The number of images published.
  size_t publish_number_, subscribe_number_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
  // auto node = rclcpp::Node::make_shared("ObiWan");
    auto node = std::make_shared<Cam2Image>(options);
  //RCLCPP_INFO(node->get_logger(),"Help me Obi-Wan Kenobi, you're my only hope");
    rclcpp::spin(node);
  rclcpp::shutdown();
std::cout<< "end" << std::endl;
  return 0;
}


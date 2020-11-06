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


#include "cam2yolo.hpp"

Cam2Image::Cam2Image(const rclcpp::NodeOptions & options)
: Node("cam2image", options), publish_number_(1u)//, is_flipped_(false)
{
  
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  freq_ = this->declare_parameter("frequency", 15.0);
  width_ = this->declare_parameter("width", 640);
  height_ = this->declare_parameter("height", 480);
  frame_id_ = this->declare_parameter("frame_id", "camera_frame");
  
  auto qos = rclcpp::SensorDataQoS(); // https://github.com/ros2/rmw/blob/master/rmw/include/rmw/qos_profiles.h
  pub_ = create_publisher<sensor_msgs::msg::Image>("detector_node/images", qos);
  sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    "detector_node/detections", qos, std::bind(&Cam2Image::detection_callback, this, std::placeholders::_1));

  cap.open(0);// init the cam
  cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
    throw std::runtime_error("Could not open video stream");
  }
  std::cout << "camera fps : " << cap.get(cv::CAP_PROP_FPS) << std::endl;
  cap >> cam_;

  // init the message
  im_msg_.is_bigendian = false;
  im_msg_.height = height_;
  im_msg_.width = width_;
  im_msg_.encoding = "bgr8"; // CV_8UC3 -> bgr8
  im_msg_.header.frame_id = frame_id_;
  std::cout << "encoding : " << im_msg_.encoding << std::endl;
  im_msg_.step = static_cast<sensor_msgs::msg::Image::_step_type>(cam_.step);
  std::cout << "rows : " << cam_.rows << " columns : " << cam_.cols << std::endl;
  size_ = cam_.step * height_;
  std::cout << "size : " << size_ << " step : " << im_msg_.step << std::endl;
  im_msg_.data.resize(size_);
  
// Start main timer loop
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
    std::bind(&Cam2Image::timerCallback, this));
}

void Cam2Image::detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg){
  D2A_msg_ = *msg;
  subscribe_number_++;
}

void Cam2Image::timerCallback(){
  cap >> cam_;
  to_disp_=cam_.clone();
  draw_detections(to_disp_, D2A_msg_);
  //if (is_flipped_) {
  //    cv::flip(to_disp_,to_pub_,1);
  //    to_disp_=to_pub_.clone();
  //}
  cv::imshow(frame_id_, to_disp_);
  cv::waitKey(1);
  
  cv::cvtColor(cam_, to_pub_, cv::COLOR_BGR2RGB);
  im_msg_.data.resize(size_);
  memcpy(&im_msg_.data[0], to_pub_.data, size_); // conversion mat to msg

  printf("\33[2K\rPublishing image #%06zd\tMessages received %06zd", publish_number_++, subscribe_number_);
  pub_->publish(std::move(im_msg_));
}

void Cam2Image::draw_detections(cv::Mat& frame, const vision_msgs::msg::Detection2DArray& msg){
  int imax;
  std::string id;
  double score;
  cv::Point pt1, pt2;
  cv::Scalar color;
  for (auto const& detection: msg.detections) {
      imax = results_arg_max(detection.results);
      id = detection.results[imax].id;
      score = detection.results[imax].score;
      color = get_color(classes.at(id) , nclasses_);
      pt1.x = detection.bbox.center.x - detection.bbox.size_x/2;
      pt1.y = detection.bbox.center.y - detection.bbox.size_y/2;
      pt2.x = detection.bbox.center.x + detection.bbox.size_x/2;
      pt2.y = detection.bbox.center.y + detection.bbox.size_y/2;
      cv::rectangle(frame, pt1, pt2, color);
      cv::putText(frame, id+" "+std::to_string(static_cast<int>(score*100))+"%",
                  pt1, cv::FONT_HERSHEY_COMPLEX_SMALL, 0.5, cv::Scalar());
  }
}


constexpr float colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
cv::Scalar get_color(int x, int max)
{// https://github.com/pjreddie/darknet/blob/master/src/image.c
    float v = x * 123457 % max;
    float ratio = (v/max)*5;
    int i = floor(ratio);
    int j = ceil(ratio);
    ratio -= i;
    return
                          cv::Scalar(
                                    (1-ratio) * colors[i][0] + ratio*colors[j][0],
                                    (1-ratio) * colors[i][1] + ratio*colors[j][1],
                                    (1-ratio) * colors[i][2] + ratio*colors[j][2] )
                          *256;

}

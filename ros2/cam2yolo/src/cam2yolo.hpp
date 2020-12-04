#ifndef CAM2YOLO_H
#define CAM2YOLO_H

#include <map>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"


inline const std::map<std::string, int> classes={{"person",0}, {"bicycle",1}, {"car",2}, {"motorbike",3}, {"aeroplane",4}, {"bus",5}, {"train",6}, {"truck",7}, {"boat",8}, {"traffic light",9}, {"fire hydrant",10}, {"stop sign",11}, {"parking meter",12}, {"bench",13}, {"bird",14}, {"cat",15}, {"dog",16}, {"horse",17}, {"sheep",18}, {"cow",19}, {"elephant",20}, {"bear",21}, {"zebra",22}, {"giraffe",23}, {"backpack",24}, {"umbrella",25}, {"handbag",26}, {"tie",27}, {"suitcase",28}, {"frisbee",29}, {"skis",30}, {"snowboard",31}, {"sports ball",32}, {"kite",33}, {"baseball bat",34}, {"baseball glove",35}, {"skateboard",36}, {"surfboard",37}, {"tennis racket",38}, {"bottle",39}, {"wine glass",40}, {"cup",41}, {"fork",42}, {"knife",43}, {"spoon",44}, {"bowl",45}, {"banana",46}, {"apple",47}, {"sandwich",48}, {"orange",49}, {"broccoli",50}, {"carrot",51}, {"hot dog",52}, {"pizza",53}, {"donut",54}, {"cake",55}, {"chair",56}, {"sofa",57}, {"pottedplant",58}, {"bed",59}, {"diningtable",60}, {"toilet",61}, {"tvmonitor",62}, {"laptop",63}, {"mouse",64}, {"remote",65}, {"keyboard",66}, {"cell phone",67}, {"microwave",68}, {"oven",69}, {"toaster",70}, {"sink",71}, {"refrigerator",72}, {"book",73}, {"clock",74}, {"vase",75}, {"scissors",76}, {"teddy bear",77}, {"hair drier",78}, {"toothbrush",79}};


template <typename F>
struct CompareBy
{
   bool operator()(const typename F::argument_type& x,
                   const typename F::argument_type& y)
   { return f(x) < f(y); }

   CompareBy(const F& f) : f(f) {}

private:
   F f;
};


template <typename T, typename U>
struct Member : std::unary_function<U, T>
{
   Member(T U::*ptr) : ptr(ptr) {}
   const T& operator()(const U& x) { return x.*ptr; }

private:
   T U::*ptr;
};

template <typename F>
CompareBy<F> by(const F& f) { return CompareBy<F>(f); }

template <typename T, typename U>
Member<T, U> mem_ptr(T U::*ptr) { return Member<T, U>(ptr); }

template <typename T, typename A>
int results_arg_max(std::vector<T, A> const& vec) {
    //https://stackoverflow.com/a/5085323
  return static_cast<int>(std::distance(vec.begin(), std::max_element(vec.begin(), vec.end(), by(mem_ptr(&T::score)))));
}

template <typename T, typename A>
int arg_max(std::vector<T, A> const& vec) {
  return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
}

cv::Scalar get_color(int x, int max);

class Cam2yolo : public rclcpp::Node{
  // https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp
private:
  cv::VideoCapture cap;
  cv::Mat cam_, to_disp_, to_pub_;

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  vision_msgs::msg::Detection2DArray D2A_msg_;
  sensor_msgs::msg::Image im_msg_;

  double freq_;
  size_t width_, height_, size_;
  std::string frame_id_;
  int nclasses_=80;
  size_t publish_number_, subscribe_number_;
  //bool is_flipped_;
  
  void timerCallback();
  // how to use Detection2DArray: https://github.com/ros2/openrobotics_darknet_ros/blob/master/src/detector_network.cpp
  void detection_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  // https://github.com/AlexeyAB/darknet/blob/91b5dd2da7dcb9acc69d129e504ad9ef83045c4c/src/image_opencv.cpp#L878
  void draw_detections(cv::Mat& frame, const vision_msgs::msg::Detection2DArray& msg);
public:
  explicit Cam2yolo(const rclcpp::NodeOptions & options);
  
};

#endif

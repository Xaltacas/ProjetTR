#!/bin/bash

set -e

sed -i 's/free_network(network_);/free_network_ptr(network_); \/\/ alexeyAB/g' detector_network.cpp
sed -i 's/set_batch_network(impl_->network_, batch);/ \/\/set_batch_network(impl_->network_, batch); \/\/ alexeyAB/g' detector_network.cpp
sed -i 's/network_predict(impl_->network_, resized_image.image_.data);/ network_predict_ptr(impl_->network_, resized_image.image_.data); \/\/ alexeyAB/g' detector_network.cpp
sed -i 's/&num_detections);/&num_detections,0); \/\/ alexeyAB/g' detector_network.cpp

sed -i 's/set_on_parameters_set_callback(/ add_on_set_parameters_callback(/g' detector_node.cpp
sed -i '/impl_->detections_pub_/i  auto qos = rclcpp::SensorDataQoS();' detector_node.cpp
sed -i 's/detections", 1);/detections", qos);/g' detector_node.cpp
sed -i 's/images", 12,/images", qos,/g' detector_node.cpp

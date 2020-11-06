# Raspberry-ROS2-YOLO

## Raspberry Pi

Installation des packages
```shell
cd
source ros2_foxy/install/setup.sh
mkdir -p dev_ws/src && cd dev_ws/src
git clone -b ros2 https://github.com/Kukanani/vision_msgs.git
```
Puis, ajouter le package cam2yolo dans `src`.
Enfin,
```shell
cd ~/dev_ws
colcon build
source install/setup.sh
ros2 run cam2yolo cam2yolo
```

## Ordi
Installation des packages
```shell
cd
source ros2_foxy/install/setup.sh
mkdir -p dev_ws/src
cd dev_ws/src
git clone -b ros2 https://github.com/Kukanani/vision_msgs.git
git clone https://github.com/ros2/openrobotics_darknet_ros.git
git clone https://github.com/ros2/darknet_vendor.git
git clone -b ros2 https://github.com/ros-perception/vision_opencv.git
```
- Técharger les poids: [yolov4-tiny.weights](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights) | 
[yolov4.weights](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights)

Il faut ensuite faire des modifications à la main pour le rendre compatible avec [AlexeyAB](https://github.com/AlexeyAB/darknet) (je n'ai pas réussi à faire fonctioner avec le répertoire officiel).
- remplacer `darknet_vendor/CMakeLists.txt` par le CMakeLists.txt fourni
- modifier le fichier `openrobotics_darknet_ros/detector_network.cpp`
```shell
# PATCH
cd ~/dev_ws/src/openrobotics_darknet_ros
sed -i 's/free_network(network_);/free_network_ptr(network_); \/\/ alexeyAB/g' detector_network.cpp
sed -i 's/set_batch_network(impl_->network_, batch);/ \/\/set_batch_network(impl_->network_, batch); \/\/ alexeyAB/g' detector_network.cpp
sed -i 's/network_predict(impl_->network_, resized_image.image_.data);/ network_predict_ptr(impl_->network_, resized_image.image_.data); \/\/ alexeyAB/g' detector_network.cpp
sed -i 's/num_detections);/num_detections,0); \/\/ alexeyAB/g' detector_network.cpp
```
- modifier le fichier `openrobotics_darknet_ros/detector_node.cpp`
```shell
cd ~/dev_ws/src/openrobotics_darknet_ros
sed -i 's/set_on_parameters_set_callback(/ add_on_set_parameters_callback(/g' detector_node.cpp
sed -i '/impl_->detections_pub_/i  auto qos = rclcpp::SensorDataQoS();' detector_node.cpp 
sed -i 's/detections", 1);/detections", qos);/g' detector_node.cpp
sed -i 's/images", 12;/ images", qos;/g' detector_node.cpp
```
- remplacer la ligne 99 par `rclcpp::ParameterValue("<PATH_OF_yolo.cfg>"),` exemple: $HOME/dev_ws/install/darknet_vendor/share/darknet/cfg/yolov4-tiny.cfg
- remplacer la ligne 109 par `rclcpp::ParameterValue("<PATH_OF_yolo.weights>"),` 
- remplacer la ligne 119 par `rclcpp::ParameterValue("<PATH_OF_coco.names>"),` exemple: $HOME/dev_ws/install/darknet_vendor/share/darknet/cfg/coco.names

Il faut indiquer le chemin absolu en dur car je n'ai pas réussi à lancer le programme comme indiqué dans la [doc officiel](https://github.com/ros2/openrobotics_darknet_ros).
- Dans le fichier `.cfg`, décommenter la ligne `#batch=1`, optionnellement, modifier la résolution (`height` et `width` doivent être un multiple de 32).
Enfin,
```shell
cd ~/dev_ws
colcon build
source install/setup.sh
rosrun openrobotics_darknet_ros detector_node
```
# gesture_detec_demo Package

## 1.概述
实现了手势识别和显示，以及手势的ROS消息发布。
## 2.如何运行
安装以下依赖库，可用pip安装
- opencv-python
- numpy
- imutils
- operator

在终端运行以下命令，启动摄像头节点
```
roslaunch realsense2_camera rs_camera.launch
```
在新终端运行以下命令，启动手势识别demo节点
```
rosrun gesture_detec_demo GestureRos.py
```
## 3.节点API
### 3.1.话题
#### 3.1.1.Subscribed Topics
|名称|类型|说明|
|:-:|:-:|:-:|
|/camera/color/image_raw|[Image][sensor_msgs/Image]|图像消息|
- /camera/color/image_raw  
  ROS标准的图像消息，参考类型链接。
#### 3.1.2.Published Topics
|名称|类型|说明|
|:-:|:-:|:-:|
|/camera/gesture|Gesture|识别的手势消息|
- /camera/gesture  
  手势识别Demo节点发布的识别手势结果，消息定义如下
  ```
  string[] lableList
  uint8[] indexList
  ```
  分别代表手势标签列表；当前识别到的手势在标签表的索引，多个索引值代表识别到多个手势。
### 3.2.服务
无
### 3.3.参数
无
## 附录
[sensor_msgs/Image]: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
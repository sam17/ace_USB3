# ace_USB3
A ROS Driver for Basler Ace USB 3.0 Camera using Pleora SDK

![image](http://s.baslerweb.com/fp-1424089632/media/assets/store/product/00/00/00/00/00/00/0f/1f/3ac7f0bfedfd6055dc05abdcd750a6b1.jpg)

## Supported hardware
This driver should work at least with a Basler USB 3.0 Ace Area-scan camera ARM Version.

## ROS API

### flir_gige_node

`ros_pleora` is a driver for a Basler Ace USB 3.0 camera.

#### Published topics

`~image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))  
    The unprocessed image data.

## Installing Pleora eBUS SDK
You can find the latest version of Pleora eBus SDK from [here](http://www.pleora.com/support-center/documentation-downloads).  

To install, run the following command:

```bash
cd install
sudo ./eBUS_SDK_X.X.X.X_armhf-X.run
# accept all default options
```

This will install the eBUS SDK to `/opt/pleora`. 

## Running the node
Running the node is easy. Just do

```
roscore &
rosrun ace_pleora_wrapper ros_pleora
```
##Additional Features
 A test code using only OpenCV to test camera grabbing and siplaying with OpenCV is added in the test folder.

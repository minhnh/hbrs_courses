# ROS Exercises on youBot - Ball Detector Task
## 1. Description

__Requirements__:

* RGB camera or video
* OpenCV library (most likely this is already installed with ROS)
* Uniformly coloured ball

__Language__: Python or C++

__Inputs Topics__:

* `∼/event in`: `'e_start'` and `'e_stop'` to start and stop the node
* `∼/input image`: RGB image from camera or video

__Outputs Topics__:

* `∼/event out`: ”e found ball” and ”e no ball” to indicate presence or absence of a ball
* `∼/position`: Horizontal position of the ball expressed as a string ”left” or ”right” depending on if the centre of the ball is on the left or right of the image

## 2. Image Processing
Following steps are observed:

* Convert from ROS's `sensor_msgs/Image` to OpenCV's 'Mat' object
* Convert image's color from BGR to HSV type
* Filter out red color, creating a binary image with white being the red ball
* Perform open and close morphology operations on the binary image to eliminate noises and big holes
* Calculate the "center of mass" of the white object to get the ball position

## 3. State Machine
The node have 3 states:

* `'INIT'`: entry of program; waits on `∼/event in` for `'e_start'` to go to `'PROC'` state, or `'e_trigger'` to go to `'TRIG'` state.
* `'PROC'`: process image; call to `process_image()` with the image message form `∼/input image`. If `'e_stop'` is received go to `'INIT'` state, otherwise keep looping in `'PROC'` state.
* `'TRIG'`: process image once then return to `'INIT'` state.

## 4. Testing with ROS's usb_cam Package
* Install package `ros-indigo-usb-cam` to convert webcam/usb camera input to a `sensor_msgs/Image` feed:

```
sudo apt-get install ros-indigo-usb-cam
```
* Create a launch file referencing instruction from [Pharos Testbed Wiki](http://pharos.ece.utexas.edu/wiki/index.php/How_to_Use_a_Webcam_in_ROS_with_the_usb_cam_Package "How to Use a Webcam in ROS with the usb_cam Package"), then add node `ball_detector_ros_node` and remap `~input_image` to `/usb_cam/image_raw`


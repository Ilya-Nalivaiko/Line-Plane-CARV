v4l2-ctl --list-devices

# probably need some & and && here
# also remember to source ros setup

source /Line-Plane-CARV/Examples/ROS/setup.bash


roscore


rosparam set /cv_camera/device_id 0 #num
rosrun cv_camera cv_camera_node


rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.bin realsense_d405_hd.yaml /camera/image_raw:=/cv_camera/image_raw

source /Line-Plane-CARV/Examples/ROS/ORB_SLAM2/build/devel/setup.bash

v4l2-ctl --list-devices

roscore & (sleep 1 && rosparam set /cv_camera/device_id 0) & (sleep 2 && rosrun cv_camera cv_camera_node)
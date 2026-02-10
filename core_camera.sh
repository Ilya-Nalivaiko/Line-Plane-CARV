v4l2-ctl --list-devices

#roscore & (sleep 1 && rosparam set /cv_camera/device_id 0) & (sleep 2 && rosrun cv_camera cv_camera_node)



roscore & (sleep 2 && roslaunch ros_cam_publishers mjpegcam.launch ip:=192.168.1.109)
#roscore & (sleep 2 && roslaunch ros_cam_publishers mp4cam.launch file:=/Line-Plane-CARV/test_data/cabinets1.mp4 loop:=true fps:=20)
#roscore & (sleep 2 && roslaunch ros_cam_publishers tumcam.launch dataset_dir:=/Line-Plane-CARV/test_data/rgbd_dataset_freiburg2_pioneer_slam time_scale:=0.05)

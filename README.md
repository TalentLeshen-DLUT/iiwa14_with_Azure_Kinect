ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.024 --ros-args --remap image:=/rgb/image_raw  #Calibrate the camera

ros2 launch azure_kinect_ros_driver driver.launch.py #open the Kinect

ros2 run rqt_image_view rqt_image_view #run the rqt

ros2 run cv_bridge image_converter.py #run the image_converter.py

ros2 run cv_bridge face_detector.py #run the face_detector.py

ros2 launch lbr_moveit_cpp hello_moveit.launch.py mode:=mock model:=iiwa14 #run the hello_moveit.launch.py

This project is based on https://github.com/lbr-stack/lbr_fri_ros2_stack and https://github.com/chenshuxiao/azure-kinect-driver-ros2-humble?tab=readme-ov-file

I modified the xacro model in lbr_description


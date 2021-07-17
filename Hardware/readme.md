## Notes

1. `cd catkin_ws`

2. `source devel/setup.bash` %in all terminator windows!

3. `ping 10.0.3.26` %Optional - only if you want to check that the communication with the robot is working

4. Open terminator
  4. Window 1 - `roscore`
    
5. Window 2 - Initialize Kortex driver
  `roslaunch kortex_driver kortex_driver.launch ip_address:=10.0.3.26 start_rviz:=false start_moveit:=false gripper:=robotiq_2f_85` %It only works with the gripper

6. Window 3 - Initialize IMU node
  `rosrun lpms_imu lpms_imu_node _sensor_model:="DEVICE_LPMS_U" _port:="A5014194"` 
  
7. Window 4 - Initialize Camera node
  `roslaunch ueye_cam rgb8.launch` %Change launch file if necessary (frame rate, pixel clock, resolution, etc)
  
To see if everything is working properly, run "rostopic list" and verify if all the available topics are there.
- Open MATLAB
- Check if the generate trajectories are according to the specs
- Check the MATLAB API before running experiments
- Start recording a rosbag of all the topics

8. Window 5 - `rosrun image_view image_view image:=/camera/image_raw` %Check if image stream is ok

9. Window 6 - `rosbag record -O /media/goncalopereira/DATA/test.bag /camera/image_raw /imu /my_gen3/joint_states`


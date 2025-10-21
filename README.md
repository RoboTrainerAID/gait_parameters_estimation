gait_parameters_estimation
==========================================

This repo is used to estimate gait parameters from various sensor data.  
There is a lot of legacy code in here that does not work with the current RoboTrainer stack.
Normal process is to first record an rosbag from an experiment and then afterwards estimate the gait_parameters from the bag file.
toe_detection from Pointcloud can take longer than realtime leading to skipped frames. Thats why it is done offline from a bag file.

## How to start
1. Record rosbag from experiment
   - Instructions for that are in the 'robotrainer_user_performance/robotrainer_study_automatic_assessment' README.md

2. Launch robot URDF and RViz for visualization of .bag file
      ```bash
      roslaunch gait_parameters_estimation rviz_with_urdf.launch
      rosbag play /data/FILE_NAME.bag
      ```
3. Run toe detection with Kalman filter from .bag file (creates new bag file with toe positions)
      ```bash
      roslaunch camera_lower_leg_tracking toe_detection_kalman_from_bag.launch
      # default saves the new bag to /home/docker/ros_ws/data/toe_positions.bag

      # For visualization with RViz
      roslaunch gait_parameters_estimation rviz_with_urdf.launch
      rosbag play /home/docker/ros_ws/data/toe_positions.bag /data/FILE_NAME.bag

      # For visualization with Plotjuggler
      rosrun plotjuggler plotjuggler
      # Load layout from src/camera_lower_leg_tracking/include/swaped_analysis_plotjuggler.xml
      ```
4. Run gait estimation from .bag file with toe positions (creates new bag file with gait parameters)
      ```bash
      roslaunch gait_parameters_estimation gait_estimation_from_bag.launch
      # default saves the new bag to /home/docker/ros_ws/data/toe_positions_gait_output.bag

      # For visualization with RViz
      roslaunch gait_parameters_estimation rviz_with_urdf.launch
      rosbag play /home/docker/ros_ws/data/toe_positions_gait_output.bag /data/FILE_NAME.bag

5. Automatically generate gait_data for all bags in a folder
      ```bash
      ./gait_parameters_estimation/iterate_folder_and_estimate_gait.bash /path/to/folder/with/bags
      # default saves the new bag to /home/docker/ros_ws/data/toe_positions_gait_output.bag
      ```

### Quickstart
```bash
# Also publish scenario data in rviz
roslaunch robotrainer_study_automatic_assessment rviz_scenario_and_data.launch

# Visualization
roslaunch gait_parameters_estimation rviz_with_urdf.launch

# Toe detection from bag
roslaunch camera_lower_leg_tracking toe_detection_kalman_from_bag.launch

# Gait estimation from bag
roslaunch gait_parameters_estimation gait_estimation_from_bag.launch

# (Finally)
rosbag play /path/to/your.bag
```

## Testing asynchronous gait estimation from bag files
```bash
roslaunch camera_lower_leg_tracking toe_detection_kalman_from_bag_node.launch
roslaunch gait_parameters_estimation gait_estimation_from_bag_node.launch
rostopic pub -1 /robotrainer_user_study_manager/study_status std_msgs/String "data: 'KATE_AA_U010_16_yellow_line_force_right_60-1'"

rosservice call /toe_detection_kalman_from_bag_node/process
rosservice call /gait_estimation_from_bag_node/process

# (Finally)
rosbag play /path/to/your.bag
```


## ROS Interface (from outdated/deprecated code that uses live ros topics)
### Node: gait_estimation_node.py
Class: EstimatorBase
- Input Speed: /base/fts_adaptive_force_controller/debug/velocity_output (geometry_msgs/TwistStamped)(50 Hz)
- Input Pose: /mobile_robot_pose (ipr_helpers/Pose2DStamped)(500 Hz)

Class: EstimatorLegs (Laserscanner)
- Input Leg: /leg_detection/people_msg_stamped (leg_tracker/PersonMsg)(24 Hz)

Class: EstimatorForce (Force-Torque Sensor)
- Input Force: /base/output_data (geometry_msgs/WrenchStamped)(200 Hz)

Class: EstimatorToe (Lower Depth Camera)
- Input Toe: /right_toe, /left_toe (geometry_msgs/PointStamped)(0.8 - 1.2 Hz)

Class: EstimatorShoulder (Upper Depth Camera)
- Input Shoulders: /human_body_detection/points

### Node: toe_detection_node.cpp
```bash
rosbag play raw_image_data_2025-04-10-18-28-41.bag --topics /lower_legs_camera/depth_registered/points

# in rviz choose base_link frame

roslaunch gait_parameters_estimation toe_detection.launch
# Was ist der Unterschied?
# Toe detection uses a simple algorithm to detect the toe position
# based on the point cloud data from the lower legs camera.
# It does not use a Kalman filter or any initialization process.
```

- Input PointCloud: /lower_legs_camera/depth_registered/points (sensor_msgs/PointCloud2)
- Output Toe: /right_toe, /left_toe (geometry_msgs/PointStamped)
  
### Node: feet_detection_node.cpp
```bash
# in rviz choose base_link frame

roslaunch gait_parameters_estimation feet_detection.launch
# Was ist der Unterschied?
# Feet detection uses kalman filter and an initialization process to detect heel, ankle, toes and leg axis
```

- Input PointCloud: /lower_legs_camera/depth_registered/points (sensor_msgs/PointCloud2)
- Output Toe: /camera_lower_leg_tracking/right_toe, /camera_lower_leg_tracking/left_toe (geometry_msgs/PointStamped)
- Output Heel: /camera_lower_leg_tracking/right_heel, /camera_lower_leg_tracking/left_heel (geometry_msgs/PointStamped)
- Output Ankle: /camera_lower_leg_tracking/right_ankle, /camera_lower_leg_tracking/left_ankle (geometry_msgs/PointStamped)
- Output Foot Axis: /camera_lower_leg_tracking/right_foot_axis, /camera_lower_leg_tracking/left_foot_axis (visualization_msgs/Marker)
- Output Foot: /camera_lower_leg_tracking/right_Foot, /camera_lower_leg_tracking/left_Foot (sensor_msgs/PointCloud2)
- Output Leg: /camera_lower_leg_tracking/right_Leg_icp, /camera_lower_leg_tracking/left_Leg_icp (sensor_msgs/PointCloud2)
- Output Foot Strip: /footStrip (sensor_msgs/PointCloud2)
- Service Reset: /camera_lower_leg_tracking/init_reset (std_srvs/Trigger)

### Node: leg_tracker

```bash
roslaunch gait_parameters_estimation leg_tracker_one_person.launch
```
- Input Laserscan: /base_laser_back/scan (sensor_msgs/LaserScan)
- Output Legs: /leg_detection/people_msg_stamped (leg_tracker/PersonMsg)
- Output Body Center: /leg_detection/body_center_stamped (geometry_msgs/PointStamped)


## Related Resources
- weighted Fourier linear combiner (WFLC)
- https://www.cs.cmu.edu/~micron/filtering.htm
- fs = sampling frequency
- dst = double support time (currently not used)

### For ROS Debugging
- Install the ROS vscode extension
- Downgrade the python extension to support Python 2.7/3.6
- Click on the arrow next to "uninstall", install specific version and select 2021.5.9...
- Then create a roslaunch task and run the debugger.

## TODO
- [x] Fix correct transform map -> base_link not convert to topic and back but directly form topic
- [x] Neues script
- [x] 1. rosbag einlesen (wie bei Yuliia ros2 (rosbags) oder Marie ros1 (rosbag_pandas))
- [x] 1.1 einlesen in cpp um pointcloud conversion mit lower legs camera zu machen -> ouput ist toe_positions als .bag oder csv file
- [ ] 1.2 Die selben toe_positions daten schätzen aus Laserscanner mit ros_people package to detect legs
- [ ] 1.3 Welche Werte sind genauer/zuverlässiger?
- [ ] 1.4 Daten glätten und zusammenführen aus verschiedenen quellen mit kalman filter
- [ ] 2. Funktion machen um alle Daten (gait usw) aus einem experiment herausfinden
- [x] 3. Datenglättung und schätzung mit kalman filters
- [x] 4. Gait parameter berechnung
- [ ] Improved Kalman Filter
  - [x] new KalmanFilter(target_Frequency)
  - [ ] dt wird sowieso ingoriert. Sollte man in template rausnehmen
  - [x] eine einzige Funktion in der man den neusten Messwert eingibt und zurück kommen alle predictions bis dort hin sowie die aktuelle Korrektur mit den neuesten realen Messwert.
  - [ ] Rausziehen in eigenes Package, improvement von iirob_filters. Evtl ros_node draus machen
  - [x] reduce to 6 dimensions and leave out z
  - [x] https://soulhackerslabs.com/recursive-state-estimation-with-kalman-filters-and-ros-2-b869d3775357
  - [x] https://github.com/pcdangio/ros-kalman_filter
  - [x] https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/include/robot_localization/ukf.hpp
  - [x] Maybe one of the best ccp packages: https://github.com/FrancoisCarouge/Kalman
  - [x] Tune Your Existing Filter (Easiest):
         Increase Process Noise Q: This is the most common solution. A larger Q makes the filter trust its model less and follow the measurements more closely. The trade-off is that if Q is too high, the output becomes less smooth and more like the noisy raw data.
  - [ ] Use a Higher-Order Linear Model:
         You can use a "Constant Jerk" model. Jerk is the rate of change of acceleration. You would add jerk (jx, jy, jz) to your state vector, making it 12-dimensional. This model uses a third-order polynomial to approximate the motion, which can follow a curve better than a parabola. It's still a linear model and works with a standard Kalman Filter.
   - [ ] Use an Extended Kalman Filter (EKF):
         If you want to stick with a second-order model but allow for non-linear motion, you can use an EKF. This involves defining a non-linear state transition function that can better capture the dynamics of your system. The EKF linearizes this function around the current estimate at each time step.
   - [ ] Use an Unscented Kalman Filter (UKF):
         The UKF is another option for non-linear systems. It uses a deterministic sampling approach to capture the mean and covariance of the state distribution more accurately than the EKF. This can be beneficial if your system exhibits significant non-linearities.
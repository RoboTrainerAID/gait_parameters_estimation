#!/usr/bin/env python

import rospy
import rosbag
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, PointStamped, TransformStamped, Vector3, Quaternion
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np


class EstimatorToeFromBag():

    def __init__(self):

        bag_file_path = '/home/docker/ros_ws/data/toe_positions.bag'

        topics = [
            '/tf',
            '/tf_static',
            '/toe_position/left/kalman',
            '/toe_position/right/kalman',
        ]

        self.tf_buffer, messages = self._read_data_from_bag(bag_file_path, topics)

        toe_data_left = messages["/toe_position/left/kalman"] # type: Dict[rospy.Time, PointStamped]
        toe_data_right = messages["/toe_position/right/kalman"] # type: Dict[rospy.Time, PointStamped]

        # --- Analyze Timestamps for each topic ---
        # self._analyze_and_plot_timestamps(toe_data_left, "Left Toe Kalman")
        # self._analyze_and_plot_timestamps(toe_data_right, "Right Toe Kalman")
        # -----------------------------------------

        toe_data_left_in_map_frame = self._transform_to_map_frame(toe_data_left)
        toe_data_right_in_map_frame = self._transform_to_map_frame(toe_data_right)

        # --- Plot transformed data series ---
        self._plot_data_series(toe_data_left_in_map_frame, "Left Toe Position in Map Frame", labels_to_plot=['x', 'y', 'dist'])
        self._plot_data_series(toe_data_right_in_map_frame, "Right Toe Position in Map Frame", labels_to_plot=['x', 'y', 'dist'])
        # ------------------------------------

        self._synchronized_toe_data = self._synchronize_toe_data(toe_data_left_in_map_frame, toe_data_right_in_map_frame, slop=0.001)

        # self.toe_data = TimeSeriesData(self._fs, self._window_size, self._window_step)

        # self._toe_lock = Lock()
        # self._sub_toes = rospy.Subscriber("/toe_detection/toe_positions", PoseArray, self.listen_toes)

        # self._data_synced = False

    def _synchronize_toe_data(self, left_data, right_data, slop):
        """
        Synchronizes two dictionaries of messages based on their header timestamps
        using an approximate time policy.

        Args:
            left_data (Dict[rospy.Time, PointStamped]): Messages for the left toe.
            right_data (Dict[rospy.Time, PointStamped]): Messages for the right toe.
            slop (float): The maximum time difference (in seconds) allowed for a match.

        Returns:
            List of synchronized (left_msg, right_msg) tuples.
        """
        # Get sorted lists of timestamps
        left_stamps = sorted(left_data.keys())
        right_stamps = sorted(right_data.keys())

        synchronized_pairs = []
        l_idx, r_idx = 0, 0
        used_r_indices = set()
        slop_duration = rospy.Duration.from_sec(slop)

        while l_idx < len(left_stamps) and r_idx < len(right_stamps):
            l_stamp = left_stamps[l_idx]
            best_match_r_stamp = None
            min_diff = slop_duration

            # Search for the best match in the right list
            temp_r_idx = r_idx
            while temp_r_idx < len(right_stamps):
                r_stamp = right_stamps[temp_r_idx]
                diff = abs(l_stamp - r_stamp)

                if diff <= min_diff:
                    min_diff = diff
                    best_match_r_stamp = r_stamp
                    # Tentatively update the starting point for the next search
                    search_start_r_idx = temp_r_idx
                
                if r_stamp - l_stamp > slop_duration:
                    break # Right stamp is too far ahead, move to next left stamp
                
                temp_r_idx += 1

            if best_match_r_stamp and search_start_r_idx not in used_r_indices:
                l_msg = left_data[l_stamp]
                r_msg = right_data[best_match_r_stamp]
                synchronized_pairs.append((l_msg, r_msg))
                used_r_indices.add(search_start_r_idx)
                r_idx = search_start_r_idx + 1 # Start next search from the next right message
            
            l_idx += 1

        unpaired_count = len(left_data) + len(right_data) - 2 * len(synchronized_pairs)
        rospy.loginfo("Successfully synchronized %d message pairs." % len(synchronized_pairs))
        rospy.loginfo("%d messages could not be paired and were discarded." % unpaired_count)

        return synchronized_pairs
    
    def _read_data_from_bag(self, bag_path, topics_to_read):
        """
        Reads specified topics from a rosbag file.
        It populates a tf2_ros.Buffer with TF messages and collects all other
        messages into a dictionary mapping timestamps to messages.
        Args:
            bag_path (str): The path to the rosbag file.
            topics_to_read (list): A list of topic names to read.
        Returns:
            tuple(tf2_ros.Buffer, dict): A tuple containing:
                - The buffer populated with transforms from the bag.
                - A dictionary where keys are topic names and values are dictionaries of {timestamp: message}.
        """
        rospy.loginfo("Reading data from bag: %s", bag_path)
        
        cache_duration = 3600  # 1 hour
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_duration))
        
        # Initialize a dictionary to hold dictionaries of {timestamp: message} for each topic
        populated_messages = {topic: {} for topic in topics_to_read}

        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                # Check if the TF buffer cache is long enough for the entire bag
                bag_duration_secs = bag.get_end_time() - bag.get_start_time()
                buffer_cache_secs = cache_duration
                if bag_duration_secs > buffer_cache_secs:
                    rospy.logwarn("Bag duration (%.2f s) is greater than TF buffer cache time (%.2f s).", 
                                  bag_duration_secs, buffer_cache_secs)
                    rospy.logwarn("This may lead to loss of old transforms and cause lookup errors. "
                                  "Consider increasing the 'cache_duration' for the tf2_ros.Buffer.")

                # Iterate over all specified topics
                for topic, msg, t in bag.read_messages(topics=topics_to_read):
                    # Handle TF messages to populate the buffer
                    if topic in ['/tf', '/tf_static']:
                        for transform in msg.transforms:
                            is_static = (topic == '/tf_static')
                            try:
                                if is_static:
                                    tf_buffer.set_transform_static(transform, "bag_authority")
                                else:
                                    tf_buffer.set_transform(transform, "bag_authority")
                            except tf2_ros.TransformException as ex:
                                rospy.logwarn("Failed to set transform: %s", ex)
                    
                    # For all other topics, add the message to the dictionary with its timestamp as the key
                    elif topic in populated_messages:
                        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                            populated_messages[topic][msg.header.stamp] = msg
                        else:
                            rospy.logwarn("Message on topic '%s' has no header/stamp, cannot add to dictionary.", topic)

        except rosbag.ROSBagException as e:
            rospy.logerr("Error opening or reading bag file: %s", e)
            return None, None
        
        # Clean up the dictionary by removing the TF topics that were handled separately
        populated_messages.pop('/tf', None)
        populated_messages.pop('/tf_static', None)

        rospy.loginfo("Bag data reading complete:")
        for topic in populated_messages:
            rospy.loginfo("Topic: {}, Number of messages: {}".format(topic, len(populated_messages[topic])))

        return tf_buffer, populated_messages
    
    def _analyze_and_plot_timestamps(self, data_dict, topic_name):
        """
        Analyzes and plots the distribution of timestamps from a dictionary of messages.
        Args:
            data_dict (Dict[rospy.Time, PointStamped]): A dictionary of ROS messages.
            topic_name (str): A descriptive name for logging and plotting.
        """
        if not data_dict:
            rospy.logwarn("No data found for topic '%s' to analyze.", topic_name)
            return

        timestamps = [ts.to_sec() for ts in data_dict.keys()]

        earliest_time = min(timestamps)
        latest_time = max(timestamps)

        rospy.loginfo("--- Timestamp Analysis for '%s' ---", topic_name)
        rospy.loginfo("Earliest timestamp: %.4f (%s)", earliest_time, 
                  rospy.Time.from_sec(earliest_time).to_sec().__str__() if hasattr(rospy.Time, 'to_sec') else "")
        rospy.loginfo("Latest timestamp:   %.4f (%s)", latest_time, 
                  rospy.Time.from_sec(latest_time).to_sec().__str__() if hasattr(rospy.Time, 'to_sec') else "")
        rospy.loginfo("Total duration:     %.4f seconds", latest_time - earliest_time)

        # Also print ISO format using datetime for clarity
        rospy.loginfo("Earliest ISO: %s", datetime.utcfromtimestamp(earliest_time).isoformat())
        rospy.loginfo("Latest ISO:   %s", datetime.utcfromtimestamp(latest_time).isoformat())

        # Plot the distribution
        plt.figure(figsize=(12, 6))
        relative_timestamps = [ts - earliest_time for ts in timestamps]
        plt.hist(relative_timestamps, bins=100, color='royalblue', alpha=0.8)
        plt.title('Distribution of Timestamps for %s' % topic_name)
        plt.xlabel('Time (seconds from first message)')
        plt.ylabel('Number of Messages per Bin')
        plt.grid(True, linestyle='--', alpha=0.6)
        rospy.loginfo("Displaying timestamp distribution plot for '%s'...", topic_name)
        plt.show()

    def _plot_data_series(self, data_dict, title, labels_to_plot=['x', 'y', 'z']):
        """
        Plots a time series of 3D point data from a dictionary of PointStamped messages.

        Args:
            data_dict (Dict[rospy.Time, PointStamped]): A dictionary of PointStamped messages.
            title (str): The title for the plot.
            labels_to_plot (list): A list of strings specifying which components to plot.
                                   Options are 'x', 'y', 'z', and 'dist' (for sqrt(x^2+y^2)).
        """
        if not data_dict:
            rospy.logwarn("No data provided for plotting '%s'.", title)
            return

        # Sort by timestamp to ensure correct plotting order
        sorted_stamps = sorted(data_dict.keys())
        data_series = [data_dict[ts] for ts in sorted_stamps]

        timestamps = [ts.to_sec() for ts in sorted_stamps]
        
        # Make timestamps relative for a cleaner x-axis
        start_time = timestamps[0]
        relative_timestamps = [ts - start_time for ts in timestamps]

        x_vals = np.array([msg.point.x for msg in data_series])
        y_vals = np.array([msg.point.y for msg in data_series])
        z_vals = np.array([msg.point.z for msg in data_series])

        plt.figure(figsize=(15, 7))

        if 'x' in labels_to_plot:
            plt.plot(relative_timestamps, x_vals, label='x')
        if 'y' in labels_to_plot:
            plt.plot(relative_timestamps, y_vals, label='y')
        if 'z' in labels_to_plot:
            plt.plot(relative_timestamps, z_vals, label='z')
        if 'dist' in labels_to_plot:
            dist_vals = -np.sqrt(x_vals**2 + y_vals**2 + z_vals**2)
            plt.plot(relative_timestamps, dist_vals, label='dist', linestyle='--')

        plt.title(title)
        plt.xlabel("Time (seconds from start)")
        plt.ylabel("Position / Distance")
        plt.legend()
        plt.grid(True)
        rospy.loginfo("Displaying plot: '%s'", title)
        plt.show()

    def _transform_to_map_frame(self, toe_data_dict):
        """
        Transforms a dictionary of PointStamped messages to the target frame.

        Args:
            toe_data_dict (Dict[rospy.Time, PointStamped]): A dictionary of points to transform.

        Returns:
            Dict[rospy.Time, PointStamped]: A new dictionary of transformed points.
        """
        transformed_points = {}
        if not toe_data_dict:
            return {}

        source_frame = next(iter(toe_data_dict.values())).header.frame_id

        for stamp, point_stamped in toe_data_dict.items():
            try:
                # Look up the transform at the specific time of the message
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    source_frame,
                    stamp,
                    rospy.Duration(0.1)  # Timeout for the lookup
                )
                
                # Apply the transform
                point_in_map = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                transformed_points[point_in_map.header.stamp] = point_in_map

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Could not transform point at time %s: %s", str(stamp), e)
        
        rospy.loginfo("Transformed %d out of %d points to '%s' frame.", len(transformed_points), len(toe_data_dict), 'map')
        return transformed_points


    # estimate gait parameters from toe positions
    def gait_estimation(self, timer_event):

        start_time = rospy.Time.now().to_sec()

        # print("TOE DATA")
        # print("Length toe data: ", len(self.toe_data.data))
        window_toe = self.toe_data.get_window_by_fs()
        # print("VEL DATA")
        # print("Length vel data: ", len(self.velocity_data.data))
        window_vel = self.velocity_data.get_window_by_fs()
        # print("POSE DATA")
        # print("Length pose data: ", len(self.pose_data.data))
        window_poses = self.pose_data.get_window_by_fs()
        self._window_poses = window_poses  # for campatibility with EstimatorBase

        if window_toe is None or window_poses is None or window_vel is None:
            rospy.logwarn("No valid data in window, skipping gait estimation.")
            self._data_synced = False
            return

        self._data_synced = self.sync_data()

        avg_window_vel = self.get_avg_speed(window_vel)

        toe_fs = self.toe_data.est_fs(window_toe, len(window_toe))
        print("Toe fs: ", toe_fs)
        print("self.toe_data.fs: ", self.toe_data.fs)

        print(avg_window_vel)

        time_stamps = []
        left_sumsq = []
        right_sumsq = []

        for toe_msg in window_toe:
            time_stamps.append(toe_msg.header.stamp.to_sec())
            left_sumsq.append(toe_msg.poses[0].position.x ** 2 + toe_msg.poses[0].position.y ** 2)
            right_sumsq.append(toe_msg.poses[1].position.x ** 2 + toe_msg.poses[1].position.y ** 2)

        left_toe_norm = np.sqrt(left_sumsq)
        right_toe_norm = np.sqrt(right_sumsq)

        params = gp()
        params.header.stamp = window_toe[-1].header.stamp

        debug_plt = []
        plt_ind = []
        debugs = []

        # #check if robot moved by velocity, if under threshold set cadence to zero
        # if all(s < 0.2 for s in avg_window_vel):
        #     params.cadence = 0.0
        #     params.cadence_avg = 0.0
        # else:
        #     cad, cad_avg, debugs = self._wflc.wflc(td_bandpassed, self._fs, t = time_stamps)
        #     params.cadence = cad
        #     params.cadence_avg = cad_avg

        # print("cadence: ", params.cadence)
        # print("cadence_avg: ", params.cadence_avg)

        leg1_param = leg_params()
        leg2_param = leg_params()

        min_peak_dist = toe_fs * (1.0 / self._highcut) * 2
        pkwidth = toe_fs * (0.1 / self._highcut) if toe_fs * (0.1 / self._highcut) > 1.0 else 1.0

        print("min_peak_dist: ", min_peak_dist)
        print("pkwidth: ", pkwidth)

        if min_peak_dist < 1.0:
            # signal.find_peaks requires a minimum distance of 1.0
            rospy.logerr("min_peak_dist is < 1, return estimator_toe method")
            return

        peak_indexes_l1, _ = signal.find_peaks(left_toe_norm, distance=min_peak_dist, prominence=0.025, width=pkwidth)
        valley_indexes_l1, _ = signal.find_peaks(-left_toe_norm, distance=min_peak_dist, prominence=0.025, width=pkwidth)
        peak_indexes_l2, _ = signal.find_peaks(right_toe_norm, distance=min_peak_dist, prominence=0.025, width=pkwidth)
        valley_indexes_l2, _ = signal.find_peaks(-right_toe_norm, distance=min_peak_dist, prominence=0.025, width=pkwidth)

        # only for debugging
        # self.visualize(time_stamps, left_toe_norm, right_toe_norm, peak_indexes_l1, valley_indexes_l1, peak_indexes_l2, valley_indexes_l2)

        if avg_window_vel[0] == max(avg_window_vel):
            if avg_window_vel[0] >= 0:
                to_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])
                hs_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])

                to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
                hs_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2])
            else:
                to_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])
                hs_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])

                to_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2])
                hs_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
        else:
            if avg_window_vel[1] >= 0:
                to_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])
                hs_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])

                to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
                hs_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2])
            else:
                to_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])
                hs_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])

                to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
                hs_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2])

        # pose_at_heelstrike_left = self.get_mobile_robot_pose_at_time(window_poses, valley_indexes_l1)
        # pose_at_heelstrike_right = self.get_mobile_robot_pose_at_time(window_poses, valley_indexes_l2)

        # get mobile robot pose at the timestamp of the heel strikes
        # def get_mobile_robot_pose_at_time(self, poses, indexes):
        # type: (NDArray[float], List[int]) -> List

        # get toe data at valley indixes
        toe_at_heelstrike_left = window_toe[peak_indexes_l1]
        toe_at_heelstrike_right = window_toe[peak_indexes_l2]

        left_heelstrike_in_map_frame = self.calculate_toe_in_map_frame(window_poses, toe_at_heelstrike_left, left=True)
        right_heelstrike_in_map_frame = self.calculate_toe_in_map_frame(window_poses, toe_at_heelstrike_right, left=False)

        if not left_heelstrike_in_map_frame is None:
            print("Number of left heelstrikes in window: ", len(left_heelstrike_in_map_frame))
            for toe in left_heelstrike_in_map_frame:
                left_toe_heelstrike_in_map.publish(toe)

        if not right_heelstrike_in_map_frame is None:
            print("Number of right heelstrikes in window: ", len(right_heelstrike_in_map_frame))
            for toe in right_heelstrike_in_map_frame:
                right_toe_heelstrike_in_map.publish(toe)

        # TODO: (Andreas) calculate step length as distance between heel strikes of left and right toe
        # 1. Get main movement axis between last two heelstrikes of one foot
        # 2. Project point of other foot heelstrike onto that axis
        # 2. calculate distance of one heelstrike to heelstrike of other foot only along this axis

        leg1_param.cadence = self.cadence_leg(time_stamps, hs_l1_t, to_l1_t)
        leg2_param.cadence = self.cadence_leg(time_stamps, hs_l2_t, to_l2_t)

        hs_l1_ind = [time_stamps.index(hs1) for hs1 in hs_l1_t]
        hs_l2_ind = [time_stamps.index(hs2) for hs2 in hs_l2_t]
        to_l1_ind = [time_stamps.index(to1) for to1 in to_l1_t]
        to_l2_ind = [time_stamps.index(to2) for to2 in to_l2_t]

        leg1_param.stride_length, leg1_param.swing_time = self.stride_length(time_stamps, left_toe_norm, hs_l1_ind, to_l1_ind)
        leg2_param.stride_length, leg2_param.swing_time = self.stride_length(time_stamps, right_toe_norm, hs_l2_ind, to_l2_ind)

        leg1_param.stride_intervall = self.stride_time(to_l1_t)
        leg2_param.stride_intervall = self.stride_time(to_l2_t)

        if leg1_param.stride_intervall != 0.0:
            leg1_param.cadence = 1.0 / leg1_param.stride_intervall
        if leg2_param.stride_intervall != 0.0:
            leg2_param.cadence = 1.0 / leg2_param.stride_intervall

        leg1_param.stance_time = self.stance_duration(hs_l1_t, to_l1_t)
        leg2_param.stance_time = self.stance_duration(hs_l2_t, to_l2_t)

        leg1_param.step_length, leg2_param.step_length = self.step_length(time_stamps, left_toe_norm, right_toe_norm)
        params.leg1 = leg1_param
        params.leg2 = leg2_param

        ret_dict = {}
        ret_dict['/gait/toe_params'] = params
        ret_dict['/gait/toe_band'] = debugs

        # self.publish_params(ret_dict)
        print("Publishing gait parameters for toe")
        end_time = rospy.Time.now().to_sec()
        rospy.loginfo("Gait estimation took %.2f seconds", end_time - start_time)

        return ret_dict

    # Plot the three time series together
    def visualize(self, time_stamps, left_toe_norm, right_toe_norm, peak_indexes_l1, valley_indexes_l1, peak_indexes_l2, valley_indexes_l2):
        plt.ion()
        plt.figure(figsize=(8, 5))
        plt.plot(time_stamps, left_toe_norm, label='Left Toe RMS', alpha=0.7)
        plt.plot(time_stamps, right_toe_norm, label='Right Toe RMS', alpha=0.7)

        # Add peaks and valleys as dots
        plt.scatter([time_stamps[i] for i in peak_indexes_l1], [left_toe_norm[i]
                    for i in peak_indexes_l1], color='red', label='Left Toe Peaks', zorder=5)
        plt.scatter([time_stamps[i] for i in valley_indexes_l1], [left_toe_norm[i]
                    for i in valley_indexes_l1], color='blue', label='Left Toe Valleys', zorder=5)
        plt.scatter([time_stamps[i] for i in peak_indexes_l2], [right_toe_norm[i]
                    for i in peak_indexes_l2], color='green', label='Right Toe Peaks', zorder=5)
        plt.scatter([time_stamps[i] for i in valley_indexes_l2], [right_toe_norm[i]
                    for i in valley_indexes_l2], color='orange', label='Right Toe Valleys', zorder=5)

        plt.xlabel('Time (s)')
        plt.ylabel('Distance')
        plt.title('Toe RMS and Toe Difference')
        plt.legend()
        plt.grid(True)
        # plt.show()
        plt.draw()
        plt.pause(0.001)

    def calculate_toe_in_map_frame(self, window_poses, window_toe, left):
        if not self._data_synced or len(window_poses) == 0 or len(window_toe) == 0:
            return None

        pose_timestamps = [pose.header.stamp.to_sec() - self.pose_data.timestamp_offset for pose in window_poses]

        pose_at_toe_point = window_poses[[self.closest_node((hs.header.stamp.to_sec() - self.toe_data.timestamp_offset), pose_timestamps)
                                          for hs in window_toe]]

        toe_in_map_frame = []

        for pose_at_toe_point, toe_point in zip(pose_at_toe_point, window_toe):

            point = PointStamped()
            point.header = toe_point.header
            if left:
                point.point = toe_point.poses[0].position
            else:
                point.point = toe_point.poses[1].position

            q = tf.transformations.quaternion_about_axis(pose_at_toe_point.pose.theta, (0, 0, 1))

            t = TransformStamped()
            t.header.stamp = pose_at_toe_point.header.stamp
            t.header.frame_id = pose_at_toe_point.header.frame_id  # Should be 'map' frame
            t.child_frame_id = toe_point.header.frame_id  # Should be 'base_link' frame
            t.transform.translation.x = pose_at_toe_point.pose.x
            t.transform.translation.y = pose_at_toe_point.pose.y
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            toe_in_map_frame.append(tf2_geometry_msgs.do_transform_point(point, t))

            # print("Toe position in map frame: ", left_point)

        return toe_in_map_frame

    def step_length(self, time_stamps, leg1, leg2):

        # TODO: (Andreas) calculate step length
        # 1. Transform positions of heel strike to map frame with mobile_robot_pose
        # 2. Calculate distance between heel strike of one foot to heel strike of other foot
        # 3. Publishi visualization to confirm locations on the map with camera data

        min_peak_dist = self.toe_data.fs * (1.0 / self._highcut) * 2

        if min_peak_dist < 1.0:
            # signal.find_peaks requires a minimum distance of 1.0
            rospy.logerr("min_peak_dist is < 1, return estimator_toe method")
            return 0.0, 0.0

        # since we're using RMS (absolute distance) peak = TO (maximum distance), valley = HS (minimum distance)
        min_peak_dist = self.toe_data.fs * (1.0 / self._highcut) * 2
        pkwidth = self.toe_data.fs * (0.1 / self._highcut) if self.toe_data.fs * (0.1 / self._highcut) > 1.0 else 1.0
        peak_indexes_l1, _ = signal.find_peaks(leg1, distance=min_peak_dist, prominence=0.025, width=pkwidth)
        valley_indexes_l1, _ = signal.find_peaks(-leg1, distance=min_peak_dist, prominence=0.025, width=pkwidth)
        peak_indexes_l2, _ = signal.find_peaks(leg2, distance=min_peak_dist, prominence=0.025, width=pkwidth)
        valley_indexes_l2, _ = signal.find_peaks(-leg2, distance=min_peak_dist, prominence=0.025, width=pkwidth)

        # TODO: (Andreas) unused
        # to_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])
        # hs_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])

        # to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
        # hs_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2])

        xc, yc = prep.interpolated_intercept(np.array(time_stamps), leg1, leg2)

        # plt.figure(figsize=(8, 5))
        # plt.plot(time_stamps, leg1, label='Leg1 (Left Toe)', alpha=0.7)
        # plt.plot(time_stamps, leg2, label='Leg2 (Right Toe)', alpha=0.7)

        # # Add peaks and valleys
        # plt.scatter([time_stamps[i] for i in peak_indexes_l1], [leg1[i] for i in peak_indexes_l1],
        #             color='red', label='Leg1 Peaks', zorder=5)
        # plt.scatter([time_stamps[i] for i in valley_indexes_l1], [leg1[i] for i in valley_indexes_l1],
        #             color='blue', label='Leg1 Valleys', zorder=5)
        # plt.scatter([time_stamps[i] for i in peak_indexes_l2], [leg2[i] for i in peak_indexes_l2],
        #             color='green', label='Leg2 Peaks', zorder=5)
        # plt.scatter([time_stamps[i] for i in valley_indexes_l2], [leg2[i] for i in valley_indexes_l2],
        #             color='orange', label='Leg2 Valleys', zorder=5)

        # # Visualize interpolated intercept
        # plt.scatter(xc, yc, color='black', label='Interpolated Intercept', marker='x', zorder=6)

        # plt.xlabel('Time (s)')
        # plt.ylabel('Distance')
        # plt.title('Leg1 and Leg2 Timeseries with Peaks, Valleys, and Intercept')
        # plt.legend()
        # plt.grid(True)
        # plt.show()

        xc_ind = []

        for x in xc:
            if x in time_stamps:
                xc_ind.append(time_stamps.index(x))
            else:
                ind = next((t for t in time_stamps if t > x), None)
                if ind:
                    xc_ind.append(time_stamps.index(ind))

        l1steps = []
        l2steps = []

        # rotation Hs = valley; negative x :HS = Peak; negative y HS:valley
        l1_hs_ind = valley_indexes_l1
        l2_hs_ind = valley_indexes_l2

        for i in range(len(l1_hs_ind)):
            if any(x > l1_hs_ind[i] for x in xc_ind):
                x1 = next((x for x in xc_ind if x > l1_hs_ind[i]), None)
                if x1 and ((i == len(l1_hs_ind) - 1) or (x1 < l1_hs_ind[i + 1])):
                    l1steps.append(leg1[l1_hs_ind[i]] - leg1[x1])
        for i in range(len(l2_hs_ind)):
            if any(x > l2_hs_ind[i] for x in xc_ind):
                x2 = next((x for x in xc_ind if x > l2_hs_ind[i]), None)
                if x2 and ((i == len(l2_hs_ind) - 1) or (x2 < l2_hs_ind[i + 1])):
                    l2steps.append(leg2[l2_hs_ind[i]] - leg2[x2])

        l1_step = 0.0
        l2_step = 0.0
        if l1steps:
            l1_step = sum(l1steps) / len(l1steps)
            # rospy.loginfo("!!!! ::::::::: >>>>>STEP LENGTH L1 %.8f", sum(l1steps) / len(l1steps) )
        if l2steps:
            l2_step = sum(l2steps) / len(l2steps)
            # rospy.loginfo("!!!! ::::::::: >>>>>STEP LENGTH L2 %.8f", sum(l2steps) / len(l2steps) )

        return l1_step, l2_step


if __name__ == '__main__':


    rospy.init_node('tf_bag_reader')

    EstimatorToeFromBag()

    while not rospy.is_shutdown():
        rospy.spin()



    # rospy.init_node('gait_estimation', log_level=rospy.INFO)
    # rospy.get_rostime()
    # rospy.get_time()

    # # Retrieve bool parameters (with default = True)
    # use_force = rospy.get_param('/gait_estimation/use_force', True)
    # use_leg = rospy.get_param('/gait_estimation/use_leg', True)
    # use_toe = rospy.get_param('/gait_estimation/use_toe', True)
    # use_shoulder = rospy.get_param('/gait_estimation/use_shoulder', True)

    # window_size = rospy.get_param('/gait_estimation/window_size', 7.0)
    # window_step = rospy.get_param('/gait_estimation/window_step', 1.0)
    # estimators = {}
    # pub_dicts = {}

    # force_pub = rospy.Publisher('/gait/force_params', gp, tcp_nodelay=True, queue_size=1024)
    # legs_pub = rospy.Publisher('/gait/leg_params', gp, tcp_nodelay=True, queue_size=1024)
    # toe_pub = rospy.Publisher('/gait/toe_params', gp, tcp_nodelay=True, queue_size=1024)
    # shoulder_pub = rospy.Publisher('/gait/shoulder_params', gp, tcp_nodelay=True, queue_size=1024)
    # left_toe_heelstrike_in_map = rospy.Publisher('/gait/left_toe_heelstrike_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)
    # right_toe_heelstrike_in_map = rospy.Publisher('/gait/right_toe_heelstrike_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)
    # left_toe_toe_off_in_map = rospy.Publisher('/gait/left_toe_toe_off_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)
    # right_toe_toe_off_in_map = rospy.Publisher('/gait/right_toe_toe_off_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)

    # rospy.sleep(rospy.Duration(0.25))
    # # est_force = EstimatorForce()
    # # est_leg = EstimatorLegs()
    # est_toe = EstimatorToeFromBag()
    # # est_sh = EstimatorShoulder()

    # # Add each estimator to the dictionary only if its parameter is true
    # # if use_force:
    # #     estimators['force'] = est_force
    # # if use_leg:
    # #     estimators['legs'] = est_leg
    # if use_toe:
    #     estimators['toe'] = est_toe
    # # if use_shoulder:
    # #     estimators['shoulder'] = est_sh

    # rospy.loginfo("Start collecting data")
    # rospy.sleep(rospy.Duration(window_size))
    # rospy.loginfo("Start timed thread")
    # rospy.timer.Timer(rospy.Duration(window_step), est_toe.gait_estimation)

    # while not rospy.is_shutdown():
    #     rospy.spin()

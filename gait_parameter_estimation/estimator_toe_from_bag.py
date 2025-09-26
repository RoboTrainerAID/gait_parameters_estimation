#!/usr/bin/env python

import rospy
import rosbag
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import numpy as np
from scipy import signal
from std_msgs.msg import Float32, Int32

import matplotlib.pyplot as plt


class EstimatorToeFromBag():

    def __init__(self, bag_file_path, topics):

        self.tf_buffer, messages = self._read_data_from_bag(bag_file_path, topics)

        # --- Extract msgs from bag ---
        toe_left_msg = messages["/toe_position/left/kalman"] # type: Dict[float, PointStamped]
        toe_right_msg = messages["/toe_position/right/kalman"] # type: Dict[float, PointStamped]

        # --- Filter the data to remove jitter ---
        toe_left_msg_filtered = self._smooth_and_filter_data(toe_left_msg, cutoff=2.0, order=4)
        toe_right_msg_filtered = self._smooth_and_filter_data(toe_right_msg, cutoff=2.0, order=4)

        # --- Transform to 'map' frame and store as instance variables ---
        self.toe_left_msg_map_frame = self._transform_to_map_frame(toe_left_msg_filtered)
        self.toe_right_msg_map_frame = self._transform_to_map_frame(toe_right_msg_filtered)
        self.synchronized_toe_data = self._synchronize_toe_data(self.toe_left_msg_map_frame, self.toe_right_msg_map_frame, slop=0.001)

        # --- Calculate normalized toe distance and store ---
        self.left_t, self.left_dist = self._get_normalized_distance(toe_left_msg_filtered)
        self.right_t, self.right_dist = self._get_normalized_distance(toe_right_msg_filtered)

        # --- Store original data for plotting comparison ---
        self.original_left_t, self.original_left_dist = self._get_normalized_distance(toe_left_msg)
        self.original_right_t, self.original_right_dist = self._get_normalized_distance(toe_right_msg)

    def plot_data(self, plot_lines={}, plot_points={}):
        """
        Plots an arbitrary number of data series on a single graph using timestamps.

        Args:
            plot_lines (dict): A dictionary where keys are labels and values are (timestamps, data) tuples to be plotted as lines.
            plot_points (dict): A dictionary where keys are labels and values are (timestamps, data) tuples to be plotted as scatter points.
        """
        plt.figure(figsize=(15, 7))

        # Plot all line series
        for label, (timestamps, data) in plot_lines.items():
            if data.size > 0 and timestamps.size == data.size:
                plt.plot(timestamps, data, label=label, alpha=0.8)
            else:
                rospy.logwarn("Skipping plot for line '%s': data/timestamp size mismatch or empty.", label)

        # Plot all point series
        for label, (timestamps, data) in plot_points.items():
            if data.size > 0 and timestamps.size == data.size:
                plt.scatter(timestamps, data, label=label, s=10) # s for marker size
            else:
                rospy.logwarn("Skipping plot for points '%s': data/timestamp size mismatch or empty.", label)
                print('sizes:', timestamps.size, data.size)

        plt.title("Data Series Comparison")
        plt.xlabel("Time (seconds)")
        plt.ylabel("Value (e.g., Distance)")
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.6)
        rospy.loginfo("Displaying combined data plot...")
        plt.show()

    def _synchronize_toe_data(self, left_data, right_data, slop):
        """
        Synchronizes two dictionaries of messages based on their header timestamps
        using an approximate time policy.

        Args:
            left_data (Dict[float, PointStamped]): Messages for the left toe with float timestamps.
            right_data (Dict[float, PointStamped]): Messages for the right toe with float timestamps.
            slop (float): The maximum time difference (in seconds) allowed for a match.

        Returns:
            List of synchronized (left_msg, right_msg) tuples.
        """
        left_stamps = sorted(left_data.keys())
        right_stamps = sorted(right_data.keys())

        synchronized_pairs = []
        l_idx, r_idx = 0, 0
        used_r_indices = set()

        while l_idx < len(left_stamps) and r_idx < len(right_stamps):
            l_stamp = left_stamps[l_idx]
            best_match_r_stamp = None
            min_diff = slop

            temp_r_idx = r_idx
            while temp_r_idx < len(right_stamps):
                r_stamp = right_stamps[temp_r_idx]
                diff = abs(l_stamp - r_stamp)

                if diff <= min_diff:
                    min_diff = diff
                    best_match_r_stamp = r_stamp
                    search_start_r_idx = temp_r_idx
                
                if r_stamp - l_stamp > slop:
                    break
                
                temp_r_idx += 1

            if best_match_r_stamp is not None and search_start_r_idx not in used_r_indices:
                l_msg = left_data[l_stamp]
                r_msg = right_data[best_match_r_stamp]
                synchronized_pairs.append((l_msg, r_msg))
                used_r_indices.add(search_start_r_idx)
                r_idx = search_start_r_idx + 1
            
            l_idx += 1

        unpaired_count = len(left_data) + len(right_data) - 2 * len(synchronized_pairs)
        rospy.loginfo("Successfully synchronized %d message pairs." % len(synchronized_pairs))
        rospy.loginfo("%d messages could not be paired and were discarded." % unpaired_count)

        return synchronized_pairs
    
    def _smooth_and_filter_data(self, data_dict, cutoff, order=4):
        """
        Applies a two-stage filter: Savitzky-Golay for smoothing followed by a
        low-pass Butterworth filter for removing jitter.

        Args:
            data_dict (Dict[float, PointStamped]): A dictionary of PointStamped messages.
            cutoff (float): The cutoff frequency for the Butterworth filter in Hz.
            order (int): The order of the Butterworth filter.

        Returns:
            Dict[rospy.Time, PointStamped]: A new dictionary with the filtered data.
        """
        if len(data_dict) < 20:
            rospy.logwarn("Not enough data points to filter, returning original data.")
            return data_dict

        # --- Extract Data ---
        sorted_stamps = sorted(data_dict.keys())
        data_series = [data_dict[ts] for ts in sorted_stamps]
        x_vals = np.array([msg.point.x for msg in data_series])
        y_vals = np.array([msg.point.y for msg in data_series])
        z_vals = np.array([msg.point.z for msg in data_series])
        # --------------------

        # --- Stage 1: Savitzky-Golay Smoothing ---
        # This filter smooths data by fitting a polynomial to a window of points.
        # It's good at preserving peak shape and height.
        # `window_length`: Number of points for the fit. Must be an odd integer.
        # `polyorder`: Order of the polynomial. Must be less than window_length.
        savgol_window = 11  # Must be odd
        savgol_poly = 3
        if len(x_vals) > savgol_window:
            x_vals = signal.savgol_filter(x_vals, savgol_window, savgol_poly)
            y_vals = signal.savgol_filter(y_vals, savgol_window, savgol_poly)
            z_vals = signal.savgol_filter(z_vals, savgol_window, savgol_poly)

        # --- Stage 2: Butterworth Low-pass Filter ---
        fs = 1.0 / np.mean(np.diff(sorted_stamps))
        rospy.loginfo("Estimated sampling frequency: %.2f Hz", fs)

        # Design Filter
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Apply a zero-phase filter (filtfilt) to each axis
        x_filtered = signal.filtfilt(b, a, x_vals)
        y_filtered = signal.filtfilt(b, a, y_vals)
        z_filtered = signal.filtfilt(b, a, z_vals)

        # --- Reconstruct Data ---
        filtered_dict = {}
        for i, stamp_sec in enumerate(sorted_stamps):
            new_msg = PointStamped()
            new_msg.header = data_dict[stamp_sec].header
            new_msg.point.x = x_filtered[i]
            new_msg.point.y = y_filtered[i]
            new_msg.point.z = z_filtered[i]
            filtered_dict[stamp_sec] = new_msg
        
        return filtered_dict

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
        
        cache_duration = 3600
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(cache_duration))
        
        populated_messages = {topic: {} for topic in topics_to_read}

        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                # Check if the TF buffer cache is long enough for the entire bag
                bag_duration_secs = bag.get_end_time() - bag.get_start_time()
                if bag_duration_secs > cache_duration:
                    rospy.logwarn("Bag duration (%.2f s) is greater than TF buffer cache time (%.2f s).", 
                                  bag_duration_secs, cache_duration)
                    rospy.logwarn("This may lead to loss of old transforms and cause lookup errors. "
                                  "Consider increasing the 'cache_duration' for the tf2_ros.Buffer.")

                # Iterate over all specified topics
                for topic, msg, t in bag.read_messages(topics=topics_to_read):
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
                    
                    elif topic in populated_messages:
                        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                            populated_messages[topic][msg.header.stamp.to_sec()] = msg
                        else:
                            rospy.logwarn("Msg on topic '%s' has no header/stamp.", topic)

        except rosbag.ROSBagException as e:
            rospy.logerr("Error reading bag file: %s", e)
            return None, None
        
        populated_messages.pop('/tf', None)
        populated_messages.pop('/tf_static', None)

        rospy.loginfo("Bag data reading complete.")
        for topic, messages in populated_messages.items():
            rospy.loginfo("Topic: %s, Number of messages: %d", topic, len(messages))

        return tf_buffer, populated_messages

    def _transform_to_map_frame(self, toe_data_dict):
        """
        Transforms a dictionary of PointStamped messages to the target frame.

        Args:
            toe_data_dict (Dict[float, PointStamped]): A dictionary of points to transform.

        Returns:
            Dict[float, PointStamped]: A new dictionary of transformed points.
        """
        transformed_points = {}
        if not toe_data_dict:
            return {}

        source_frame = next(iter(toe_data_dict.values())).header.frame_id

        for stamp_sec, point_stamped in toe_data_dict.items():
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map',
                    source_frame,
                    rospy.Time.from_sec(stamp_sec),
                    rospy.Duration(0.1)
                )
                point_in_map = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                transformed_points[point_in_map.header.stamp.to_sec()] = point_in_map

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn("Could not transform point at time %s: %s", str(stamp_sec), e)
        
        rospy.loginfo("Transformed %d out of %d points to 'map' frame.", len(transformed_points), len(toe_data_dict))
        return transformed_points

    def _get_normalized_distance(self, data_dict):
        """
        Calculates the normalized Euclidean distance for a dictionary of PointStamped messages.

        Args:
            data_dict (Dict[float, PointStamped]): A dictionary of messages.

        Returns:
            tuple(np.array, np.array): A tuple containing:
                - An array of timestamps in seconds.
                - An array of corresponding normalized distances.
        """
        if not data_dict:
            return np.array([]), np.array([])

        sorted_stamps = sorted(data_dict.keys())
        
        distances = np.array([
            np.sqrt(data_dict[ts].point.x**2 + data_dict[ts].point.y**2 + data_dict[ts].point.z**2)
            for ts in sorted_stamps
        ])
        
        return np.array(sorted_stamps), distances
    
    def gait_parameters(self):
        """
        Calculates gait parameters by finding peaks and valleys in the distance data.
        Uses pre-processed data stored in instance variables.
        """
        if len(self.left_t) < 2:
            rospy.logwarn("Not enough data to perform gait analysis.")
            return {}

        # --- Estimate Sampling Frequency from Timestamps ---
        fs = 1.0 / np.mean(np.diff(self.left_t))
        rospy.loginfo("Gait analysis using estimated sampling frequency: %.2f Hz", fs)

        # --- Peak Detection Parameters ---
        # `prominence`: Required prominence of a peak. Measures how much a peak stands out from its surroundings.
        #             This is often more robust than a fixed height threshold.
        # `min_peak_distance_sec`: The minimum expected time between consecutive steps.
        peak_prominence = 0.025  # Adjust based on expected peak shape and noise level
        min_peak_distance_sec = 0.4  # Corresponds to a max cadence of 150 steps/min
        peak_distance = int(min_peak_distance_sec * fs)
        # print("Using peak prominence: {:.3f} and minimum peak distance: {} samples ({:.2f} sec)".format(
        #     peak_prominence, peak_distance, min_peak_distance_sec))

        # TODO: (Andreas) This is only valid for RoboTrainer moving forward in +x direction, when moving backwards the peak valley mapping to TO/HS is inverted
        left_to_idx, _ = signal.find_peaks(self.left_dist, distance=peak_distance, prominence=peak_prominence)
        left_hs_idx, _ = signal.find_peaks(-self.left_dist, distance=peak_distance, prominence=peak_prominence)
        right_to_idx, _ = signal.find_peaks(self.right_dist, distance=peak_distance, prominence=peak_prominence)
        right_hs_idx, _ = signal.find_peaks(-self.right_dist, distance=peak_distance, prominence=peak_prominence)

        rospy.loginfo("Detected %d left peaks and %d left valleys.", len(left_to_idx), len(left_hs_idx))
        rospy.loginfo("Detected %d right peaks and %d right valleys.", len(right_to_idx), len(right_hs_idx))

        # --- Calculate Stride Parameters ---
        left_strides = self._calculate_strides(self.toe_left_msg_map_frame, self.left_t, left_hs_idx, left_to_idx)
        right_strides = self._calculate_strides(self.toe_right_msg_map_frame, self.right_t, right_hs_idx, right_to_idx)

        # --- Calculate Step Parameters ---
        left_steps, right_steps = self._calculate_step_lengths(
            self.left_t, self.right_t, left_hs_idx, right_hs_idx,
            self.toe_left_msg_map_frame, self.toe_right_msg_map_frame
        )

        avg_param_dict = {}

        if not left_strides['stride_length'] or not right_strides['stride_length'] or not left_steps['step_length'] or not right_steps['step_length']:
            rospy.logwarn("Insufficient stride or step data to compute averages.")
            return avg_param_dict
        
        avg_param_dict['/left/stride_length/avg'] = np.mean(left_strides['stride_length'])
        avg_param_dict['/left/stride_duration/avg'] = np.mean(left_strides['stride_duration'])
        avg_param_dict['/left/stride_swing_time/avg'] = np.mean(left_strides['stride_swing_time'])
        avg_param_dict['/left/stride_stance_time/avg'] = np.mean(left_strides['stride_stance_time'])
        avg_param_dict['/left/num_strides'] = len(left_strides['stride_length'])
        left_raw = left_strides.copy()
        left_raw.update(left_steps)
        avg_param_dict['/left/raw'] = left_raw
        
        avg_param_dict['/right/stride_length/avg'] = np.mean(right_strides['stride_length'])
        avg_param_dict['/right/stride_duration/avg'] = np.mean(right_strides['stride_duration'])
        avg_param_dict['/right/stride_swing_time/avg'] = np.mean(right_strides['stride_swing_time'])
        avg_param_dict['/right/stride_stance_time/avg'] = np.mean(right_strides['stride_stance_time'])
        avg_param_dict['/right/num_strides'] = len(right_strides['stride_length'])
        right_raw = right_strides.copy()
        right_raw.update(right_steps)
        avg_param_dict['/right/raw'] = right_raw

        avg_param_dict['/left/step_length/avg'] = np.mean(left_steps['step_length'])
        avg_param_dict['/left/step_duration/avg'] = np.mean(left_steps['step_duration'])
        avg_param_dict['/left/num_steps'] = len(left_steps['step_length'])

        avg_param_dict['/right/step_length/avg'] = np.mean(right_steps['step_length'])
        avg_param_dict['/right/step_duration/avg'] = np.mean(right_steps['step_duration'])
        avg_param_dict['/right/num_steps'] = len(right_steps['step_length'])

        avg_param_dict['/timestamp'] = left_steps['step_timestamps'][-1] 

        # Cadence (steps/min) = 60 / (left_stride_durations + right_stride_durations) / 2
        avg_stride_duration = (avg_param_dict['/left/stride_duration/avg'] + avg_param_dict['/right/stride_duration/avg']) / 2.0
        avg_param_dict['/cadence/avg'] = 60.0 / avg_stride_duration

        # Speed (m/s) = avg_stride_length / avg_stride_duration
        avg_stride_length = (avg_param_dict['/left/stride_length/avg'] + avg_param_dict['/right/stride_length/avg']) / 2.0
        avg_param_dict['/speed/avg'] = avg_stride_length / avg_stride_duration

        # --- Plot comparison ---
        # plot_lines = {
        #     'original_left': (estimator.original_left_t, estimator.original_left_dist),
        #     'original_right': (estimator.original_right_t, estimator.original_right_dist),
        #     # 'left_dist': (self.left_t, self.left_dist),
        #     # 'right_dist': (self.right_t, self.right_dist),
        #     '/left/stride_length': (np.array(avg_param_dict['/left/raw']['stride_timestamps']), np.array(avg_param_dict['/left/raw']['stride_length'])),
        #     '/right/stride_length': (np.array(avg_param_dict['/right/raw']['stride_timestamps']), np.array(avg_param_dict['/right/raw']['stride_length'])),
        #     '/left/step_length': (np.array(avg_param_dict['/left/raw']['step_timestamps']), np.array(avg_param_dict['/left/raw']['step_length'])),
        #     '/right/step_length': (np.array(avg_param_dict['/right/raw']['step_timestamps']), np.array(avg_param_dict['/right/raw']['step_length'])),
        # }
        # plot_points = {
        #     'left_to': (self.left_t[left_to_idx], self.left_dist[left_to_idx]),
        #     'left_hs': (self.left_t[left_hs_idx], self.left_dist[left_hs_idx]),
        #     'right_to': (self.right_t[right_to_idx], self.right_dist[right_to_idx]),
        #     'right_hs': (self.right_t[right_hs_idx], self.right_dist[right_hs_idx]),
        # }

        # self.plot_data(plot_lines, plot_points)

        return avg_param_dict

    def _calculate_step_lengths(self, left_t, right_t, left_hs_idx, right_hs_idx, left_pos_data, right_pos_data):
        """
        Calculates step length by projecting the step vector onto the walking direction.
        Returns two dictionaries, one for each foot, containing step parameters.
        """
        left_steps = {'step_length': [], 'step_duration': [], 'step_timestamps': []}
        right_steps = {'step_length': [], 'step_duration': [], 'step_timestamps': []}

        left_hs_times = left_t[left_hs_idx]
        right_hs_times = right_t[right_hs_idx]

        # Create a single sorted list of all heel-strike events
        all_hs_events = sorted(
            [(t, 'left') for t in left_hs_times] +
            [(t, 'right') for t in right_hs_times]
        )

        if len(all_hs_events) < 3:
            rospy.logwarn("Not enough heel strikes to calculate step length.")
            return left_steps, right_steps

        # Iterate through sequences of three consecutive heel strikes (e.g., L->R->L or R->L->R)
        for i in range(len(all_hs_events) - 2):
            hs1_time, hs1_foot = all_hs_events[i]
            hs2_time, hs2_foot = all_hs_events[i+1]
            hs3_time, hs3_foot = all_hs_events[i+2]

            # We need an alternating foot pattern (e.g., left-right-left)
            if hs1_foot == hs3_foot and hs1_foot != hs2_foot:
                try:
                    # Get the 3D points for the three heel strikes
                    pos_data1 = left_pos_data if hs1_foot == 'left' else right_pos_data
                    pos_data2 = left_pos_data if hs2_foot == 'left' else right_pos_data
                    
                    p1 = np.array([pos_data1[hs1_time].point.x, pos_data1[hs1_time].point.y, pos_data1[hs1_time].point.z])
                    p2 = np.array([pos_data2[hs2_time].point.x, pos_data2[hs2_time].point.y, pos_data2[hs2_time].point.z])
                    p3 = np.array([pos_data1[hs3_time].point.x, pos_data1[hs3_time].point.y, pos_data1[hs3_time].point.z])

                    # Vector representing the walking direction for this stride
                    walking_vector = p3 - p1
                    walking_vector_norm = np.linalg.norm(walking_vector)

                    if walking_vector_norm < 1e-6: # Avoid division by zero
                        continue

                    # Vector representing the current step
                    step_vector = p3 - p2

                    # Project step_vector onto walking_vector to get the step length
                    # Formula: |step_vector . walking_vector| / |walking_vector|
                    step_length = np.abs(np.dot(step_vector, walking_vector)) / walking_vector_norm
                    step_duration = hs3_time - hs2_time

                    # Filter out steps that are essentially zero length (e.g., from standing still)
                    if step_length > 0.01: # 1 cm threshold
                        if hs3_foot == 'left':
                            left_steps['step_length'].append(step_length)
                            left_steps['step_duration'].append(step_duration)
                            left_steps['step_timestamps'].append(hs3_time)
                        else:
                            right_steps['step_length'].append(step_length)
                            right_steps['step_duration'].append(step_duration)
                            right_steps['step_timestamps'].append(hs3_time)

                except KeyError as e:
                    rospy.logwarn("KeyError finding position data for timestamp %s. Skipping step calculation.", e)
                except Exception as e:
                    rospy.logerr("An error occurred during step length calculation: %s", e)

        return left_steps, right_steps

    def _calculate_strides(self, position_data, timestamps, toe_off_indices, heel_strike_indices):
        """
        Calculates stride parameters based on heel-strike and toe-off events.

        A stride is defined from one heel-strike to the next of the same foot.
        It is composed of a stance phase (HS to TO) and a swing phase (TO to next HS).

        Args:
            position_data (Dict[float, PointStamped]): Dictionary of toe positions in the map frame.
            timestamps (np.array): Array of timestamps (in seconds) corresponding to the data used for peak detection.
            toe_off_indices (list): List of array indices for toe-off events (valleys).
            heel_strike_indices (list): List of array indices for heel-strike events (peaks).

        Returns:
            dict: A dictionary containing lists of 'stride_length', 'stride_duration',
                  'stride_swing_time', 'stride_stance_time', and 'stride_timestamps' for all detected strides.
        """
        strides_dict = {
            'stride_length': [],
            'stride_duration': [],
            'stride_swing_time': [],
            'stride_stance_time': [],
            'stride_timestamps': [],
            'HS_in_map_frame': [],
            'TO_in_map_frame': [],
        }
        if len(heel_strike_indices) < 2:
            rospy.logwarn("Not enough heel-strike events to calculate strides.")
            return strides_dict
        
        # Add PointStamped messages in map_frame for heel-strikes
        for hs_idx in heel_strike_indices:
            if hs_idx < len(timestamps):
                strides_dict['HS_in_map_frame'].append(position_data[timestamps[hs_idx]])
            else:
                rospy.logwarn("Heel-strike index %d out of bounds for timestamps array of length %d.", hs_idx, len(timestamps))

        # Add PointStamped messages in map_frame for toe-offs
        for to_idx in toe_off_indices:
            if to_idx < len(timestamps):
                strides_dict['TO_in_map_frame'].append(position_data[timestamps[to_idx]])
            else:
                rospy.logwarn("Toe-off index %d out of bounds for timestamps array of length %d.", to_idx, len(timestamps))

        # Iterate through consecutive heel-strikes to define each stride
        for i in range(len(heel_strike_indices) - 1):
            hs1_idx = heel_strike_indices[i]
            hs2_idx = heel_strike_indices[i+1]

            # Find the toe-off that occurs between these two heel-strikes
            to_indices_within_stride = [to_idx for to_idx in toe_off_indices if hs1_idx < to_idx < hs2_idx]
            if not to_indices_within_stride:
                continue # Not a valid stride if no toe-off occurs

            to_idx = to_indices_within_stride[0]

            hs1_time_sec = timestamps[hs1_idx]
            hs2_time_sec = timestamps[hs2_idx]
            to_time_sec = timestamps[to_idx]

            # --- Calculate Durations ---
            stride_duration = hs2_time_sec - hs1_time_sec
            stance_time = to_time_sec - hs1_time_sec
            swing_time = hs2_time_sec - to_time_sec

            # --- Calculate Stride Length ---
            # Look up the 3D positions using the timestamps as keys
            if hs1_time_sec in position_data and hs2_time_sec in position_data:
                p_hs1 = position_data[hs1_time_sec].point
                p_hs2 = position_data[hs2_time_sec].point
                stride_length = np.sqrt((p_hs2.x - p_hs1.x)**2 + (p_hs2.y - p_hs1.y)**2 + (p_hs2.z - p_hs1.z)**2)

                # Filter out strides that are essentially zero length and have valid duration
                if stride_duration > 0 and stride_length > 0.01: # 1 cm threshold
                    strides_dict['stride_length'].append(stride_length)
                    strides_dict['stride_duration'].append(stride_duration)
                    strides_dict['stride_swing_time'].append(swing_time)
                    strides_dict['stride_stance_time'].append(stance_time)
                    strides_dict['stride_timestamps'].append(hs2_time_sec)
            else:
                rospy.logwarn("Could not find position data for a stride event timestamp. Skipping stride.")

        return strides_dict

    def write_to_bag(self, bag_path, param_dict):
        """
        Writes the gait parameters to a new rosbag file.

        Args:
            bag_path (str): The path to the output rosbag file.
            param_dict (dict): The dictionary of gait parameters to write.
        """
        try:
            with rosbag.Bag(bag_path, 'w') as bag:
                timestamp_avg = param_dict["/timestamp"] 

                for key, value in param_dict.items():
                    if 'timestamp' in key:
                        continue

                    if isinstance(value, dict):
                        for k in value.keys():
                            timestamps_raw = value["step_timestamps"] if "step" in value else value["stride_timestamps"]
                            if "timestamps" in k:
                                continue
                            for data, t in zip(value[k], timestamps_raw):
                                if isinstance(data, float):
                                    msg = Float32()
                                    msg.data = data
                                    bag.write('/gait' + key + '/' + k , msg, rospy.Time.from_sec(t))
                                elif isinstance(data, PointStamped):
                                    bag.write('/gait' + key + '/' + k , data, data.header.stamp)
                                else:
                                    rospy.logwarn("Unsupported data type in list for key '%s'. Skipping.", k)
                                    continue
                                
                        
                    elif isinstance(value, (float, int)):
                        msg = Float32() if isinstance(value, float) else Int32()
                        msg.data = value
                        bag.write('/gait' + key, msg, rospy.Time.from_sec(timestamp_avg))

                    else:
                        rospy.logwarn("Unsupported data type for key '%s', type: %s. Skipping.", key, type(value))
                        continue
                    

            rospy.loginfo("Gait parameters written to bag: %s", bag_path)
        except Exception as e:
            rospy.logerr("Failed to write gait parameters to bag: %s", e)

if __name__ == '__main__':

    rospy.init_node('gait_estimation_from_bag')
    
    bag_file_path = '/home/docker/ros_ws/data/toe_positions.bag'
    output_bag_path = bag_file_path.replace('.bag', '_gait_output.bag')
    topics = [
        '/tf',
        '/tf_static',
        '/toe_position/left/kalman',
        '/toe_position/right/kalman',
    ]

    estimator = EstimatorToeFromBag(bag_file_path, topics)

    # --- Calculate gait parameters ---
    param_dict = estimator.gait_parameters()

    filtered_param_dict = {k: v for k, v in param_dict.items() if 'raw' not in k}
    rospy.loginfo("Estimated Gait Parameters: %s", filtered_param_dict)

    # --- Write results to a new bag file ---
    estimator.write_to_bag(output_bag_path, param_dict)

#!/usr/bin/env python

from gait_estimator_base_refactor import EstimatorBaseRefactor
from gait_estimator_base_refactor import *

import matplotlib.pyplot as plt


class EstimatorToeRefactor(EstimatorBaseRefactor):

    def __init__(self):

        super(EstimatorToeRefactor, self).__init__('toe')
        self._sensor = 'toe'
        self._toe_data = []  # type: List[PoseArray]

        self.toe_data = TimeSeriesData(self._fs, self._window_size, self._window_step)

        self._toe_lock = Lock()
        self._sub_toes = rospy.Subscriber("/toe_detection/toe_positions", PoseArray, self.listen_toes)

        self._data_synced = False

    def listen_toes(self, data):
        # type: (PoseArray) -> None
        with self.toe_data.lock:
            self.toe_data.data.append(data)
            self.toe_data.est_fs_and_update(3)

    # def get_window_by_timestamp(self, data, cutoff_time):
    #     if len(data) == 0:
    #             return None

    #     with self._toe_lock:
    #         first_index_in_window = 0

    #         print("BEFORE Length: ", len(data))
    #         print("BEFORE Time difference: ", data[-1].header.stamp.to_sec() - data[0].header.stamp.to_sec())

    #         # Find the first valid timestamp in ascending order
    #         while first_index_in_window < len(data) and data[first_index_in_window].header.stamp.to_sec() < cutoff_time:
    #             first_index_in_window += 1

    #         # Slice from the first valid index to the end
    #         data = data[first_index_in_window:]

    #         if len(data) == 0:
    #             return None

    #         window = np.array(data, copy=True)
    #         print("AFTER Length: ", len(window))
    #         print("AFTER Time difference: ", window[-1].header.stamp.to_sec() - window[0].header.stamp.to_sec())
    #         return window

    # This is to compensate for one topic beeing published from bag while others are calculated in real-time
    def sync_data(self):
        if self._data_synced:
            return True
        if self.toe_data.last_timestamp is 0.0 or self.velocity_data.last_timestamp is 0.0 or self.pose_data.last_timestamp is 0.0:
            return False

        min_timestamp = min(self.toe_data.last_timestamp, self.velocity_data.last_timestamp, self.pose_data.last_timestamp)
        self.toe_data.timestamp_offset = self.toe_data.last_timestamp - min_timestamp
        self.velocity_data.timestamp_offset = self.velocity_data.last_timestamp - min_timestamp
        self.pose_data.timestamp_offset = self.pose_data.last_timestamp - min_timestamp

        rospy.loginfo("Toe data synced with timestamp offset: %.2f", self.toe_data.timestamp_offset)

        return True

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
    rospy.init_node('gait_estimation', log_level=rospy.INFO)
    rospy.get_rostime()
    rospy.get_time()

    # Retrieve bool parameters (with default = True)
    use_force = rospy.get_param('/gait_estimation/use_force', True)
    use_leg = rospy.get_param('/gait_estimation/use_leg', True)
    use_toe = rospy.get_param('/gait_estimation/use_toe', True)
    use_shoulder = rospy.get_param('/gait_estimation/use_shoulder', True)

    window_size = rospy.get_param('/gait_estimation/window_size', 7.0)
    window_step = rospy.get_param('/gait_estimation/window_step', 1.0)
    estimators = {}
    pub_dicts = {}

    force_pub = rospy.Publisher('/gait/force_params', gp, tcp_nodelay=True, queue_size=1024)
    legs_pub = rospy.Publisher('/gait/leg_params', gp, tcp_nodelay=True, queue_size=1024)
    toe_pub = rospy.Publisher('/gait/toe_params', gp, tcp_nodelay=True, queue_size=1024)
    shoulder_pub = rospy.Publisher('/gait/shoulder_params', gp, tcp_nodelay=True, queue_size=1024)
    left_toe_heelstrike_in_map = rospy.Publisher('/gait/left_toe_heelstrike_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)
    right_toe_heelstrike_in_map = rospy.Publisher('/gait/right_toe_heelstrike_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)
    left_toe_toe_off_in_map = rospy.Publisher('/gait/left_toe_toe_off_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)
    right_toe_toe_off_in_map = rospy.Publisher('/gait/right_toe_toe_off_in_map', PointStamped, tcp_nodelay=True, queue_size=1024)

    rospy.sleep(rospy.Duration(0.25))
    # est_force = EstimatorForce()
    # est_leg = EstimatorLegs()
    est_toe = EstimatorToeRefactor()
    # est_sh = EstimatorShoulder()

    # Add each estimator to the dictionary only if its parameter is true
    # if use_force:
    #     estimators['force'] = est_force
    # if use_leg:
    #     estimators['legs'] = est_leg
    if use_toe:
        estimators['toe'] = est_toe
    # if use_shoulder:
    #     estimators['shoulder'] = est_sh

    rospy.loginfo("Start collecting data")
    rospy.sleep(rospy.Duration(window_size))
    rospy.loginfo("Start timed thread")
    rospy.timer.Timer(rospy.Duration(window_step), est_toe.gait_estimation)

    while not rospy.is_shutdown():
        rospy.spin()

from __future__ import division
from gait_estimator_base import EstimatorBase
from gait_estimator_base import *
from threading import Lock
from message_filters import ApproximateTimeSynchronizer, Subscriber


class EstimatorToe(EstimatorBase):

    def __init__(self):

        super(EstimatorToe, self).__init__('toe')
        self._sensor = 'toe'
        self._right_toe = []
        self._left_toe = []

        self._tlock = Lock()
        self._sub_rtoe_sync = Subscriber("/right_toe",PointStamped)
        self._sub_ltoe_sync = Subscriber("/left_toe",PointStamped)
        self._ats = ApproximateTimeSynchronizer([self._sub_rtoe_sync, self._sub_ltoe_sync], queue_size=5, slop=0.2)
        self._ats.registerCallback(self.toe_sync)



    #Listen to Topic for toe position
    def toe_sync(self, right_toe, left_toe):
        self._tlock.acquire()
        self._right_toe.append(right_toe)
        self._left_toe.append(left_toe)
        if len(self._right_toe) > 3:
            fs = self.est_fs(self._right_toe[-3:])
            if fs > 0.0:
                self._fs = (0.1 * fs + 0.9 * self._fs)
        self._tlock.release()

    def get_window(self):
        step_percentage = self._window_step / self._window_size
        cutoff_r = len(self._right_toe) * step_percentage
        cutoff_l = len(self._left_toe) * step_percentage
        self._tlock.acquire()
        win_r = np.array(self._right_toe, copy = True)
        win_l = np.array(self._left_toe, copy = True)
        if win_r.shape[0] == 0 or win_l.shape[0] == 0:
            self._tlock.release()
            return None
        else:
            self._right_toe = self._right_toe[int(cutoff_r):]
            self._left_toe = self._left_toe[int(cutoff_l):]
            self._tlock.release()
            return [win_l, win_r]

    #Synchronize Toe position so same amount of measurement for right and left position is given
    def remove_toes(self, smaller, bigger):
        truncated_arr = []
        bigger_stamps = [b.header.stamp.to_sec() for b in bigger]

        for x in smaller:
            closest_ind = self.closest_node(x.header.stamp.to_sec(), bigger_stamps)
            truncated_arr.append(bigger[closest_ind])
        return truncated_arr

    #estimate gait parameters from toe positions
    def gait_estimation(self, window, pipe):
        ltoe_window = window[0]
        rtoe_window = window[1]
        old_fs = self._fs
        self._fs = min(self.est_fs(ltoe_window), self.est_fs(rtoe_window))
        if self._fs == 0:
            self._fs = old_fs

        self.get_avg_speed()
        self.set_pose_win()


        rtime = [rt.header.stamp.to_sec() for rt in rtoe_window]
        if len(rtoe_window) < len(ltoe_window):
            ltoe_window = self.remove_toes(rtoe_window, ltoe_window)
            rtime = [rt.header.stamp.to_sec() for rt in rtoe_window]
        elif len(ltoe_window) < len(rtoe_window):
            rtoe_window = self.remove_toes(ltoe_window, rtoe_window)
            rtime = [lt.header.stamp.to_sec() for lt in ltoe_window]


        ltoe_rms = [lt.point.x ** 2 + lt.point.y ** 2 for lt in ltoe_window]
        ltoe_rms = np.sqrt(ltoe_rms)
        rtoe_rms = [rt.point.x ** 2 + rt.point.y ** 2 for rt in rtoe_window]
        rtoe_rms = np.sqrt(rtoe_rms)

        toe_diff = np.sqrt(np.array([((ltoe_window[i].point.x - rtoe_window[i].point.x) ** 2 + (ltoe_window[i].point.y - rtoe_window[i].point.y) ** 2) for i in range(len(rtime))]))

        order = 3
         #preprocess data by applying lowpass filter to smoothe and bandpass filter to filter signals outside of gait cadence
        try:
            ltoe_smoothed = prep.smooth_data(ltoe_rms, self._highcut / 2.0, self._fs, win_size = 0.3)
            rtoe_smoothed = prep.smooth_data(rtoe_rms, self._highcut /2.0, self._fs, win_size = 0.3)
            toe_diff_smoothed = prep.smooth_data(toe_diff, self._highcut, self._fs, win_size = 0.3)

            td_bandpassed = prep.butter_bandpass_filter(toe_diff_smoothed, self._lowcut / 2.0, self._highcut, self._fs, order)
        except:
            ltoe_smoothed = prep.smooth_data(ltoe_rms, self._highcut / 2.0, self._fs, win_size = 0.3)
            rtoe_smoothed = prep.smooth_data(rtoe_rms, self._highcut / 2.0, self._fs, win_size = 0.3)
            toe_diff_smoothed = prep.smooth_data(toe_diff, self._highcut, self._fs, win_size = 0.3)

            td_bandpassed = prep.butter_bandpass_filter(toe_diff_smoothed, self._lowcut / 2.0, self._highcut, self._fs, order - 1)

        params = gp()
        params.header.stamp = rospy.Time.now()

        debug_plt = []
        plt_ind = []
        debugs = []
        
        #check if robot moved by velocity, if under threshold set cadence to zero
        if all(s < 0.2 for s in self._window_vel):
            params.cadence = 0.0
            params.cadence_avg = 0.0
        else:
            cad, cad_avg, debugs = self._wflc.wflc(td_bandpassed, self._fs, t = rtime)
            params.cadence = cad 
            params.cadence_avg = cad_avg 


        leg1_param = leg_params()
        leg2_param = leg_params()

        time_stamps = rtime
        min_peak_dist = self._fs * (1.0 / self._highcut) * 2 
        pkwidth = self._fs * (0.1 / self._highcut) if self._fs * (0.1 / self._highcut) > 1.0 else 1.0

        peak_indexes_l1, _ = signal.find_peaks(ltoe_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        valley_indexes_l1, _ = signal.find_peaks(-ltoe_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        peak_indexes_l2, _ = signal.find_peaks(rtoe_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        valley_indexes_l2, _ = signal.find_peaks(-rtoe_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)

        if self._window_vel[0] == max(self._window_vel):
            if self._window_vel[0] >= 0:
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
            if self._window_vel[1] >= 0:
                to_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])
                hs_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])

                to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
                hs_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2])
            else:
                to_l1_t = np.array([time_stamps[v] for v in valley_indexes_l1])           
                hs_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])

                to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])     
                hs_l2_t = np.array([time_stamps[v] for v in valley_indexes_l2]) 

        leg1_param.cadence = self.cadence_leg(time_stamps, hs_l1_t, to_l1_t)
        leg2_param.cadence = self.cadence_leg(time_stamps, hs_l2_t, to_l2_t)

        hs_l1_ind = [time_stamps.index(hs1) for hs1 in hs_l1_t]
        hs_l2_ind = [time_stamps.index(hs2) for hs2 in hs_l2_t]
        to_l1_ind = [time_stamps.index(to1) for to1 in to_l1_t]
        to_l2_ind = [time_stamps.index(to2) for to2 in to_l2_t]

        leg1_param.stride_length, leg1_param.swing_time = self.stride_length(time_stamps, ltoe_rms, hs_l1_ind, to_l1_ind)
        leg2_param.stride_length, leg2_param.swing_time = self.stride_length(time_stamps, rtoe_rms, hs_l2_ind, to_l2_ind)

        leg1_param.stride_intervall = self.stride_time(to_l1_t)
        leg2_param.stride_intervall = self.stride_time(to_l2_t)

        if leg1_param.stride_intervall != 0.0:
            leg1_param.cadence = 1.0 / leg1_param.stride_intervall
        if leg2_param.stride_intervall != 0.0:
            leg2_param.cadence = 1.0 / leg2_param.stride_intervall

        leg1_param.stance_time = self.stance_duration(hs_l1_t, to_l1_t)
        leg2_param.stance_time = self.stance_duration(hs_l2_t, to_l2_t)

        leg1_param.step_length, leg2_param.step_length = self.step_length(time_stamps, ltoe_rms, rtoe_rms)
        params.leg1 = leg1_param
        params.leg2 = leg2_param

        ret_dict = {}
        ret_dict['/gait/toe_params'] = params
        ret_dict['/gait/toe_band'] = debugs
        pipe.put(ret_dict)

    def step_length(self, time_stamps, leg1, leg2):

        min_peak_dist = self._fs * (1.0 / self._highcut) * 2

        #since we're using RMS (absolute distance) peak = TO (maximum distance), valley = HS (minimum distance)
        min_peak_dist = self._fs * (1.0 / self._highcut) * 2
        pkwidth = self._fs * (0.1 / self._highcut) if self._fs * (0.1 / self._highcut) > 1.0 else 1.0
        peak_indexes_l1, _ = signal.find_peaks(leg1,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        valley_indexes_l1, _ = signal.find_peaks(-leg1,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        peak_indexes_l2, _ = signal.find_peaks(leg2,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        valley_indexes_l2, _ = signal.find_peaks(-leg2,distance = min_peak_dist, prominence = 0.025, width = pkwidth)

        to_l1_t = np.array([time_stamps[p] for p in peak_indexes_l1])
        hs_l1_t = np.array([time_stamps[p] for v in valley_indexes_l1])

        to_l2_t = np.array([time_stamps[p] for p in peak_indexes_l2])
        hs_l2_t = np.array([time_stamps[p] for v in valley_indexes_l2])

        xc,yc = prep.interpolated_intercept(np.array(time_stamps), leg1, leg2)
        
        xc_ind = []
        
        for x in xc:
            if x in time_stamps:
                xc_ind.append(time_stamps.index(x))
            else:
                ind = next( (t for t in time_stamps if t > x ), None)
                if ind:
                    xc_ind.append(time_stamps.index(ind))
        
        l1steps = []
        l2steps = []

        #rotation Hs = valley; negative x :HS = Peak; negative y HS:valley
        l1_hs_ind = valley_indexes_l1
        l2_hs_ind = valley_indexes_l2

        for i in range(len(l1_hs_ind)):
            if any(x > l1_hs_ind[i] for x in xc_ind):
                x1 = next( (x for x in xc_ind if x > l1_hs_ind[i] ), None)
                if x1 and ((i == len(l1_hs_ind) - 1) or (x1 < l1_hs_ind[i + 1])):
                    l1steps.append(leg1[l1_hs_ind[i]] - leg1[x1])
        for i in range(len(l2_hs_ind)):
            if any(x > l2_hs_ind[i] for x in xc_ind):
                x2 = next( (x for x in xc_ind if x > l2_hs_ind[i] ), None)
                if x2 and ((i == len(l2_hs_ind) - 1) or (x2 < l2_hs_ind[i + 1])):
                    l2steps.append(leg2[l2_hs_ind[i]] - leg2[x2])
        
        l1_step = 0.0
        l2_step = 0.0
        if l1steps:
            l1_step = sum(l1steps) / len(l1steps) 
            #rospy.loginfo("!!!! ::::::::: >>>>>STEP LENGTH L1 %.8f", sum(l1steps) / len(l1steps) )
        if l2steps:
            l2_step = sum(l2steps) / len(l2steps)
            #rospy.loginfo("!!!! ::::::::: >>>>>STEP LENGTH L2 %.8f", sum(l2steps) / len(l2steps) )
            
        return l1_step,l2_step

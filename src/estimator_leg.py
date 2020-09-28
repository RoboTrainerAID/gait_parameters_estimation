from __future__ import division
from gait_estimator_base import EstimatorBase
from gait_estimator_base import *
from leg_tracker.msg import PersonMsg
from threading import Lock

class EstimatorLegs(EstimatorBase):

    def __init__(self):
        super(EstimatorLegs, self).__init__('legs')

        self._sensor = 'legs'
        self._llock = Lock()

        self._sub_legs = rospy.Subscriber("/leg_detection/people_msg_stamped",PersonMsg, self.listen_leg_tracker)
        self._legs_pub = rospy.Publisher('/gait/leg_params', gp, tcp_nodelay=True, queue_size=1024)
        self._legs_band = rospy.Publisher('/gait/leg_band', PointStamped, tcp_nodelay=True, queue_size=1024)


    def listen_leg_tracker(self,data):
        self._llock.acquire()
        #Stamp are not unique, so reassign them here.
        data.header.stamp = rospy.Time.now()
        self._data.append(data)
        self._llock.release()
        if len(self._data) > 3:
            fs = self.est_fs(self._data[-3:])
            if fs > 0.0:
                self._fs = (0.1 * fs + 0.9 * self._fs)

    def get_window(self):
        step_percentage = self._window_step / self._window_size
        cutoff = len(self._data) * step_percentage
        self._llock.acquire()
        win = np.array(self._data, copy = True)
        if win.shape[0] == 0:
            self._llock.release()
            return None
        else:
            self._data = self._data[int(cutoff):]
            self._llock.release()
            return win

    def gait_estimation(self, window, pipe):
        old_fs = self._fs
        self._fs = self.est_fs(window)
        if self._fs == 0:
            self._fs = old_fs
        
        time_stamps = [w.header.stamp.to_sec() for w in window]

        self.get_avg_speed()
        self.set_pose_win()
        try:
            cutoff = int(len(self._data) * 0.2)
            leg1_avg_y = sum([w.leg1.position.y for w in window]) / len(window)
            leg2_avg_y = sum([w.leg2.position.y for w in window]) / len(window)
        except:
            leg1_avg_y = 1
            leg2_avg_y = 0

        #leg 1 = left, leg 2 = right
        if leg1_avg_y >= leg2_avg_y:
            leg1x = np.array([w.leg1.position.x for w in window])
            leg1y = np.array([w.leg1.position.y for w in window])
            leg2x = np.array([w.leg2.position.x for w in window])
            leg2y = np.array([w.leg2.position.y for w in window])
        else:
            leg1x = np.array([w.leg2.position.x for w in window])
            leg1y = np.array([w.leg2.position.y for w in window])
            leg2x = np.array([w.leg1.position.x for w in window])
            leg2y = np.array([w.leg1.position.y for w in window])

        legs1_rms = (leg1x ** 2) + (leg1y ** 2)
        legs1_rms = np.sqrt(legs1_rms)
        legs2_rms = (leg2x ** 2) + (leg2y ** 2)
        legs2_rms = np.sqrt(legs2_rms)

        leg_diff = np.sqrt(np.array([((w.leg1.position.x - w.leg2.position.x) ** 2 + (w.leg1.position.y - w.leg2.position.y) ** 2) for w in window]))

        order = 3
         #preprocess data by applying lowpass filter to smoothe and bandpass filter to filter signals outside of gait cadence
        try:
            leg1_smoothed = prep.smooth_data(legs1_rms, self._highcut, self._fs, win_size = 0.5)
            leg2_smoothed = prep.smooth_data(legs2_rms, self._highcut, self._fs, win_size = 0.5)
            ld_smoothed = prep.smooth_data(leg_diff, self._highcut, self._fs, win_size = 0.5)

            ld_bandpassed = prep.butter_bandpass_filter(ld_smoothed, self._lowcut / 2.0, self._highcut, self._fs, order)
        except:
            leg1_smoothed = prep.smooth_data(legs1_rms, self._highcut, self._fs, win_size = 0.5)
            leg2_smoothed = prep.smooth_data(legs2_rms, self._highcut, self._fs, win_size = 0.5)
            ld_smoothed = prep.smooth_data(leg_diff, self._highcut, self._fs, win_size = 0.5)

            ld_bandpassed = prep.butter_bandpass_filter(ld_smoothed, self._lowcut / 2.0, self._highcut, self._fs, order - 1)

        params = gp()
        params.header.stamp = rospy.Time.now()

        debug_plt = []
        plt_ind = []
        debugs = []
        
        #check if robot moved from velocity of robot, if under threshold set cadence to 0
        if all(s < 0.2 for s in self._window_vel):
            params.cadence = 0.0
            params.cadence_avg = 0.0
        else:
            cad, cad_avg, debugs = self._wflc.wflc(ld_bandpassed, self._fs, t = time_stamps)
            params.cadence = cad 
            params.cadence_avg = cad_avg 



        leg1_param = leg_params()
        leg2_param = leg_params()

        min_peak_dist = self._fs * (1.0 / self._highcut) * 2 
        pkwidth = self._fs * (0.1 / self._highcut) if self._fs * (0.1 / self._highcut) > 1.0 else 1.0
        
        peak_indexes_l1, _ = signal.find_peaks(legs1_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        valley_indexes_l1, _ = signal.find_peaks(-legs1_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        peak_indexes_l2, _ = signal.find_peaks(legs2_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)
        valley_indexes_l2, _ = signal.find_peaks(-legs2_rms,distance = min_peak_dist, prominence = 0.025, width = pkwidth)


        #TO HS dependend on movement direction
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


        leg1_param.stride_length, leg1_param.swing_time = self.stride_length(time_stamps, legs1_rms, hs_l1_ind, to_l1_ind)
        leg2_param.stride_length, leg2_param.swing_time = self.stride_length(time_stamps, legs2_rms, hs_l2_ind, to_l2_ind)
        


        leg1_param.stride_length *= 1.3
        leg2_param.stride_length *= 1.3

        leg1_param.stride_intervall = self.stride_time(to_l1_t)
        leg2_param.stride_intervall = self.stride_time(to_l2_t)

        if leg1_param.stride_intervall != 0.0:
            leg1_param.cadence = 1.0 / leg1_param.stride_intervall
        if leg2_param.stride_intervall != 0.0:
            leg2_param.cadence = 1.0 / leg2_param.stride_intervall

        leg1_param.stance_time = self.stance_duration(hs_l1_t, to_l1_t)
        leg2_param.stance_time = self.stance_duration(hs_l2_t, to_l2_t)

        leg1_param.step_length, leg2_param.step_length = self.step_length(time_stamps, legs1_rms, legs2_rms)
        params.leg1 = leg1_param
        params.leg2 = leg2_param

        ret_dict = {}
        ret_dict['/gait/leg_params'] = params
        ret_dict['/gait/leg_band'] = debugs
        pipe.put(ret_dict)

    def step_length(self, time_stamps, leg1, leg2):

        min_peak_dist = self._fs * (1.0 / self._highcut) * 2

        #since we're using RMS (absolute distance) peak = TO (maximum distance), valley = HS (minimum distance)
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


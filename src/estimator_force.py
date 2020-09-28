from __future__ import division
from gait_estimator_base import EstimatorBase
from gait_estimator_base import *
import tf2_ros
import tf2_geometry_msgs
from threading import Lock

class EstimatorForce(EstimatorBase):

    def __init__(self):
        super(EstimatorForce, self).__init__('force')

        self._sensor = 'force'
        self._tfBuffer = tf2_ros.Buffer()
        self._tflistener = tf2_ros.TransformListener(self._tfBuffer)
        self._flock = Lock()

        self._sub_force = self._sub_force = rospy.Subscriber("/base/sensor_data",WrenchStamped, self.listen_sensor)
        self._force_pub = rospy.Publisher('/gait/force_params', gp, tcp_nodelay=True, queue_size=1024)
        self._force_band = rospy.Publisher('/gait/force_band', PointStamped, tcp_nodelay=True, queue_size=1024)

    def listen_sensor(self,data):
        tf_data = self.transform_sensor(data)
        self._flock.acquire()
        if tf_data:
            self._data.append(tf_data)
        self._flock.release()
        if len(self._data) > 5:
            p = int(len(self._data) * 0.2)
            p = p if p > 2 else 2
            fs = self.est_fs(self._data[-p:])
            if fs > 0.0:
                self._fs = (0.1 * fs + 0.9 * self._fs)

    def transform_sensor(self, data):
        try:
            tf = self._tfBuffer.lookup_transform('fts_base_link','fts_reference_link', rospy.Time().now(), timeout = rospy.Duration(0.1))
            data_tf = tf2_geometry_msgs.do_transform_wrench(data, tf)
            data_tf.header = data.header
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Transformation Data from 'fts_reference_link' to 'fts_base_link' not found")
            return None
    
        
        return data_tf

    def get_window(self):
        step_percentage = self._window_step / self._window_size
        cutoff = len(self._data) * step_percentage
        self._flock.acquire()
        win = np.array(self._data, copy = True)
        if win.shape[0] == 0:
            self._flock.release()
            return None
        else:
            self._data = self._data[int(cutoff):]
            self._flock.release()
            return win
        

    #estimates gate parameters from data
    def gait_estimation(self, window, pipe):
        old_fs = self._fs

        self.get_avg_speed()
        self.set_pose_win()
        self._fs = self.est_fs(window)
        if self._fs == 0.0:
            self._fs = old_fs

        time_stamps =  time_stamps = [w.header.stamp.to_sec() for w in window]
        #use euclidean product of x and y force to be independent of movement direction
        force = np.array([w.wrench.force.x ** 2 + w.wrench.force.y ** 2 for w in window])
        force = np.sqrt(force)

        torque_x = np.array([w.wrench.torque.x for w in window])
        torque_z = np.array([w.wrench.torque.z for w in window])

        params = gp()
        params.header.stamp = rospy.Time.now()
        order = 3
        
        #preprocess data by applying lowpass filter to smoothe and bandpass filter to filter signals outside of gait cadence
        try:
            sensor_data = np.abs(force)
            smoothed_data = prep.smooth_data(sensor_data, self._highcut, self._fs, 0.3)
            bandpassed_data = prep.butter_bandpass_filter(smoothed_data, self._lowcut, self._highcut, self._fs, order)
            tx_smoothed = prep.smooth_data(torque_x, self._highcut, self._fs, 0.3)
            bandpassed_tx = prep.butter_bandpass_filter(tx_smoothed, self._lowcut, self._highcut, self._fs, order)
        except:
            sensor_data = np.abs(force)
            smoothed_data = prep.smooth_data(sensor_data, self._highcut, self._fs, 0.3)
            bandpassed_data = prep.butter_bandpass_filter(smoothed_data, self._lowcut, self._highcut, self._fs, order - 1)
            tx_smoothed = prep.smooth_data(torque_x, self._highcut, self._fs, 0.3)
            bandpassed_tx = prep.butter_bandpass_filter(tx_smoothed, self._lowcut, self._highcut, self._fs, order)

        plt_seq = []

        debug_plt_ind = []
        cad = 0.0

        debugs = []
        #check if robot moved from velocity of robot, if under threshold set cadence to 0
        if all(s < 0.2 for s in self._window_vel):
            params.cadence = 0.0
            params.cadence_avg = 0.0
        else:
            cad,cad_avg, debugs = self._wflc.wflc(bandpassed_data, self._fs, debug_plt_ind, plt_seq, time_stamps)
            leg1_param, leg2_param = self.torque_cadence(time_stamps, bandpassed_tx)
            leg1_param.stride_length = np.sqrt(self._window_vel[0] ** 2 + self._window_vel[1] ** 2 + self._window_vel[2] ** 2) * leg1_param.cadence
            leg2_param.stride_length = np.sqrt(self._window_vel[0] ** 2 + self._window_vel[1] ** 2 + self._window_vel[2] ** 2) * leg2_param.cadence


            params.cadence = cad
            params.cadence_avg = cad_avg

        
        #return result through python multiprocessing pipe
        ret_dict = {}
        ret_dict['/gait/force_params'] = params
        ret_dict['/gait/force_band'] = debugs
        pipe.put(ret_dict)

    #try to estimate cadence through torque
    def torque_cadence(self,time_stamp, data):
        #peak -> vall = Right step, vall -> peak = Left,step
        min_peak_dist = self._fs * (1.0 / self._highcut)

        #since we're using RMS (absolute distance) peak = TO (maximum distance), valley = HS (minimum distance)
        pkwidth = self._fs * (0.1 / self._highcut) if self._fs * (0.1 / self._highcut) > 1.0 else 1.0
        torque_peak, _ = signal.find_peaks(data,distance = min_peak_dist, prominence = 0.2, width = pkwidth)
        torque_valley, _ = signal.find_peaks(-data,distance = min_peak_dist, prominence = 0.2, width = pkwidth)
        leg1_param = leg_params()
        leg2_param = leg_params()
        
        l1_cad = 0.0
        l2_cad = 0.0
        l1_steps = []
        l2_steps = []
        
        for i in range(torque_peak.shape[0]):
            v = next( (x for x in torque_valley if x > torque_peak[i] ), None)
            if v and ((i == torque_peak.shape[0] - 1) or (v < torque_peak[i + 1])):
                l1_steps.append(np.abs(v - torque_peak[i]))
                ##rospy.loginfo("! TORQUE ! p: %d , v: %d, dist %.8f ", torque_peak[i], v, np.abs(v - torque_peak[i]))       
        for i in range(torque_valley.shape[0]):
            p = next( (x for x in torque_peak if x > torque_valley[i] ), None)
            if p and ((i == torque_valley.shape[0] - 1) or (p < torque_valley[i + 1])):
                l2_steps.append(np.abs(p - torque_valley[i]))
                ##rospy.loginfo("! TORQUE ! v: %d , p: %d, dist %.8f ", torque_valley[i], p, np.abs(p - torque_valley[i]))
        T = time_stamp[-1] - time_stamp[0]
        if l1_steps:
            #l1_cad = 1.0 / (sum(l1_steps) / len(l1_steps))
            l1_cad = len(l1_steps) / T
        if l2_steps:
            #l2_cad = 1.0 / (sum(l2_steps) / len(l2_steps))
            l2_cad = len(l2_steps) / T
        
        leg1_param.cadence = l1_cad
        leg2_param.cadence = l2_cad
        
        #rospy.loginfo("!!!! TORQUE TIMESPAN  %.8f ", T)
        #rospy.loginfo("! TORQUE ! l1 cadence is %.8f , l2 cadence is %.8f ", l1_cad, l2_cad)

        return leg1_param, leg2_param






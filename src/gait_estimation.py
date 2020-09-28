#!/usr/bin/env python

from __future__ import division
import rospy
from geometry_msgs.msg import WrenchStamped, Twist, PointStamped, PolygonStamped, Vector3
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from threading import Lock
import numpy as np
import gait_preprocessing as prep
import scipy.signal as signal
import tf2_ros
import tf2_geometry_msgs
import Queue
import detect_peaks as peaks
from gait_parameters_estimation.msg import gait_debug as dbm
from gait_parameters_estimation.msg import gait_performance as gperf
from gait_parameters_estimation.msg import gait_params as gp
from gait_parameters_estimation.msg import leg_params as leg_params
from ipr_helpers.msg import Pose2DStamped
from wflc import WFLC
from leg_tracker.msg import PersonMsg
from scipy.interpolate import CubicSpline
from multiprocessing import Process, Pipe
from multiprocessing import Queue as MQ
from message_filters import ApproximateTimeSynchronizer, Subscriber

class GaitEstimation():

    def __init__(self):

        self._lowcut = 0.5
        self._highcut = 2.5
        self._gait_freq = self._lowcut + 0.25
        self._window_speed = [0.0, 0.0, 0.0]
        self._window_poses = []
        
        
        self._sensor_data = []
        self._leg_data = []
        self._speed_data = []
        self._right_toe = []
        self._left_toe = []
        self._pose_data = []
        self._shoulder_data = []
        self._last_vel_time = rospy.Time.now()

        self._tfBuffer = tf2_ros.Buffer()
        self._tflistener = tf2_ros.TransformListener(self._tfBuffer)
        
        #self._sub_force = rospy.Subscriber("/base/sensor_data",WrenchStamped, self.listen_sensor)
        #self._sub_leg = rospy.Subscriber("/leg_detection/people_msg_stamped",PersonMsg, self.listen_leg_tracker)
        #self._sub_speed = rospy.Subscriber("/base/fts_controller/fts_command",Twist, self.listen_speed)
        #self._sub_rtoe = rospy.Subscriber("/right_toe",PointStamped, self.listen_right_toe)
        #self._sub_ltoe = rospy.Subscriber("/left_toe",PointStamped, self.listen_left_toe)

        
        #used to hold elements to plot to rqt_plot
        self._debug_buffer = {}
        self._rnd = 1
        
        self._force_windows = Queue.Queue()
        self._leg_windows = Queue.Queue()
        self._toe_windows = Queue.Queue()
        self._shoulder_windows = Queue.Queue()
        
        #window size in seconds
        self._window_size = 5.0
        self._buffer_size = 1.0

        #starting values for sampling rates, will be updated once data comes in
        self._force_fs = 80.0
        self._leg_fs = 5.0
        self._toe_fs = 5.0
        self._speed_fs = 50.0
        self._pose_fs = 150.0
        self._shoulder_fs = 6.0
        
        self._lst_peaks = []
        self._lst_valleys = []

        self._first_run = True
        self._data_recv = False
        self._collect_Thread = None
        self._pub_dict = {}

        self._flock = Lock()
        self._llock = Lock()
        self._tlock = Lock()
        self._shlock = Lock()

        #TODO get values from ROSparams
        #w0 start value, freq update, amp update, prefix
        self._wflc_force = WFLC(self._lowcut , 2.5 * (10**-4), 1.5 * (10**-2), topic_prefix = 'force')
        self._wflc_forceH = WFLC(self._lowcut , 2.5 * (10**-4), 1.5 * (10**-2), topic_prefix = 'forceH')
        self._wflc_leg = WFLC(self._lowcut , 4.0 * (10**-3), 2.0 * (10**-2) , topic_prefix = 'leg')
        self._wflc_legH = WFLC(self._lowcut , 4.0 * (10**-3), 2.0 * (10**-2) , topic_prefix = 'legH')
        self._wflc_toe = WFLC(self._lowcut , 6.0 * (10**-3), 6.0 * (10**-2) , topic_prefix = 'toe')
        self._wflc_toeH = WFLC(self._lowcut , 6.0 * (10**-3), 6.0 * (10**-2) , topic_prefix = 'toeH')
        self._wflc_shoulder = WFLC(self._lowcut , 8.0 * (10**-3), 2.0 * (10**-1) , topic_prefix = 'shoulder')
        self._wflc_shoulderH = WFLC(self._lowcut , 8.0 * (10**-3), 2.0 * (10**-1) , topic_prefix = 'shoulderH')
        
        self._pub = rospy.Publisher('/gait/test_data', dbm,tcp_nodelay=True, queue_size=1024)
        self._pub3 = rospy.Publisher('/gait/performance', gperf, tcp_nodelay=True, queue_size=1024)
        self._force_pub = rospy.Publisher('/gait/force_params', gp, tcp_nodelay=True, queue_size=1024)
        self._force_horiz_pub = rospy.Publisher('/gait/force_paramsH', gp, tcp_nodelay=True, queue_size=1024)
        self._leg_pub = rospy.Publisher('/gait/leg_params', gp, tcp_nodelay=True, queue_size=1024)
        self._leg_horiz_pub = rospy.Publisher('/gait/leg_paramsH', gp, tcp_nodelay=True, queue_size=1024)
        self._toe_pub = rospy.Publisher('/gait/toe_params', gp, tcp_nodelay=True, queue_size=1024)
        self._toe_horiz_pub = rospy.Publisher('/gait/toe_paramsH', gp, tcp_nodelay=True, queue_size=1024)

        self._rsh_pub = rospy.Publisher('/right_shoulder',PointStamped, tcp_nodelay=True, queue_size=1024)
        self._lsh_pub = rospy.Publisher('/left_shoulder',PointStamped, tcp_nodelay=True, queue_size=1024)
        self._shoulders_pub = rospy.Publisher('/gait/shoulder_params',gp, tcp_nodelay=True, queue_size=1024)
        self._vel_pub = rospy.Publisher('/gait/win_speed',PointStamped, tcp_nodelay=True, queue_size=1024)

        self._mean_fus = rospy.Publisher('/gait/mean_fus',gp, tcp_nodelay=True, queue_size=1024)
        self._lsq_fus = rospy.Publisher('/gait/lsq_fus',gp, tcp_nodelay=True, queue_size=1024)
        
        self._remap_vel_pub = rospy.Publisher("/base/fts_controller/fts_command",Twist, tcp_nodelay=True, queue_size=1024)
        
        self.init_pub_dict()
        self.register_subs()

    def init_pub_dict(self):
        self._pub_dict['/gait/force_params'] = self._force_pub
        self._pub_dict['/gait/leg_params'] = self._leg_pub
        self._pub_dict['/gait/toe_params'] = self._toe_pub
        self._pub_dict['/gait/shoulder_params'] = self._shoulders_pub
        self._pub_dict['/gait/force_band'] = rospy.Publisher('/gait/force_band', PointStamped, tcp_nodelay=True, queue_size=1024)
        self._pub_dict['/gait/leg_band'] = rospy.Publisher('/gait/leg_band', PointStamped, tcp_nodelay=True, queue_size=1024)
        self._pub_dict['/gait/toe_band'] = rospy.Publisher('/gait/toe_band', PointStamped, tcp_nodelay=True, queue_size=1024)
        self._pub_dict['/gait/shoulder_band'] = rospy.Publisher('/gait/shoulder_band', PointStamped, tcp_nodelay=True, queue_size=1024)
        
    
    def register_subs(self):
        self._sub_force = rospy.Subscriber("/base/sensor_data",WrenchStamped, self.listen_sensor)
        self._sub_leg = rospy.Subscriber("/leg_detection/people_msg_stamped",PersonMsg, self.listen_leg_tracker)
        self._sub_speed = rospy.Subscriber("/base/fts_controller/fts_command",Twist, self.listen_speed)
        #self._sub_rtoe = rospy.Subscriber("/right_toe",PointStamped, self.listen_right_toe)
        #self._sub_ltoe = rospy.Subscriber("/left_toe",PointStamped, self.listen_left_toe)
        self._sub_pose = rospy.Subscriber("/robotrainer/mobile_robot_pose",Pose2DStamped, self.listen_pose)
        self._sub_shoulders = rospy.Subscriber("/human_body_detection/points",PolygonStamped, self.list_shoulders)
        self._sub_veloc_remap = rospy.Subscriber("/base/robotrainer_controllers/base/velocity_output", Vector3, self.remap_velocity)

        self._sub_rtoe_sync = Subscriber("/right_toe",PointStamped)
        self._sub_ltoe_sync = Subscriber("/left_toe",PointStamped)
        self._ats = ApproximateTimeSynchronizer([self._sub_rtoe_sync, self._sub_ltoe_sync], queue_size=5, slop=0.2)
        self._ats.registerCallback(self.toe_sync)

    def toe_sync(self, right_toe, left_toe):
        rospy.loginfo("blablubalablab")
        self._tlock.acquire()
        self._right_toe.append(right_toe)
        self._left_toe.append(left_toe)
        if len(self._right_toe) > 3:
                fs = self.est_fs(self._right_toe[-3:])
                if fs > 0.0:
                    self._toe_fs = (0.1 * fs + 0.9 * self._toe_fs)
        self._tlock.release()
        if not self._data_recv:
                self._collect_Thread = rospy.timer.Timer(rospy.Duration(self._buffer_size), self.collect_data)
                self._data_recv = True
        

    
    #remap velocity Messages from new subscriber (Vector3) to old type (Twist)
    def remap_velocity(self, data):
        
        remapped_vel = Twist()
        remapped_vel.linear.x = data.x
        remapped_vel.linear.y = data.y
        remapped_vel.angular.z = data.z
        
        self._remap_vel_pub.publish(remapped_vel)
        
        
    def listen_sensor(self, data):      
        tf_data = self.transform_sensor(data)
        self._flock.acquire()
        if tf_data:
            self._sensor_data.append(tf_data)
            if not self._data_recv:
                self._collect_Thread = rospy.timer.Timer(rospy.Duration(self._buffer_size), self.collect_data)
                self._data_recv = True
        self._flock.release()
        if len(self._sensor_data) > 5:
            p = int(len(self._sensor_data) * 0.2)
            p = p if p > 2 else 2
            fs = self.est_fs(self._sensor_data[-p:])
            if fs > 0.0:
                self._force_fs = (0.1 * fs + 0.9 * self._force_fs)

    def transform_sensor(self, data):
        try:
            tf = self._tfBuffer.lookup_transform('fts_base_link','fts_reference_link', rospy.Time().now(), timeout = rospy.Duration(0.1))
            data_tf = tf2_geometry_msgs.do_transform_wrench(data, tf)
            data_tf.header = data.header
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("Transformation Data from 'fts_reference_link' to 'fts_base_link' not found")
            return None
    
        
        return data_tf

    def listen_leg_tracker(self, data):
        self._llock.acquire()
        self._leg_data.append(data)
        self._llock.release()
        if not self._data_recv:
                self._collect_Thread = rospy.timer.Timer(rospy.Duration(self._buffer_size), self.collect_data)
                self._data_recv = True
        if len(self._leg_data) > 3:
            fs = self.est_fs(self._leg_data[-3:])
            if fs > 0.0:
                self._leg_fs = (0.1 * fs + 0.9 * self._leg_fs) 

        
    def list_shoulders(self,data):
        try:
            tf = self._tfBuffer.lookup_transform('base_link','camera_body_rgb_optical_frame', rospy.Time().now(), timeout = rospy.Duration(1.0))
            l_shoulder, r_shoulder = self.to_PointStamped(data)
            l_shoulder_tf = tf2_geometry_msgs.do_transform_point(l_shoulder, tf)
            r_shoulder_tf = tf2_geometry_msgs.do_transform_point(r_shoulder, tf)

            l_shoulder_tf.header.stamp = data.header.stamp
            r_shoulder_tf.header.stamp = data.header.stamp

            self._lsh_pub.publish(l_shoulder_tf)
            self._rsh_pub.publish(r_shoulder_tf)

            self._shlock.acquire()
            self._shoulder_data.append( (l_shoulder_tf, r_shoulder_tf) )
            self._shlock.release()

            if not self._data_recv:
                self._collect_Thread = rospy.timer.Timer(rospy.Duration(self._buffer_size), self.collect_data)
                self._data_recv = True
            if len(self._shoulder_data) > 3:
                est_window = [w[0] for w in self._shoulder_data[-3:]]
                fs = self.est_fs(est_window)
                if fs > 0.0:
                    self._shoulder_fs = (0.1 * fs + 0.9 * self._shoulder_fs)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "TF TEST: Transformation Data from 'camera_body_rgb_optical_frame' to 'base_link' not found"
            return None

    def to_PointStamped(self,data):
        #Index Left shoulder is 5, Right is 2
        l_shoulder = PointStamped()
        r_shoulder = PointStamped()


        l_shoulder.point = data.polygon.points[5]
        r_shoulder.point = data.polygon.points[2]

        return l_shoulder, r_shoulder

    def listen_speed(self,data):
        t_now = rospy.Time.now()
        delta_t = (t_now - self._last_vel_time).to_sec()
        fs = 1.0 / delta_t

        self._speed_data.append(data)
        self._speed_fs = 0.05 * fs + 0.95 * self._speed_fs
        self._last_vel_time = t_now

    """def listen_right_toe(self,data):
                    self._tlock.acquire()
                    self._right_toe.append(data)
                    self._tlock.release()
                    if not self._data_recv:
                            self._collect_Thread = rospy.timer.Timer(rospy.Duration(self._buffer_size), self.collect_data)
                            self._data_recv = True
                    if len(self._right_toe) > 3:
                        fs = self.est_fs(self._right_toe[-3:])
                        if fs > 0.0:
                            self._toe_fs = (0.1 * fs + 0.9 * self._toe_fs)
                        
            
                def listen_left_toe(self,data):
                    self._tlock.acquire() 
                    self._left_toe.append(data)
                    self._tlock.release()"""
    
    def listen_pose(self, data):
        self._pose_data.append(data)
        if len(self._pose_data) > 5:
            fs = self.est_fs(self._pose_data[-5:])
            if fs > 0.0:
                self._pose_fs = (0.1 * fs + 0.9 * self._pose_fs)

    #Remove outliers?
    def est_fs(self, window):
        sd_time_stamps = [i.header.stamp for i in window]
        t = np.array(map(lambda s:s.to_sec() ,sd_time_stamps))
        t_dif = [(sd_time_stamps[i+1] - sd_time_stamps[i]).to_sec() for i in range(len(sd_time_stamps)-1)]

        if len(t_dif) == 0:
            return 0.0
        avg = (sum(t_dif)/len(t_dif))
        if avg == 0:
            return 0.0
        return 1.0/avg
    
    def get_avg_speed(self,end, cutoff):
        rospy.loginfo("Speed: array length %d, end %d, cutoff %d", len(self._speed_data), int(end * self._speed_fs), int(cutoff * self._speed_fs))
        speeds = self._speed_data[:int(end * self._speed_fs)]
        self._speed_data = self._speed_data[int(cutoff * self._speed_fs):]
        
        if not speeds:
            return self._window_speed

        x_avg = [s.linear.x for s in speeds]
        x_avg = sum(x_avg) / len(x_avg)
        y_avg = [s.linear.y for s in speeds]
        y_avg = sum(y_avg) / len(y_avg)
        z_avg = [s.angular.z for s in speeds]
        z_avg = sum(z_avg) / len(z_avg)
        
        self._window_speed = [x_avg, y_avg, z_avg]
        
        P = PointStamped()
        P.header.stamp = rospy.Time.now()
        P.point.x = x_avg
        P.point.y = y_avg
        P.point.z = z_avg

        self._vel_pub.publish(P)

    
    def set_pose_win(self):
        buff_size = int(self._pose_fs * self._buffer_size)
        win_size  = int(self._pose_fs * self._window_size)
        
        win = np.array(self._pose_data[:win_size + 2 * buff_size])

        self._pose_data = self._pose_data[buff_size:]
        self._window_poses = win
        self._pose_fs = self.est_fs(self._window_poses)


    def pose_dist(self, pose1, pose2):
        dist_x = pose2.pose.x - pose1.pose.x
        dist_y = pose2.pose.y - pose1.pose.y
        
        return np.sqrt(dist_x ** 2 + dist_y ** 2)
    
    #t1 and t2 must be in seconds
    def pose_dist_t(self, t1, t2):
        t = [p.header.stamp.to_sec() for p in self._window_poses]
        if t1 not in t:
            ind1 = self.closest_node(t1, t)
        else:
            ind1 = t.index(t1)
        if t2 not in t:
            ind2 = self.closest_node(t2, t)
        else:
            ind2 = t.index(t2)
        
        pose1 = self._window_poses[ind1]
        pose2 = self._window_poses[ind2]
        
        return self.pose_dist(pose1, pose2)

    def gait_estimation_shoulders(self, window, send_pipe):
        left_shoulder_w = np.array([w[0].point.y for w in window])
        right_shoulder_w = np.array([w[1].point.y for w in window])

        old_fs = self._shoulder_fs
        self._shoulder_fs = self.est_fs([w[0] for w in window])
        if self._shoulder_fs == 0.0:
            self._shoulder_fs = old_fs

        time_stamps = [w[0].header.stamp.to_sec() for w in window]

        sh_params, debugs = self.gait_params_shoulders(time_stamps, left_shoulder_w, right_shoulder_w)
        if all(abs(s) < 0.2 for s in self._window_speed):
            sh_params.cadence = 0.0
            sh_params.cadence_avg = 0.0
            sh_params.leg1.cadence = 0.0
            sh_params.leg2.cadence = 0.0

        #self._shoulders_pub.publish(sh_params)
        self._shoulder_windows.task_done()
        ret_dict = {}
        ret_dict['/gait/shoulder_params'] = sh_params
        ret_dict['/gait/shoulder_band'] = debugs
        send_pipe.put(ret_dict)
        return [('/gait/shoulder_params', sh_params)]


    def gait_params_shoulders(self,t,lsh, rsh):
        buff_size = int(self._shoulder_fs * self._buffer_size)

        try:
            smoothed_lsh = prep.smooth_data(lsh, self._highcut, self._shoulder_fs)
            smoothed_rsh = prep.smooth_data(rsh, self._highcut, self._shoulder_fs)
            bandpassed_lsh = prep.butter_bandpass_filter(smoothed_lsh, self._lowcut / 2.0, self._highcut, self._shoulder_fs)
            bandpassed_rsh = prep.butter_bandpass_filter(smoothed_rsh, self._lowcut / 2.0, self._highcut, self._shoulder_fs)
        except ValueError as e:
            #rospy.loginfo(e)
            return gp(), []

        plt_seqs = []
        debug_plt = {}

        debugs_1 = []
        sh1_cad, sh1_cad_avg, debugs_1 = self._wflc_shoulder.wflc(bandpassed_lsh, self._shoulder_fs, plt_seqs, debug_plt, t = t)
        sh2_cad, sh2_cad_avg, debugs_2 = self._wflc_shoulderH.wflc(bandpassed_rsh, self._shoulder_fs, plt_seqs, debug_plt, t = t)

        gait_params = gp()
        gait_params.header.stamp = rospy.Time.now()
        gait_params.cadence = (sh1_cad * 2.0 + sh2_cad * 2.0 ) / 2.0
        gait_params.cadence_avg = sh1_cad_avg + sh2_cad_avg
        gait_params.dst = 0.0
        gait_params.leg1.cadence = sh1_cad * 2.0
        gait_params.leg2.cadence = sh2_cad * 2.0
        return gait_params, debugs_1

    def gait_estimation_force(self,window, send_pipe):
        T1 = rospy.Time.now()
        old_fs = self._force_fs
        self._force_fs = self.est_fs(window)
        if self._force_fs == 0.0:
            self._force_fs = old_fs
                
        #rospy.loginfo("POSES : updated fs is %.8f ", self._pose_fs) 
        #self._window_speed = self.get_avg_speed(self._window_size + 2 * self._buffer_size)
        #rospy.loginfo("Calculated avg SPEED for window is x = %.8f, y = %.8f", self._window_speed[0], self._window_speed[1])
        time_stamps = [w.header.stamp.to_sec() for w in window]
        
        force_x = np.array([w.wrench.force.x for w in window])
        force_y = np.array([w.wrench.force.y for w in window])
        
        torque_x = np.array([w.wrench.torque.x for w in window])
        torque_y = np.array([w.wrench.torque.y for w in window])

        force_rms = np.sqrt(force_x ** 2 + force_y ** 2)        

        x_params, x_debugs = self.gait_params_force(0,time_stamps,force_x,torque_x)
        if abs(self._window_speed[0]) < 0.20 and abs(self._window_speed[2]) < 0.20:
            x_params.cadence = 0.0
            x_params.cadence_avg = 0.0
        #self._force_pub.publish(x_params)

        y_params, y_debugs = self.gait_params_force(1,time_stamps,force_y,torque_y)   
        if abs(self._window_speed[1]) < 0.20 and abs(self._window_speed[2]) < 0.20:
            y_params.cadence = 0.0
            y_params.cadence_avg = 0.0     
        #self._force_horiz_pub.publish(y_params)


        #rospy.loginfo("FORCE: force end at : %.8f; difference of %.8f", T2.to_sec(), (T2 - T1).to_sec() )
        
        self._force_windows.task_done()
        if abs(self._window_speed[0]) >= abs(self._window_speed[1]) and abs(self._window_speed[0]) >= abs(self._window_speed[2]) :
            #self._force_pub.publish(x_params)
            ret_dict = {}
            x_params.cadence_fft = 0
            ret_dict['/gait/force_params'] = x_params
            ret_dict['/gait/force_band'] = x_debugs
            send_pipe.put(ret_dict, block = True, timeout = 0.2)
        else:
            #self._force_pub.publish(y_params)
            ret_dict = {}
            y_params.cadence_fft = 1 if abs(self._window_speed[1]) > abs(self._window_speed[2]) else 2
            ret_dict['/gait/force_params'] = y_params
            ret_dict['/gait/force_band'] = y_debugs
            send_pipe.put(ret_dict, block = True, timeout = 0.2)
        
        
        
    def gait_params_force(self,t, force, torque):
        buff_size = int(self._force_fs * self._buffer_size)
        
        k = self._rnd % 3 + 1
        
        
        perf_msg = gperf()
        
        
        #use RMS as per https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5038701/
        #sensor_data = np.array([np.sqrt((x.wrench.force.x)**2 + (x.wrench.force.y)**2 + (x.wrench.force.z)**2) for x in window])   
        #sensor_data = np.array([np.sqrt((x.wrench.force.x)**2 + (x.wrench.force.y)**2)  for x in window])  
        try:
            sensor_data = np.abs(force)
            #sensor_data[np.abs(sensor_data) <= 1.0] = 0.0
            smoothed_data = prep.smooth_data(sensor_data, self._highcut, self._force_fs, win_size = 0.3)
            bandpassed_data = prep.butter_bandpass_filter(smoothed_data, self._lowcut, self._highcut, self._force_fs, 5)
        except ValueError as e:
            #rospy.loginfo("Error when applying filters : %s", e)
            return gp(), []
        
        peak_times, valley_times = self.get_extrema(t,smoothed_data, self._force_fs * (1.0 / self._highcut), 0.5)

        

        #print peaks and valleys to rqt_plot
        #for p in range(smoothed_data[buff_size:-buff_size].shape[0]):
            #pmsg = gperf()
            #pmsg.header.stamp = window[p + buff_size].header.stamp
            #pmsg.wflc_time = (p in peak_indexes) * 5 * k
            #pmsg.window_time = (p in valley_indexes) * (-5) * k
            #self._pub3.publish(pmsg)
        
        
        tohs_cadence = self.cadence_from_spikes(peak_times, valley_times)
        
        #rospy.loginfo("_______ Cadence form TO / HS %.8f", tohs_cadence)
        
        torque_smooothed = prep.smooth_data(torque, self._highcut, self._force_fs, 0.75)
        torque_bandpassed = prep.butter_bandpass_filter(torque_smooothed, self._lowcut, self._highcut, self._force_fs, 5)[buff_size:-buff_size]
        
        torque_peak, torque_valley = self.get_extrema(t[buff_size:-buff_size],torque_bandpassed, self._force_fs * (1.0 / self._highcut), thresh = 0.1, prominence = 0.5)
        
        leg1_param, leg2_param = self.torque_cadence(t, torque_smooothed)
        
        plt_seqs = []

        #creating messages for plotting refactor later
        ##rospy.loginfo("truncated window starts at time: %f ends at time: %f",window[buff_size].header.stamp.to_nsec(), window[-buff_size].header.stamp.to_nsec())
        #for i in range(buff_size, smoothed_data.shape[0] - buff_size):

            #msg = dbm()

            
            #if window[i + buff_size].header.seq not in self._debug_buffer:
                #try:
                    #msg.header = window[i].header
                    #msg.transformed_x = np.sqrt((window[i].wrench.force.x)**2 + (window[i].wrench.force.y)**2 + (window[i].wrench.force.z)**2)
                    #msg.smoothed_x = torque_smooothed[i]
                    #msg.bandpassed_x = torque_bandpassed[i - buff_size]
                    #msg.gait_freq = tohs_cadence
                    #self._debug_buffer[window[i].header.seq] = msg
                    #msg.peak = (window[i].header.stamp.to_sec() in torque_peak) * 5 * k
                    #msg.valley = (window[i].header.stamp.to_sec() in torque_valley) * (-5) * k
                    #plt_seqs.append(window[i].header.seq)
                #except IndexError as e:
                    ##rospy.loginfo("exception adding element i : %d, window_size: %d, seq number: %d" , i, window.shape[0], window[i].header.seq)
                    ##rospy.loginfo("exception is %s ", e)
        
        cad = 0.0
        
        debugs = []
        if axis == 0:
            cad, cad_avg, debugs = self._wflc_force.wflc(bandpassed_data, self._force_fs, debug_plt_ind = plt_seqs, plt_buffer = self._debug_buffer, t = t)
        if axis == 1:
            cad, cad_avg, debugs = self._wflc_forceH.wflc(bandpassed_data, self._force_fs, debug_plt_ind = plt_seqs, plt_buffer = self._debug_buffer, t = t)

        #for key in self._debug_buffer.keys():
            #msg = self._debug_buffer.pop(key,None)
            #if msg:
                #self._pub.publish(msg)
        
        gait_params = gp()
        gait_params.header.stamp = rospy.Time.now()
        gait_params.cadence = cad
        gait_params.cadence_avg = cad_avg
        gait_params.dst = 0.0
        gait_params.leg1 = leg1_param
        gait_params.leg2 = leg2_param
        


        #self._pub3.publish(perf_msg)
        return gait_params, debugs
    
    def gait_estimation_leg(self,window, send_pipe):
        T1 = rospy.Time.now()
        #rospy.loginfo("TIME: Leg start at : %.8f", T1.to_sec())
        old_fs = self._leg_fs
        self._leg_fs = self.est_fs(window)
        if self._leg_fs == 0:
            self._leg_fs = old_fs
        
        time_stamps = [w.header.stamp.to_sec() for w in window]
        
        try:
            buff_size = int(self._leg_fs * self._buffer_size)
            leg1_avg_y = sum([w.leg1.position.y for w in window]) / len(window)
            leg2_avg_y = sum([w.leg2.position.y for w in window]) / len(window)
        except:
            leg1_avg_y = 1
            leg2_avg_y = 0

        # leg1 = right leg, leg2 = left leg
        if leg1_avg_y < leg2_avg_y:
            leg1x = np.array([w.leg1.position.x for w in window])
            leg2x = np.array([w.leg2.position.x for w in window])
            leg1y = np.array([w.leg1.position.y for w in window])
            leg2y = np.array([w.leg2.position.y for w in window])
        else:
            leg1x = np.array([w.leg2.position.x for w in window])
            leg2x = np.array([w.leg1.position.x for w in window])
            leg1y = np.array([w.leg2.position.y for w in window])
            leg2y = np.array([w.leg1.position.y for w in window])
        #x- Axis
        
        
        
        x_params, x_debugs = self.gait_params_leg(0,time_stamps,leg1x, leg2x)
        if abs(self._window_speed[0]) < 0.20 and abs(self._window_speed[2]) < 0.20:
            x_params.cadence = 0.0
            x_params.cadence_avg = 0.0 
        #self._leg_pub.publish(x_params)
        
        #y- Axis
        
        
        
        y_params, y_debugs = self.gait_params_leg(1,time_stamps,leg1y, leg2y)
        if abs(self._window_speed[1]) < 0.20 and abs(self._window_speed[2]) < 0.20:
            y_params.cadence = 0.0
            y_params.cadence_avg = 0.0
        #self._leg_horiz_pub.publish(y_params)
        T2 = rospy.Time.now()
        #rospy.loginfo("TIME: leg end at : %.8f; difference of %.8f", T2.to_sec(), (T2 - T1).to_sec() )
        self._leg_windows.task_done()
        if abs(self._window_speed[0]) >= abs(self._window_speed[1]) and abs(self._window_speed[0]) >= abs(self._window_speed[2]):
            #self._leg_pub.publish(x_params)
            ret_dict = {}
            ret_dict['/gait/leg_params'] = x_params
            ret_dict['/gait/leg_band'] = x_debugs
            send_pipe.put(ret_dict)
        else:
            #self._leg_pub.publish(y_params)
            ret_dict = {}
            ret_dict['/gait/leg_params'] = y_params
            ret_dict['/gait/leg_band'] = y_debugs
            send_pipe.put(ret_dict)
        
    
    def gait_params_leg(self,axis, t, leg1, leg2):
        buff_size = int(self._leg_fs * self._buffer_size)
        
        leg_distance = leg1 - leg2
        try:
            leg1_smoothed = prep.smooth_data(leg1, self._highcut, self._leg_fs, win_size = 0.5)
            leg2_smoothed = prep.smooth_data(leg2, self._highcut, self._leg_fs, win_size = 0.5)
            ld_smoothed = prep.smooth_data(leg_distance, self._highcut, self._leg_fs, win_size = 0.5)
            
            #l1_bandpassed = prep.butter_bandpass_filter(leg1_smoothed, self._lowcut / 2.0, self._highcut, self._leg_fs, 5)[buff_size:-buff_size]
            #l2_bandpassed = prep.butter_bandpass_filter(leg2_smoothed, self._lowcut / 2.0, self._highcut, self._leg_fs, 5)[buff_size:-buff_size]
            ld_bandpassed = prep.butter_bandpass_filter(ld_smoothed, 0.2, self._highcut, self._leg_fs, 5)
            
            min_peak_dist = self._leg_fs * (1.0 / self._highcut) * 2
            l1_peak, l1_valley = self.get_extrema(t, leg1_smoothed, min_peak_dist, width = 10)
            l2_peak, l2_valley = self.get_extrema(t, leg2_smoothed, min_peak_dist, width = 10)
        except ValueError as e:
            #rospy.loginfo("Error when applying filters : %s", e)
            return gp(), []
            
        #rospy.loginfo(">>>>>STRIDE: Leg Strides :")
        leg1_param, leg2_param = self.params_from_extrema(t,axis, leg1_smoothed, leg2_smoothed,prominence = 0.2, pkwidth = 10, min_peak_dist = min_peak_dist)
        
        plt_seqs = []
        debug_plt = {}
        
        #for i in range(buff_size, ld_smoothed.shape[0] - buff_size):

            #msg = dbm()
            #if window[i + buff_size].header.seq not in debug_plt:
                #try:
                    #msg.header = window[i].header
                    #msg.transformed_x = leg_distance[i]
                    #msg.smoothed_x = leg2_smoothed[i]
                    #msg.bandpassed_x = ld_bandpassed[i - buff_size]
                    #debug_plt[window[i].header.seq] = msg
                    #plt_seqs.append(window[i].header.seq)
                #except IndexError as e:
                    ##rospy.loginfo("exception adding element i : %d, window_size: %d, seq number: %d" , i, window.shape[0], window[i].header.seq)
                    ##rospy.loginfo("exception is %s ", e)
        
        leg_cad = 0.0
        debugs = []
        if axis == 0:
            leg_cad, leg_cad_avg, debugs = self._wflc_leg.wflc(ld_bandpassed, self._leg_fs, plt_seqs, debug_plt, t)
        if axis == 1:
            leg_cad, leg_cad_avg, debugs = self._wflc_legH.wflc(ld_bandpassed, self._leg_fs, plt_seqs, debug_plt, t)
            
        if self._window_speed[axis] >= 0:
            l1_to = l1_valley
            l1_hs = l1_peak
            l2_to = l2_valley
            l2_hs = l2_peak
        if self._window_speed[axis] < 0:
            l1_to = l1_peak
            l1_hs = l1_valley
            l2_to = l2_peak
            l2_hs = l2_valley

        dst = 0.0
        if (l1_to.shape[0] > 0) and (l2_to.shape[0] > 0):
            if l1_to[0] < l2_to[0]:
                dst = self.double_support_time(l1_hs,l2_to)
            else:
                dst = self.double_support_time(l2_hs,l1_to)
                
        gait_params = gp()
        gait_params.header.stamp = rospy.Time.now()
        gait_params.cadence = leg_cad * 2.0
        gait_params.cadence_avg = leg_cad_avg * 2.0
        gait_params.dst = dst
        gait_params.leg1 = leg1_param
        gait_params.leg2 = leg2_param
        
        #for key in debug_plt.keys():
            #msg = debug_plt.pop(key,None)
            #if msg:
      
                #self._pub.publish(msg)
        
        return gait_params, debugs
    
    #times must be in seconds
    def closest_node(self,x, times):
        times = np.asarray(times)
        dist_2 = (times - x)**2
        return np.argmin(dist_2)

    def remove_toes(self, smaller, bigger):
        truncated_arr = []
        bigger_stamps = [b.header.stamp.to_sec() for b in bigger]

        for x in smaller:
            closest_ind = self.closest_node(x.header.stamp.to_sec(), bigger_stamps)
            truncated_arr.append(bigger[closest_ind])
        return truncated_arr

    def gait_estimation_toes(self, rtoe_window,ltoe_window, send_pipe):
        T1 = rospy.Time.now()
        #rospy.loginfo("TIME: toe start at : %.8f", T1.to_sec())
        old_fs = self._toe_fs
        self._toe_fs = min(self.est_fs(rtoe_window), self.est_fs(ltoe_window))
        if self._toe_fs == 0.0:
            self._toe_fs = old_fs
        
        #rospy.loginfo("TOE: start")
        #rospy.loginfo("TOE: rsize %d, lsize %d before culling", len(rtoe_window) ,len(ltoe_window))
        rtime = [rt.header.stamp.to_sec() for rt in rtoe_window]
        if len(rtoe_window) < len(ltoe_window):
            ltoe_window = self.remove_toes(rtoe_window, ltoe_window)
            rtime = [rt.header.stamp.to_sec() for rt in rtoe_window]
        elif len(ltoe_window) < len(rtoe_window):
            rtoe_window = self.remove_toes(ltoe_window, rtoe_window)
            rtime = [lt.header.stamp.to_sec() for lt in ltoe_window]
        
        #rospy.loginfo("TOE: rsize %d, lsize %d after culling", len(rtoe_window) ,len(ltoe_window))
        
        
        if min(len(rtoe_window), len(ltoe_window)) < 5:
            return
        
        #rospy.loginfo("TOE: bigger than 5 elems")
        rtx = np.array([rt.point.x for rt in rtoe_window])
        ltx = np.array([lt.point.x for lt in ltoe_window])
        
        #rospy.loginfo("TOE: publish x params")
        x_params, x_debugs = self.gait_params_toe(0,rtime, rtx, ltx)

        if abs(self._window_speed[0]) < 0.20 and abs(self._window_speed[2]) < 0.20:
            x_params.cadence = 0.0
            x_params.cadence_avg = 0.0
        #self._toe_pub.publish(x_params)

        rty = np.array([rt.point.y for rt in rtoe_window])
        lty = np.array([lt.point.y for lt in ltoe_window])
        
        #rospy.loginfo("TOE: publish y params")
        y_params, y_debugs = self.gait_params_toe(1,rtime, rty, lty)
        
        if abs(self._window_speed[1]) < 0.20 and abs(self._window_speed[2]) < 0.20:
            y_params.cadence = 0.0
            y_params.cadence_avg = 0.0
        #self._toe_horiz_pub.publish(y_params)
        T2 = rospy.Time.now()
        #rospy.loginfo("TIME: toe end at : %.8f; difference of %.8f", T2.to_sec(), (T2 - T1).to_sec() )
        self._toe_windows.task_done()
        if abs(self._window_speed[0]) >= abs(self._window_speed[1]) and abs(self._window_speed[0]) >= abs(self._window_speed[2]):
            #self._toe_pub.publish(x_params)
            ret_dict = {}
            ret_dict['/gait/toe_params'] = x_params
            ret_dict['/gait/toe_band'] = x_debugs
            send_pipe.put(ret_dict)
        else:
            #self._toe_pub.publish(y_params)
            ret_dict = {}
            ret_dict['/gait/toe_params'] = y_params
            ret_dict['/gait/toe_band'] = y_debugs
            send_pipe.put(ret_dict)
        return [('/gait/toe_params', x_params), ('/gait/toe_paramsH', y_params)]


    def gait_params_toe(self,axis, t, rtoe, ltoe):


        buff_size = int(self._toe_fs * self._buffer_size)
        
        toe_diff = rtoe - ltoe

        mpd = self._toe_fs * (2.0 / self._highcut)
        pkwidth = self._toe_fs * (0.1 / self._highcut) if self._toe_fs * (0.1 / self._highcut) > 1.0 else 1.0
        try:
            rtoe_smoothed = prep.smooth_data(rtoe, self._highcut, self._toe_fs, win_size = 0.3)
            ltoe_smoothed = prep.smooth_data(ltoe, self._highcut, self._toe_fs, win_size = 0.3)
            td_smoothed = prep.smooth_data(toe_diff, self._highcut, self._toe_fs, win_size = 0.3)
            l1_peak, l1_valley = self.get_extrema(t, rtoe,mpd,  width = pkwidth, prominence = 0.1)
            l2_peak, l2_valley = self.get_extrema(t, ltoe,mpd,  width = pkwidth, prominence = 0.1)
            

            bp_toe_diff = prep.butter_bandpass_filter(td_smoothed, self._lowcut / 2.0, self._highcut, self._toe_fs, 5)
        except ValueError as e:
            #rospy.loginfo(e)
            return gp(), []

        #rospy.loginfo("TOE: to / hs")
        #rospy.loginfo(">>>>>STRIDE: Toe Strides :")
        r_params, l_params = self.params_from_extrema(t,axis, rtoe, ltoe,prominence = 0.1, pkwidth = pkwidth, min_peak_dist = mpd)

        plt_seqs = []
        debug_plt = {}

        toe_cad = 0.0
        debugs = [0]
        if axis == 0:
            #rospy.loginfo("TOE: x wflc")
            toe_cad, toe_cad_avg, debugs = self._wflc_toe.wflc(bp_toe_diff, self._toe_fs, plt_seqs, debug_plt, t = t)
        if axis == 1:
            #rospy.loginfo("TOE: y wflc")
            toe_cad, toe_cad_avg, debugs = self._wflc_toeH.wflc(bp_toe_diff, self._toe_fs, plt_seqs, debug_plt, t = t)


        if self._window_speed[axis] >= 0:
            l1_to = l1_valley
            l1_hs = l1_peak
            l2_to = l2_valley
            l2_hs = l2_peak
        if self._window_speed[axis] < 0:
            l1_to = l1_peak
            l1_hs = l1_valley
            l2_to = l2_peak
            l2_hs = l2_valley

        dst = 0.0
        if (l1_to.shape[0] > 0) and (l2_to.shape[0] > 0):
            if l1_to[0] < l2_to[0]:
                dst = self.double_support_time(l1_hs,l2_to)
            else:
                dst = self.double_support_time(l2_hs,l1_to)

        gait_params = gp()
        gait_params.header.stamp = rospy.Time.now()
        gait_params.cadence = toe_cad * 2.0
        gait_params.cadence_avg = toe_cad_avg * 2.0
        gait_params.dst = dst
        gait_params.leg1 = r_params
        gait_params.leg2 = l_params
        
        #rospy.loginfo("TOE: return axis %d paras", axis)
        return gait_params, debugs

    
    
    def torque_cadence(self,time_stamp, data):
        torque_peak, torque_valley = self.get_extrema(time_stamp, data, self._force_fs * (1.0 / self._highcut), thresh = 0.1,prominence = 0.5, width = 100.0)
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
        
        
        
    
    #leg1 given a x position
    def params_from_extrema(self,time_stamps,axis, leg1 ,leg2, prominence = 0.0, pkwidth = 1.0, min_peak_dist = 1):
        leg1_param = leg_params()
        leg2_param = leg_params()
        
        
        
        l1_peak, l1_valley = self.get_extrema(time_stamps, leg1, min_peak_dist,prominence = prominence, width = pkwidth)
        l2_peak, l2_valley = self.get_extrema(time_stamps, leg2, min_peak_dist,prominence = prominence, width = pkwidth)

        if axis == 0:
            if self._window_speed[0] >= 0:
                l1_to = l1_valley
                l1_hs = l1_peak
                l2_to = l2_valley
                l2_hs = l2_peak
            if self._window_speed[0] < 0:
                l1_to = l1_peak
                l1_hs = l1_valley
                l2_to = l2_peak
                l2_hs = l2_valley
        if axis == 1:
            if self._window_speed[1] >= 0:
                l1_to = l1_valley
                l1_hs = l1_peak
                l2_to = l2_valley
                l2_hs = l2_peak
            if self._window_speed[1] < 0:
                l1_to = l1_peak
                l1_hs = l1_valley
                l2_to = l2_peak
                l2_hs = l2_valley
        #l1_cad = self.cadence_from_spikes(l1_valley, l1_peak)
        #l2_cad = self.cadence_from_spikes(l2_valley, l2_peak)
        
        leg1_param.cadence = self.cadence_leg(time_stamps,l1_hs,l1_to)
        leg2_param.cadence = self.cadence_leg(time_stamps,l2_hs,l2_to)
        
        leg1_param.stride_length, leg1_param.swing_time = self.stride_length(time_stamps, leg1,l1_hs, l1_to) 
        leg2_param.stride_length, leg2_param.swing_time = self.stride_length(time_stamps, leg2,l2_hs, l2_to) 
        
        leg1_param.stride_intervall = self.stride_time(l1_to)
        leg2_param.stride_intervall = self.stride_time(l2_to) 
        
        leg1_param.stance_time = self.stance_duration(l1_hs, l1_to)
        leg2_param.stance_time = self.stance_duration(l2_hs, l2_to)
        
        leg1_param.step_length, leg2_param.step_length = self.step_length(time_stamps,axis, leg1, leg2,min_peak_dist, pkwidth)
        
        ##rospy.loginfo("Calculated V avg %.8f ", sum(V) / len(V) )
        #rospy.loginfo("!!!! ::::::::: legtest cadence leg 1 is %.8f, leg 2 is %.8f combined : %.8f", leg1_param.cadence, leg2_param.cadence, leg1_param.cadence + leg2_param.cadence)
        #rospy.loginfo("!!!! ::::::::: legtest STRIDE leg 1 is %.8f, leg 2 is %.8f ", leg1_param.stride_length, leg2_param.stride_length)
        
        dst = []
        
        return leg1_param, leg2_param
        
        #gp = gperf()
        #gp.header.stamp = rospy.Time.now()
        #gp.wflc_time = l1_stride
        #gp.window_time = l2_stride
        #self._pub3.publish(gp)
    
    #get Cadence for single leg from Laser scanner data 
    def stride_length(self,time_stamps,leg, heel_strike, toe_off):
        strides = []
        swings = []
        avg_swing = 0.0
        avg_stride = 0.0
        #valley  = Toe off, peak = heel strike
        toe_off_ind = [time_stamps.index(to) for to in toe_off]
        heel_strike_ind = [time_stamps.index(hs) for hs in heel_strike]
        
        p2p = [heel_strike[i] - heel_strike[i - 1] for i in range(1,len(heel_strike))]
        v2v = [toe_off[i] - toe_off[i - 1] for i in range(1,len(toe_off))]
        
        #v = index into leg data for valley. same for p
        for v in range(len(toe_off_ind)):
            if any(p > toe_off_ind[v] for p in heel_strike_ind):
                p = next( (x for x in heel_strike_ind if x > toe_off_ind[v] ), None)
                if (v == len(toe_off_ind) - 1) or (p < toe_off_ind[v + 1]):
                    stride = np.abs(leg[toe_off_ind[v]] - leg[p])
                    if stride > 0.1:
                        if len(self._window_poses) > 0:
                            rob_movement = self.pose_dist_t(time_stamps[toe_off_ind[v]], time_stamps[p])
                        else:
                            rob_movement = 0.0
                        #rospy.loginfo("POSES: rob movement was %.8f ", rob_movement)
                        strides.append(np.abs(leg[toe_off_ind[v]] - leg[p]) + rob_movement)
                    swings.append(time_stamps[p] - time_stamps[toe_off_ind[v]])
                    ##rospy.loginfo("! STRIDE ! v: %d , p: %d, dist %.8f ", toe_off_ind[v], p, np.abs(toe_off_ind[v] - leg[p]) )
        ##rospy.loginfo("!!!! ::::::::: legtest valley times list is  %s", v2v)
        ##rospy.loginfo("!!!! ::::::::: legtest peak times  list is  %s", p2p)
        #rospy.loginfo("!!!! ::::::::: >>>>>STRIDE  stride list is  %s", strides)
        
        if swings:
           avg_swing = sum(swings) / len(swings)
        
        if strides:
            avg_stride = sum(strides) / len(strides)
        return avg_stride,avg_swing
    
    #ds2 = Toe off leg 2, ds1 = Heel strike leg 1
    def double_support_time(self,hs_leg1, to_leg2):
        dst = []
        #for i in range(to_leg2.shape[0]):
            #if i < ds1.shape[0]:
                #dst.append(to_leg2[i] - hs_leg1[i])
        for hs in range(len(hs_leg1)):
            if any(to > hs_leg1[hs] for to in to_leg2):
                to = next( (x for x in to_leg2 if x > hs_leg1[hs] ), None)
                if (hs == len(hs_leg1) - 1) or (to < hs_leg1[hs + 1]):
                    dst.append(to - hs_leg1[hs])
        if dst:
            #rospy.loginfo("!!!! ::::::::: >>>>>STANCE double support %.8f", sum(dst) / len(dst) )
            return sum(dst) / len(dst)
        else:
            return 0.0
        
    def step_length(self, time_stamps,axis, leg1, leg2, min_peak_dist,width):
        
        l1_step = 0.0
        l2_step = 0.0
        
        l1_peak, l1_valley = self.get_extrema(time_stamps, leg1,prominence=0.1, mpd = min_peak_dist, width = 5)
        l2_peak, l2_valley = self.get_extrema(time_stamps, leg2,prominence=0.1, mpd = min_peak_dist, width = 5)
        
        if self._window_speed[axis] >= 0:
            l1_to = l1_valley
            l1_hs = l1_peak
            l2_to = l2_valley
            l2_hs = l2_peak
        if self._window_speed[axis] < 0:
            l1_to = l1_peak
            l1_hs = l1_valley
            l2_to = l2_peak
            l2_hs = l2_valley
        

        l1_hs_ind = [time_stamps.index(p) for p in l1_hs]
        l2_hs_ind = [time_stamps.index(p) for p in l2_hs]

         
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
        
        if l1steps:
            l1_step = sum(l1steps) / len(l1steps) 
            #rospy.loginfo("!!!! ::::::::: >>>>>STEP LENGTH L1 %.8f", sum(l1steps) / len(l1steps) )
        if l2steps:
            l2_step = sum(l2steps) / len(l2steps)
            #rospy.loginfo("!!!! ::::::::: >>>>>STEP LENGTH L2 %.8f", sum(l2steps) / len(l2steps) )
            
        return l1_step,l2_step
            
        
    
    def stride_time(self, toe_off):
        strides = [ np.abs(toe_off[i] - toe_off[i - 1]) for i in range(1, toe_off.shape[0]) ]
        
        if not strides:
            return 0.0
        else:
            ##rospy.loginfo("!!!! ::::::::: >>>>>STRIDE  stride list is  %s", strides)
            ##rospy.loginfo("!!!! ::::::::: >>>>>STRIDE  avg time is  %s", sum(strides) / len(strides))
            return sum(strides) / len(strides)

    def stance_duration(self, heel_strike, toe_off):
        duration = 0.0
        durs = []
        
        for p in heel_strike:
            if any(v > p for v in toe_off):
                v = next( (x for x in toe_off if x > p ), None)
                if v:
                    durs.append(v - p)
        
        if durs:
            duration = sum(durs) / len(durs)
            
        #rospy.loginfo("!!!! ::::::::: >>>>>STANCE time %.8f", duration)
        return duration

    def cadence_leg(self, time_stamps, heel_strike, toe_off):
        leg_cad = 0.0
        n_steps = 0.0
        #rospy.loginfo("legtest: number of toe_offs %d, number of heel_strike %d ", len(toe_off), len(heel_strike))
        for v in toe_off:
            if any(p > v for p in heel_strike):
                n_steps += 1.0
        T = time_stamps[-1] - time_stamps[0]
        if n_steps > 0:
            leg_cad = n_steps / T

        return leg_cad
        
        
        
        
    #Return Times of Valleys and Peaks in Window
    def get_extrema(self,x, y, mpd = 1.0, thresh = 0.0, prominence = 0.0, width = 1):
        #peak_indexes = peaks.detect_peaks(y, mpd = mpd, mph = mph_p)
        #valley_indexes = peaks.detect_peaks(y, mpd = mpd, mph = mph_v, valley = True)
        peak_indexes, _ = signal.find_peaks(y,distance = mpd, prominence = prominence, width = width)
        valley_indexes, _ = signal.find_peaks(-y,distance = mpd, prominence = prominence, width = width)
        
        peak_indexes = [p for p in peak_indexes if np.abs(y[p]) > thresh]
        valley_indexes = [v for v in valley_indexes if np.abs(y[v]) > thresh]
        
        peak_times = np.array([x[i] for i in peak_indexes])
        valley_times = np.array([x[i] for i in valley_indexes])
        

        #rospy.loginfo("peak_times has dims %s ", peak_times.shape)
        return peak_times, valley_times
        
    #def filter_extrema(self, peaks, valleys):
        ##find index of overlapping extrema
        #p_ind = 0
        #v_ind = 0
        #filtered_peaks = []
        #filtered_valleys = []
        
        #if peaks[0] > self._lst_peaks:
            #p_ind = len(self._lst_peaks) - 1
        #else:
            #p_ind = next(x[0] for x in enumerate(self._lst_peaks) if x[1] > peaks[0]) - 1
        #if valleys[0] > self._lst_valleys:
            #v_ind = len(self._lst_valleys) - 1
        #else:
            #v_ind = next(x[0] for x in enumerate(self._lst_valleys) if x[1] > valleys[0]) - 1
            
        ##fuse maxes with no valleys between
        #for i in range(len(self._lst_peaks) - p_ind):
            #if len([v for v in (self._lst_valleys + valleys) if v > self._lst_peaks[i + p_ind] and v < peaks[i]]) == 0:
                #new_peak = ((self._lst_peaks[i + p_ind] + peaks[i]) / 2.0)
                #filtered_peaks.append(new_peak)
        #for i in range(len(self._lst_valleys) - v_ind):
            #if len([p for p in (self._lst_peaks + peaks) if p > self._lst_valleys[i + p_ind] and p < valleys[i]]) == 0:
                #new_valleys = ((self._lst_valleys[i + p_ind] + valleys[i]) / 2.0)
        #pass
        
        
    def cadence_from_spikes(self, peaks, valleys):
        #filter valleys before first peak
        #rospy.loginfo("number of peaks : %s, valleys %s", peaks.shape, valleys.shape)
        
        if peaks.size == 0:
            #rospy.loginfo("No peaks detected; skipped")
            return 0

        filtered_valleys = [v for v in valleys if v > peaks[0]]
        diffs = []
        
        for i in range(len(peaks)):
            if i >= len(filtered_valleys):
                break
            peak = peaks[i]
            valley = filtered_valleys[i]
            diff = valley - peak
            
            #only count if peak and valley not too far apart, probably not connected step
            if diff < 2.0 and diff > 0.3:
                diffs.append(diff)
        cad = 0.0
        #rospy.loginfo("found %d diffs in peak detect", len(diffs))
        if len(diffs) == 0:
            cad = 0.0
        else:
            cad = 1.0 / (sum(diffs)/len(diffs))
        if cad < 0:
            #rospy.loginfo("Peaks at %s ",peaks)
            #rospy.loginfo("Valleys at %s ", filtered_valleys)
            rospy.loginfo("calculated diffs %s ", diffs)
        return cad
        
        
        


    def collect_data(self, event):
        rospy.loginfo("Thread test run at %s", rospy.Time.now().to_sec())
        
        # if self._first_run:
        #     rospy.loginfo("First run collecting all availlable Data")
        #     rospy.sleep( rospy.Duration(self._window_size))
        #     self._flock.acquire()
        #     if self._sensor_data:
        #         win_force = np.array(self._sensor_data)
        #         self._sensor_data = []
        #         self._force_windows.put(win_force)
        #     self._flock.release()
        #     self._llock.acquire()
        #     if self._leg_data:
        #         win_leg = np.array(self._leg_data)
        #         self._leg_data = []
        #         self._leg_windows.put(win_leg)
        #     self._llock.release()
        #     self._tlock.acquire()
        #     if self._right_toe and self._left_toe:
        #         rwin = np.array(self._right_toe)
        #         lwin = np.array(self._left_toe)
                
        #         self._right_toe = []
        #         self._left_toe = []
        #         self._toe_windows.put((rwin,lwin))
        #     self._tlock.release()
        #     self._shlock.acquire()
        #     if self._shoulder_data:
        #         shwin = np.array(self._shoulder_data)
                
        #         self._shoulder_data = []
        #         self._shoulder_windows.put(shwin)
        #     self._shlock.release()
        #     self._first_run = False
        if self._first_run:
            rospy.loginfo("First run collecting all availlable Data")
            rospy.sleep(rospy.Duration(self._window_size))
            self._first_run = False
        else:
            step_percentage = self._buffer_size / self._window_size

            fwin_size = int(self._window_size * self._force_fs)
            fbuff_size = int(self._buffer_size * self._force_fs)
            fbuff_size = int(len(self._sensor_data) * step_percentage)
            rospy.loginfo("Force fs is %.8f", self._force_fs)
            rospy.loginfo("leg fs is %.8f", self._leg_fs)
            rospy.loginfo("toe fs is %.8f", self._toe_fs)
            rospy.loginfo("shoulder fs is %.8f", self._shoulder_fs)
            rospy.loginfo("pose fs is %.8f", self._pose_fs)
    
            rospy.loginfo("force measurements %d, buffer size: %d, threshold is %d ", len(self._sensor_data),fbuff_size, fwin_size + 2 * fbuff_size)
            win = []
            self._flock.acquire()
            if len(self._sensor_data) >= 35:
                win = np.asarray(self._sensor_data)
                            
                self._sensor_data = self._sensor_data[fbuff_size:]
                self._force_windows.put(win)
            self._flock.release()

            lwin_size = int(self._window_size * self._leg_fs)
            lbuff_size = int(self._buffer_size * self._leg_fs)
            lbuff_size = int(len(self._leg_data) * step_percentage)
            
            rospy.loginfo("Leg measurements %d, threshold is %d ", len(self._leg_data), lwin_size + 2 * lbuff_size)
            win = []
            self._llock.acquire()
            if len(self._leg_data) >= 35:
                win = np.array(self._leg_data)
                            
                self._leg_data = self._leg_data[lbuff_size:]
                self._leg_windows.put(win)
            self._llock.release()

            twin_size = int(self._window_size * self._toe_fs)
            tbuff_size = int(self._buffer_size * self._toe_fs)
            tbuff_size = int(len(self._right_toe) * step_percentage)
            
            rospy.loginfo("TOE: collect toe data at size right %d, left %d, windowsite %d, buff_size %d ", len(self._right_toe), len(self._left_toe), twin_size, tbuff_size)
            self._tlock.acquire()
            if len(self._right_toe) >= 35 and len(self._left_toe) >= 35:
                rwin = np.asarray(self._right_toe)
                lwin = np.asarray(self._left_toe) 
                rospy.loginfo("TOE: rsize %d, lsize %d collected", len(rwin) ,len(lwin))

                self._right_toe = self._right_toe[tbuff_size:]
                self._left_toe = self._left_toe[tbuff_size:]
                self._toe_windows.put((rwin, lwin))
            self._tlock.release()

            shwin_size = int(self._window_size * self._shoulder_fs)
            shbuff_size = int(self._buffer_size * self._shoulder_fs)
            shbuff_size = int(len(self._shoulder_data) * step_percentage)
            rospy.loginfo("shoulder window size is : %d threshold is %d ", len(self._shoulder_data), shwin_size + 2 * shbuff_size)
            
            self._shlock.acquire()
            if len(self._shoulder_data) >= 35:
                shwin = np.asarray(self._shoulder_data)

                self._shoulder_data = self._shoulder_data[shbuff_size:]
                self._shoulder_windows.put(shwin)
            self._shlock.release()


    def fusion_cadence(self, data):
        #simple mean:

        cadences = [data[k].cadence for k in data.keys()]
        leg1 = np.asarray([data[k].leg1 for k in data.keys() if 'leg' in k or 'toe' in k])
        leg2 = np.asarray([data[k].leg2 for k in data.keys() if 'leg' in k or 'toe' in k])
        #TODO don't forget dst! and the leg stuff

        mean_gp = gp()
        mean_gp.header.stamp = rospy.Time.now()
        mean_gp.cadence = np.mean(np.asarray(cadences))

        self._mean_fus.publish(mean_gp)

        #least squares: z = Hx + b ; H=1.....1
        # x = (HtH)-1 * Ht * z
        #same as mean without weights
        H = np.ones(len(cadences))
        lsq_gp = gp()
        lsq_gp.header.stamp = rospy.Time.now()
        lsq_gp.cadence = (np.dot(H,H)**-1) * np.dot(H,cadences)
        #rospy.loginfo("FUS: leg 1 array has form %s", leg1)





if __name__ == '__main__':
    import sys
    rospy.init_node('gait_estimation', log_level=rospy.INFO)
    
    estimator = GaitEstimation()
    #rospy.loginfo("Start at %s", rospy.Time.now().to_sec())
    
    
    
    proces = []
    pipes = []
    results = {}
    resultsH = {}
    t1 = rospy.Time.now()
    while not rospy.is_shutdown():
        proces = []
        pipes = []
        #Gait Estimation on force data
        t2 = rospy.Time.now()
        if (t2 - t1).to_sec() > 5.0:
            rospy.loginfo("Main Loop")
            t1 = t2
        try:
            rospy.logdebug("Gait: pop from Queue")
            window = estimator._force_windows.get(block = False)
            t = rospy.Time.now()
            if len(window) < 1:
                pass
            else:
                delay = (t - window[-1].header.stamp).to_sec()
                if delay >= 2**30:
                    with estimator._force_windows.mutex:
                        estimator._force_windows.queue.clear()
                    #rospy.loginfo("FORCE: Window was %.8f seconds old, flushin queue", delay)
                else:
                    fqueue = MQ()
                    pf = Process(target = estimator.gait_estimation_force, args = (window,fqueue))
                    proces.append(pf)
                    rospy.loginfo("Starting Force process")
                    pipes.append(fqueue)
                    #paramsf = estimator.gait_estimation_force(window)
                    #results[paramsf[0][0]] = paramsf[0][1]
                    #resultsH[paramsf[1][0]] = paramsf[1][1]
        except Queue.Empty:
            rospy.logdebug("Gait:  Empty Queue")
        #Gait Estimation on legs
        try:
            window = estimator._leg_windows.get(block = False)
            t = rospy.Time.now()
            if len(window) < 1:
                pass
            else:
                delay = (t - window[-1].header.stamp).to_sec()
                if delay >= 2**30:
                    with estimator._leg_windows.mutex:
                        estimator._leg_windows.queue.clear()
                    #rospy.loginfo("Window was %.8f seconds old, flushin queue", delay)
                else:
                    lqueue = MQ() 
                    pl = Process(target = estimator.gait_estimation_leg, args = (window,lqueue))
                    pipes.append(lqueue)
                    proces.append(pl)
                    rospy.loginfo("Starting Leg process")
                    #paramsl = estimator.gait_estimation_leg(window)
                    #results[paramsl[0][0]] = paramsl[0][1]
                    #resultsH[paramsl[1][0]] = paramsl[1][1]
        except Queue.Empty:
            rospy.logdebug("Gait:  Empty Queue")
        try:
            window = estimator._toe_windows.get(block = False)
            t = rospy.Time.now()
            if len(window[0]) < 1:
                pass
            else:
                delay = (t - window[0][-1].header.stamp).to_sec()
                if delay >= 2**30:
                    with estimator._toe_windows.mutex:
                        estimator._toe_windows.queue.clear()
                    #rospy.loginfo("Window was %.8f seconds old, flushin queue", delay)
                elif len(window[0]) > 0 and delay < 2**30:
                    tqueue = MQ()
                    pt = Process(target = estimator.gait_estimation_toes, args = (window[0],window[1],tqueue))
                    pipes.append(tqueue)
                    proces.append(pt)
                    rospy.loginfo("Starting toe process")
                    #paramst = estimator.gait_estimation_toes(*window)
                    #results[paramst[0][0]] = paramst[0][1]
                    #resultsH[paramst[1][0]] = paramst[1][1]
        except Queue.Empty:
            rospy.logdebug("Gait:  Empty Queue")
        try:
            window = estimator._shoulder_windows.get(block = False)
            t = rospy.Time.now()
            if len(window[0]) < 1:
                pass
            else:
                delay = (t - window[-1][0].header.stamp).to_sec()
                if delay >= 2**30:
                    with estimator._shoulder_windows.mutex:
                        estimator._shoulder_windows.queue.clear()
                    #rospy.loginfo("Window was %.8f seconds old, flushin queue", delay)
                elif len(window[0]) > 0 and delay < 2**30:
                    shqueue = MQ() 
                    psh = Process(target = estimator.gait_estimation_shoulders, args = (window,shqueue))
                    pipes.append(shqueue)
                    proces.append(psh)
                    rospy.loginfo("Starting Shoulder process")
                    #paramsh = estimator.gait_estimation_shoulders(window)
                    #results[paramsh[0][0]] = paramsh[0][1]
                    #resultsH[paramsh[0][0]] = paramsh[0][1]
        except Queue.Empty:
            rospy.logdebug("Gait:  Empty Queue")
        #rospy.logdebug("Gait: End Main Thread")
        if results:
            estimator.fusion_cadence(results)
        if len(proces) > 0:
            estimator.get_avg_speed(estimator._window_size + 2 * estimator._buffer_size, estimator._window_size)
            estimator.set_pose_win()
        for p in proces:
            p.start()
            rospy.loginfo("started process")
        #rospy.loginfo("Joining %d processes", len(proces))
        # for pipe in pipes:
        #     if pipe.poll(1.0):
        #         param_dict = pipe.recv()
        #         for k in param_dict.keys():
        #             topic = k
        #             params = param_dict[k]
        #             if not 'band' in k:
        #                 params.header.stamp = stamp
        #                 rospy.loginfo("Got params for topic %s", k)
        #                 estimator._pub_dict[topic].publish(params)
        #             if 'band' in k:
        #                 for p in params:
        #                     estimator._pub_dict[topic].publish(p)
        #     else:
        #         rospy.loginfo("No data on Pipe")
        start = rospy.Time.now()
        stamp = rospy.Time.now()
        while (rospy.Time.now() - start).to_sec() <= 4.0:
            for i in range(len(proces)):
                try:
                    param_dict = pipes[i].get(block = True, timeout = 0.2)
                    for k in param_dict.keys():
                        topic = k
                        params = param_dict[k]
                        if not 'band' in k:
                            params.header.stamp = stamp
                            rospy.loginfo("Got params for topic %s", k)
                            estimator._pub_dict[topic].publish(params)
                        if 'band' in k:
                            rospy.loginfo("number of elements in params %d", len(params))
                            for p in params:
                                estimator._pub_dict[topic].publish(p)
                    proces[i].join()
                except Queue.Empty:
                    rospy.loginfo("Data on pipe not ready yet")           
            if any(p.is_alive() for p in proces):
                rospy.sleep(0.01)
            else:
                break
        else:
            rospy.loginfo("!!:::: Process join timeout")
            for p in proces:
                p.terminate()
                p.join()
        if proces:
            rospy.loginfo("Main loop execute time %.8f ", (rospy.Time.now() - t2).to_sec())
        results = {}
        resultsH = {}
        rospy.sleep(0.01)

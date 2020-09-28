"""
Class to Estimate Gait parameters from shoulder movement
"""

from __future__ import division
from gait_estimator_base import EstimatorBase
from gait_estimator_base import *
from geometry_msgs.msg import PolygonStamped, PointStamped
from threading import Lock
import tf2_ros
import tf2_geometry_msgs

class EstimatorShoulder(EstimatorBase):

    def __init__(self):
        super(EstimatorShoulder, self).__init__('shoulder')

        self._sensor = 'shoulder'
        self._shlock = Lock()
        self._tfBuffer = tf2_ros.Buffer()
        self._tflistener = tf2_ros.TransformListener(self._tfBuffer)

        self._sub_shoulders = rospy.Subscriber("/human_body_detection/points",PolygonStamped, self.list_shoulders)
        self._rsh_pub = rospy.Publisher('/right_shoulder',PointStamped, tcp_nodelay=True, queue_size=1024)
        self._lsh_pub = rospy.Publisher('/left_shoulder',PointStamped, tcp_nodelay=True, queue_size=1024)


    #Listen to point given by body detection and transform to base_link
    def list_shoulders(self,data):
        #messed up stamps again?
        time = rospy.Time.now()
        try:
            tf = self._tfBuffer.lookup_transform('base_link','camera_body_rgb_optical_frame', rospy.Time().now(), timeout = rospy.Duration(1.0))
            l_shoulder, r_shoulder = self.to_PointStamped(data)
            l_shoulder_tf = tf2_geometry_msgs.do_transform_point(l_shoulder, tf)
            r_shoulder_tf = tf2_geometry_msgs.do_transform_point(r_shoulder, tf)

            l_shoulder_tf.header.stamp = time
            r_shoulder_tf.header.stamp = time

            self._lsh_pub.publish(l_shoulder_tf)
            self._rsh_pub.publish(r_shoulder_tf)

            self._shlock.acquire()
            self._data.append( (l_shoulder_tf, r_shoulder_tf) )
            self._shlock.release()

            if len(self._data) > 3:
                est_window = [w[0] for w in self._data[-3:]]
                fs = self.est_fs(est_window)
                if fs > 0.0:
                    self._fs = (0.1 * fs + 0.9 * self._fs)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "TF TEST: Transformation Data from 'camera_body_rgb_optical_frame' to 'base_link' not found"
            return None

    def get_window(self):
        step_percentage = self._window_step / self._window_size
        cutoff = len(self._data) * step_percentage
        self._shlock.acquire()
        win = np.array(self._data, copy = True)
        win_l = np.array([w[0] for w in win])
        win_r = np.array([w[1] for w in win])
        if win.shape[0] == 0:
            self._shlock.release()
            return None
        else:
            self._data = self._data[int(cutoff):]
            self._shlock.release()
            return [win_l, win_r]

    def to_PointStamped(self,data):
        #Index Left shoulder is 5, Right is 2
        l_shoulder = PointStamped()
        r_shoulder = PointStamped()


        l_shoulder.point = data.polygon.points[5]
        r_shoulder.point = data.polygon.points[2]

        return l_shoulder, r_shoulder


    #estimate cadence from shoulder swing in y-direction
    def gait_estimation(self, window, pipe):
        left_shoulder_w = np.array([w[0].point.y for w in window])
        right_shoulder_w = np.array([w[1].point.y for w in window])

        old_fs = self._fs
        self._shoulder_fs = self.est_fs([w[0] for w in window])
        if self._fs == 0.0:
            self._fs = old_fs

        time_stamps = [w[0].header.stamp.to_sec() for w in window]

        #sh_params, debugs = self.gait_params_shoulders(time_stamps, left_shoulder_w, right_shoulder_w)

        order = 3
         #preprocess data by applying lowpass filter to smoothe and bandpass filter to filter signals outside of gait cadence
        try:
            smoothed_lsh = prep.smooth_data(left_shoulder_w, self._highcut, self._fs)
            smoothed_rsh = prep.smooth_data(right_shoulder_w, self._highcut, self._fs)
            bandpassed_lsh = prep.butter_bandpass_filter(smoothed_lsh, self._lowcut / 2.0, self._highcut, self._fs, order)
            bandpassed_rsh = prep.butter_bandpass_filter(smoothed_rsh, self._lowcut / 2.0, self._highcut, self._fs, order)
        except ValueError as e:
            #rospy.loginfo(e)
            smoothed_lsh = prep.smooth_data(left_shoulder_w, self._highcut, self._fs)
            smoothed_rsh = prep.smooth_data(right_shoulder_w, self._highcut, self._fs)
            bandpassed_lsh = prep.butter_bandpass_filter(smoothed_lsh, self._lowcut / 2.0, self._highcut, self._fs, order - 1)
            bandpassed_rsh = prep.butter_bandpass_filter(smoothed_rsh, self._lowcut / 2.0, self._highcut, self._fs, order - 1)

        plt_seqs = []
        debug_plt = {}

        debugs_1 = []
        sh1_cad, sh1_cad_avg, debugs_1 = self._wflc.wflc(bandpassed_lsh, self._fs, plt_seqs, debug_plt, t = time_stamps)
        sh2_cad, sh2_cad_avg, debugs_2 = self._wflcH.wflc(bandpassed_rsh, self._fs, plt_seqs, debug_plt, t = time_stamps)

        gait_params = gp()
        gait_params.header.stamp = rospy.Time.now()
        gait_params.cadence = (sh1_cad * 2.0 + sh2_cad * 2.0 ) / 2.0
        gait_params.cadence_avg = sh1_cad_avg + sh2_cad_avg
        gait_params.dst = 0.0
        gait_params.leg1.cadence = sh1_cad * 2.0
        gait_params.leg2.cadence = sh2_cad * 2.0


        if all(abs(s) < 0.2 for s in self._window_vel):
            gait_params.cadence = 0.0
            gait_params.cadence_avg = 0.0
            gait_params.leg1.cadence = 0.0
            gait_params.leg2.cadence = 0.0

        #self._shoulders_pub.publish(sh_params)

        ret_dict = {}
        ret_dict['/gait/shoulder_params'] = gait_params
        ret_dict['/gait/shoulder_band'] = debugs_1
        pipe.put(ret_dict)

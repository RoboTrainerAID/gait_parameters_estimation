"""
Base Estimator class holding functions that are common to all sensors
"""

from __future__ import division
from abc import ABCMeta, abstractmethod
import gait_preprocessing as prep
import numpy as np
import scipy.signal as signal
import rospy
from wflc import WFLC
import Queue
from geometry_msgs.msg import WrenchStamped, Twist, PointStamped, PolygonStamped, Vector3
from ipr_helpers.msg import Pose2DStamped
from multiprocessing import Process, Pipe
from multiprocessing import Queue as MQ
from gait_parameters_estimation.msg import gait_params as gp
from gait_parameters_estimation.msg import leg_params as leg_params

class EstimatorBase(object):

    __metaclass__ = ABCMeta

    def __init__(self, sensor):

        self._window_size = rospy.get_param('/gait_estimation/window_size')
        self._window_step = rospy.get_param('/gait_estimation/window_step')

        self._lowcut = rospy.get_param('/gait_estimation/'+sensor+'/lowcut')
        self._highcut = rospy.get_param('/gait_estimation/'+sensor+'/highcut')
        self._fs = rospy.get_param('/gait_estimation/'+sensor+'/fs')
        self._pose_fs = rospy.get_param('/gait_estimation/pose_fs')
        self._vel_fs = rospy.get_param('/gait_estimation/vel_fs')
        self._wflc = WFLC(self._lowcut,rospy.get_param('/gait_estimation/'+sensor+'/freq_update'), rospy.get_param('/gait_estimation/'+sensor+'/amp_update'), topic_prefix = sensor)
        self._wflcH = WFLC(self._lowcut,rospy.get_param('/gait_estimation/'+sensor+'/freq_update'), rospy.get_param('/gait_estimation/'+sensor+'/amp_update'), topic_prefix = sensor + 'H')

        self._window_vel = [0,0,0]
        self._window_poses = []
        self._data = []
        self._vel_data = []
        self._pose_data = []
        self._last_vel_time = rospy.Time.now()
        self._data_window = Queue.Queue()

        self._sub_pose = rospy.Subscriber("/robotrainer/mobile_robot_pose",Pose2DStamped, self.listen_pose)
        self._sub_speed = rospy.Subscriber("/base/fts_controller/fts_command",Twist, self.listen_speed)
        self._sub_veloc_remap = rospy.Subscriber("/base/robotrainer_controllers/base/velocity_output", Vector3, self.remap_velocity)

        self._remap_vel_pub = rospy.Publisher("/base/fts_controller/fts_command",Twist, tcp_nodelay=True, queue_size=1024)

        
    #record robot pose and movement
    def listen_pose(self, data):
        self._pose_data.append(data)
        if len(self._pose_data) > 5:
            fs = self.est_fs(self._pose_data[-5:])
            if fs > 0.0:
                self._pose_fs = (0.1 * fs + 0.9 * self._pose_fs)

    #record robot velocity to check for movement
    def listen_speed(self,data):
        t_now = rospy.Time.now()
        delta_t = (t_now - self._last_vel_time).to_sec()
        if delta_t > 0:
            fs = 1.0 / delta_t
            self._vel_fs = 0.05 * fs + 0.95 * self._vel_fs

        self._vel_data.append(data)
        self._last_vel_time = t_now

    #estimate sampling frequency for one window
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

    #estimate speed of robot for window
    def get_avg_speed(self):
        speeds = self._vel_data
        
        
        if not speeds:
            return self._window_vel

        x_avg = [s.linear.x for s in speeds]
        x_avg = sum(x_avg) / len(x_avg)
        y_avg = [s.linear.y for s in speeds]
        y_avg = sum(y_avg) / len(y_avg)
        z_avg = [s.angular.z for s in speeds]
        z_avg = sum(z_avg) / len(z_avg)
        
        self._window_vel = [x_avg, y_avg, z_avg]
        self._vel_data = self._vel_data[int(self._window_step * self._vel_fs):]
        
        P = PointStamped()
        P.header.stamp = rospy.Time.now()
        P.point.x = x_avg
        P.point.y = y_avg
        P.point.z = z_avg

        return self._window_vel

    #remap velocity data to twist for new controller
    def remap_velocity(self, data):
        
        remapped_vel = Twist()
        remapped_vel.linear.x = data.x
        remapped_vel.linear.y = data.y
        remapped_vel.angular.z = data.z
        
        self._remap_vel_pub.publish(remapped_vel)

    def set_pose_win(self):
        win_step = int(self._pose_fs * self._window_step)
        win_size  = int(self._pose_fs * self._window_size)
        
        win = np.array(self._pose_data, copy=True)

        self._pose_data = self._pose_data[win_step:]
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

    def closest_node(self,x, times):
        times = np.asarray(times)
        dist_2 = (times - x)**2
        return np.argmin(dist_2)

    #TO is times in seconds
    def stride_time(self, toe_off):
        strides = [np.abs(toe_off[i] - toe_off[i - 1]) for i in range(1, toe_off.shape[0]) ]
        
        if not strides:
            return 0.0
        else:
            ##rospy.loginfo("!!!! ::::::::: >>>>>STRIDE  stride list is  %s", strides)
            ##rospy.loginfo("!!!! ::::::::: >>>>>STRIDE  avg time is  %s", sum(strides) / len(strides))
            return sum(strides) / len(strides)

    #HS and TO in indexes for leg data
    def stride_length(self,time_stamps,leg, heel_strike, toe_off):
        strides = []
        swings = []
        avg_swing = 0.0
        avg_stride = 0.0
        
        #v = index into leg data for valley. same for p
        for v in range(len(toe_off)):
            if any(p > toe_off[v] for p in heel_strike):
                p = next( (x for x in heel_strike if x > toe_off[v] ), None)
                if (v == len(toe_off) - 1) or (p < toe_off[v + 1]):
                    stride = np.abs(leg[toe_off[v]] - leg[p])
                    if stride > 0.0:
                        if len(self._window_poses) > 0:
                            rob_movement = self.pose_dist_t(time_stamps[toe_off[v]], time_stamps[p])
                        else:
                            rob_movement = 0.0
                        #rospy.loginfo("POSES: rob movement was %.8f ", rob_movement)
                        strides.append(np.abs(leg[toe_off[v]] - leg[p]) + rob_movement)
                    swings.append(time_stamps[p] - time_stamps[toe_off[v]])
                    #rospy.loginfo("! STRIDE ! v: %d , p: %d, dist %.8f ", toe_off[v], p, np.abs(toe_off[v] - leg[p]) )
        ##rospy.loginfo("!!!! ::::::::: legtest valley times list is  %s", v2v)
        ##rospy.loginfo("!!!! ::::::::: legtest peak times  list is  %s", p2p)
        #rospy.loginfo("!!!! ::::::::: >>>>>STRIDE  stride list is  %s", strides)
        if swings:
           avg_swing = sum(swings) / len(swings)
        if strides:
            avg_stride = sum(strides) / len(strides)
        return avg_stride,avg_swing

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



    @abstractmethod
    def get_window(self):
        pass

    @abstractmethod
    def gait_estimation(self, data, pipe):
        pass

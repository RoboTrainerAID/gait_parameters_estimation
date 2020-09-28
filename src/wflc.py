"""
Applies WFLC filter to data
"""


import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from gait_parameters_estimation.msg import gait_params as gp
from std_msgs.msg import Float64

class WFLC(object):

    def __init__(self, w0, freq_update, amp_update, M = 1, weights = [], topic_prefix = ''):
        
        self._w0 = w0
        self._freq_update = freq_update
        self._amp_update = amp_update
        self._M = M
        self._sumw0 = 0.0
        self._topic_prefix = topic_prefix
        
        if not weights:
            self._w = [0] * 2 * M
            self._w[0] = 1.00
            self._w[M] = 0.00
        else:
            if len(weights) == (2 * M):
                self._w = weights
            else:
                raise ValueError('weights List must have Dimension of 2*M, has dimension %d' %( len(weights)) )
            
            
        self._w_hist = [self._w]

        self._pub = rospy.Publisher('/gait/' + topic_prefix + '_freq_test', gp, tcp_nodelay=True, queue_size=1024)
        self._pub_har = rospy.Publisher('/gait/' + topic_prefix +  '_harmonics', Float64, tcp_nodelay=True, queue_size=1024)
        self._pub_band  = rospy.Publisher('/gait/'+ topic_prefix + '_band', PointStamped, tcp_nodelay=True, queue_size=1024)
    
    @property
    def freq_update(self):
        return self._freq_update
    
    @property
    def amp_update(self):
        return self._amp_update
    
    @property
    def w0(self):
        return self._w0
    
    @property
    def weights(self):
        return self._w
    
    @property
    def weight_hist(self):
        return self._w_hist
    
    @freq_update.setter
    def freq_update(self, val):
        self._freq_update = val
        
    @amp_update.setter
    def amp_update(self, val):
        self._amp_update = val


    def reject_outliers(self, data, m=3):
        return data[abs(data - np.mean(data)) < m * np.std(data)]
    
    def wflc(self, data, fs, debug_plt_ind = [], plt_buffer = {}, t = []):
        w0 = (self._w0 * 2 * np.pi) / fs
        sumw0 = 0
        debugs = []
        #rospy.loginfo("data samples: %d ; debug plt %d", data.shape[0], len(debug_plt_ind))

        M = self._M
        x = [0] * 2 * M
        err = 2 ** 10
        w0s = [self._w0]
        errs = []

        for k in range(1, data.shape[0]):
            sumw0 += w0
            sk = 0.0
            for r in range(M):
                x[r] = np.sin( (r+1) * (sumw0) )
                x[M+r] = np.cos( (r+1) * (sumw0) )
                sk += self._w[r] * x[r]
                sk += self._w[M+r] * x[M+r]

            err = data[k] - sk    
            update = (2.0 * self._freq_update * err * sum([(r+1) * (x[M+r]*self._w[r] - x[r]*self._w[M+r]) for r in range(M)]))
            sumup = sum([(r+1) * (x[M+r]*self._w[r] - x[r]*self._w[M+r]) for r in range(M)])

            w0 = w0 + update
            w0 = abs(w0)

            #correct overswing
            highcut = 2.0
            lowcut = 0.7

            if (w0 / (2 * np.pi)) * fs > highcut:
                curr_freq = (w0 / (2 * np.pi)) * fs
                w0 = ((highcut ) * 2 * np.pi) / fs
            if (w0 / (2 * np.pi)) * fs < lowcut:
                curr_freq = (w0 / (2 * np.pi)) * fs
                w0 = ((lowcut ) * 2 * np.pi) / fs

            w0s.append((w0 / (2 * np.pi)) * fs)
            errs.append(err)


            #self._wb += 2 * err * self._bias_update
            if t:
                band = PointStamped()
                band.header.stamp = rospy.Time.now()
                band.point.x = data[k]
                band.point.y = sk
                band.point.z = (w0 / (2 * np.pi)) * fs
                if 'leg' in self._topic_prefix or 'toe' in self._topic_prefix or 'shoulder' in self._topic_prefix:
                    band.point.z *= 2.0
                if 'H' in self._topic_prefix:
                    band.point.z *= -1
                debugs.append(band)
                #band.point.z = sum((self._w))
                #self._pub_band.publish(band)
                #self._pub_har.publish(sk)
            
            """if k % (data.shape[0] / 2) == 0:
                                                    rospy.loginfo("WFLC: xr is : %f xrm : %f sk : %.8f, error: %.8f w0 : %.8f",x[r], x[M+r], sk, err,w0)
                                                    rospy.loginfo("WFLC: wr is %.8f wmr : %.8f update w0: %.8f",self._w[r], self._w[M + r] ,update)"""
            
            for r in range(2 * M):
                self._w[r] = self._w[r] + 2 * self._amp_update * err * x[r]
            

            #rospy.logdebug("WFLC: pub start")
            if debug_plt_ind and plt_buffer:
                seq = debug_plt_ind[k]
                elem = plt_buffer.pop(seq, None)
                if elem:
                    elem.harmonics_x = sk
                    elem.error_x = err
                    #self._pub.publish(elem)
                    #check weight history; if certain number is below threshold give cadence of 0 as output
                    if len(self._w_hist) > (data.shape[0] / 3):
                        #rospy.loginfo("elements in history %d ", len(self._w_hist))
                        w_blw_threshold = [w for w in self._w_hist if (sum(np.abs(w))) <= 0.00]
                        msg = gp()
                        #msg.header.stamp = elem.header.stamp
                        msg.header.stamp = rospy.Time.now()
                        if len(w_blw_threshold) >= len(self._w_hist) * 0.75:
                            msg.cadence = 0
                            #rospy.loginfo("WFLCC: Cadence zero because weights %s", self._w)
                        else:
                            msg.cadence = (w0 / (2 * np.pi)) * fs
                        self._pub.publish(msg)
                        self._w_hist = self._w_hist[-(data.shape[0] / 4):]
                #rospy.logdebug("WFLC: pub end")
                plt_buffer[seq] = elem
            self._w_hist.append(self._w)
        
        self._w0 = (w0 / (2 * np.pi)) * fs
        self._sumw0 = sumw0
        
        w_blw_threshold = [w for w in self._w_hist if (sum(np.abs(w))) < 0.0015]
        if len(w_blw_threshold) >= len(self._w_hist) * 0.75:
            return 0.0, 0.0
            #rospy.loginfo("Cadence zero because weights %s", self._w)
        else:
            w0s_arr = np.array(w0s)
            cutoff = w0s_arr.shape[0] * 0.2
            p = np.percentile(w0s_arr[int(cutoff):], 75)
            bigger_w0s = w0s_arr[w0s_arr >= p]
            avg_freq = np.mean(bigger_w0s)
            last_freq = (w0 / (2 * np.pi)) * fs

            return (last_freq + avg_freq) / 2.0, avg_freq, debugs

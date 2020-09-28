import scipy.signal as signal
import numpy as np
import rospy
import math

#create a butterworth bandpass filter with cutoff frequency [lowcut, highcut]
def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    sos = signal.butter(order, [low,high], analog=False, btype='band', output='sos')
    return sos

def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    filt_ord = math.ceil(order / 2.0)
    try:
        sos = butter_bandpass(lowcut,highcut, fs, filt_ord)
        padl = 3 * (2 * len(sos) + 1 - min((sos[:, 2] == 0).sum(),
                            (sos[:, 5] == 0).sum()))
        if data.shape[0] < padl:
            padl = data.shape[0] - 1 
        y = signal.sosfiltfilt(sos, data, padlen = padl )
    except ValueError as e:
        raise
    return y


#smooth data by applying median filter and lowpass
def smooth_data(data, highcut, fs, win_size = 0.5):
    nq = fs * 0.5
    
    N = int(data.shape[0] * 0.1)
    
    """prefix_pad = data[:N]
    suffix_pad = data[data.shape[0]-N:]
    
    prefix_pad = prefix_pad * 2 - np.flip(prefix_pad)
    suffix_pad = suffix_pad * 2 - np.flip(suffix_pad)
    
    
    input_data = np.append(prefix_pad,data)
    input_data = np.append(input_data,suffix_pad)"""
    
    try:
        smoothed_data = signal.medfilt(data,int(win_size * fs) + int(win_size * fs) % 2 + 1)
        lpf = signal.firwin(int(win_size * fs) + int(win_size * fs) % 2 + 1,(highcut / nq), window='hamming')
        smoothed_data =  signal.convolve(smoothed_data, lpf, mode='same')
    except ValueError as e:
        raise
    return smoothed_data

def interpolated_intercept(x, y1, y2):
    """Find the intercept of two curves, given by the same x data"""

    def intercept(point1, point2, point3, point4):
        """find the intersection between two lines
        the first line is defined by the line between point1 and point2
        the first line is defined by the line between point3 and point4
        each point is an (x,y) tuple.

        So, for example, you can find the intersection between
        intercept((0,0), (1,1), (0,1), (1,0)) = (0.5, 0.5)

        Returns: the intercept, in (x,y) format
        """    

        def line(p1, p2):
            A = (p1[1] - p2[1])
            B = (p2[0] - p1[0])
            C = (p1[0]*p2[1] - p2[0]*p1[1])
            return A, B, -C

        def intersection(L1, L2):
            D  = L1[0] * L2[1] - L1[1] * L2[0]
            Dx = L1[2] * L2[1] - L1[1] * L2[2]
            Dy = L1[0] * L2[2] - L1[2] * L2[0]

            x = Dx / D
            y = Dy / D
            return x,y

        L1 = line([point1[0],point1[1]], [point2[0],point2[1]])
        L2 = line([point3[0],point3[1]], [point4[0],point4[1]])

        R = intersection(L1, L2)

        return R

    idx = np.argwhere(np.diff(np.sign(y1 - y2)) != 0)
    xc, yc = intercept((x[idx], y1[idx]),((x[idx+1], y1[idx+1])), ((x[idx], y2[idx])), ((x[idx+1], y2[idx+1])))
    return xc,yc

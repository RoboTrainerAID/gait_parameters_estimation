#!/usr/bin/env python

from __future__ import division
import rospy
from gait_estimator_base import EstimatorBase
from gait_estimator_base import *
from estimator_force import EstimatorForce
from estimator_leg import EstimatorLegs
from estimator_toe import EstimatorToe
from estimator_shoulders import EstimatorShoulder
from multiprocessing import Process, Pipe
from multiprocessing import Queue as MQ


window_size = rospy.get_param('/gait_estimation/window_size')
window_step = rospy.get_param('/gait_estimation/window_step')
estimators = {}
pub_dicts = {}

force_pub = rospy.Publisher('/gait/force_params', gp, tcp_nodelay=True, queue_size=1024)
legs_pub = rospy.Publisher('/gait/leg_params', gp, tcp_nodelay=True, queue_size=1024)
toe_pub = rospy.Publisher('/gait/toe_params', gp, tcp_nodelay=True, queue_size=1024)
shoulder_pub = rospy.Publisher('/gait/shoulder_params', gp, tcp_nodelay=True, queue_size=1024)


legs_band = rospy.Publisher('/gait/leg_band', PointStamped, tcp_nodelay=True, queue_size=1024)
force_band = rospy.Publisher('/gait/force_band', PointStamped, tcp_nodelay=True, queue_size=1024)
toe_band = rospy.Publisher('/gait/toe_band', PointStamped, tcp_nodelay=True, queue_size=1024)
shoulder_band = rospy.Publisher('/gait/shoulder_band', PointStamped, tcp_nodelay=True, queue_size=1024)

wls_pub = rospy.Publisher('/gait/WLS_params', gp, tcp_nodelay=True, queue_size=1024)
avg_pub = rospy.Publisher('/gait/AVG_params', gp, tcp_nodelay=True, queue_size=1024)

pub_dicts['/gait/force_params'] = force_pub
pub_dicts['/gait/force_band'] = force_band
pub_dicts['/gait/leg_params'] = legs_pub
pub_dicts['/gait/leg_band'] = legs_band
pub_dicts['/gait/toe_params'] = toe_pub
pub_dicts['/gait/toe_band'] = toe_band
pub_dicts['/gait/shoulder_params'] = shoulder_pub
pub_dicts['/gait/shoulder_band'] = shoulder_band

#fuse parameter estimations from each sensor using wls
def WLS(results):
    ws = []
    b = []
    b_avg = []
    l1_strides = []
    l2_strides = []
    l1_swings = []
    l2_swings = []
    l1_steps = []
    l2_steps = []
    l1_stance = []
    l2_stance = []
    dst = []
    l1_stride_intervall = []
    l2_stride_intervall = []
    leg1_cad = []
    leg2_cad = []

    wls_param = gp()
    avg_param = gp()
    total_weight = 0
    for k in results.keys():
        res = results[k]
        w = rospy.get_param('/gait_estimation/'+k+'/var')
        ws.append(w)
        b.append(results[k].cadence_avg)
        b_avg.append(results[k].cadence_avg)
        
        if k == 'legs' or k =='toe':
            #Camera Data from Toe is more accurate in regards to distance measurements
            if k == 'toe':
                weight = 2.0
            else:
                weight = 1.0
            total_weight += weight
            if res.leg1.cadence >= 0.0:
                leg1_cad.append(res.leg1.cadence)
            if res.leg2.cadence >= 0.0:
                leg2_cad.append(res.leg2.cadence)

            if res.leg1.step_length >= 0.0:
                l1_steps.append(res.leg1.step_length * weight)
            if res.leg2.step_length >= 0.0:
                l2_steps.append(res.leg2.step_length * weight)

            if res.leg1.stride_length >= 0.0:
                l1_strides.append(res.leg1.stride_length * weight)
            if res.leg2.stride_length >= 0.0:
                l2_strides.append(res.leg2.stride_length * weight)

            if res.leg1.swing_time >= 0.0:
                l1_swings.append(res.leg1.swing_time * weight)
            if res.leg2.swing_time >= 0.0:
                l2_swings.append(res.leg2.swing_time * weight)

            if res.leg1.stride_intervall >= 0.0:
                l1_stride_intervall.append(res.leg1.stride_intervall * weight)
            if res.leg2.stride_intervall >= 0.0:
                l2_stride_intervall.append(res.leg2.stride_intervall * weight)

            if res.leg1.stance_time >= 0.0:
                l1_stance.append(res.leg1.stance_time * weight)
            if res.leg2.stance_time >= 0.0:
                l2_stance.append(res.leg2.stance_time * weight)

            if res.dst > 0.0:
                dst.append(res.dst)

        wls_param.header.stamp = results[k].header.stamp
        avg_param.header.stamp = results[k].header.stamp
    
    cad = sum([1.0 / weight for weight in ws]) ** (-1) * sum([b[i] / ws[i] for i in range(len(ws))])
    cad_avg = sum([1.0 / weight for weight in ws]) ** (-1) * sum([b_avg[i] / ws[i] for i in range(len(ws))])
    wls_param.cadence = cad
    wls_param.cadence_avg = cad_avg

    if len(l1_strides) > 0:
        wls_param.leg1.stride_length = sum(l1_strides) / total_weight
    if len(l2_strides) > 0:
        wls_param.leg2.stride_length = sum(l2_strides) / total_weight

    if len(l1_steps) > 0:
        wls_param.leg1.step_length = sum(l1_steps) / total_weight
    if len(l2_steps) > 0:
        wls_param.leg2.step_length = sum(l2_steps) / total_weight

    if len(l1_swings) > 0:
        wls_param.leg1.swing_time = sum(l1_swings) / total_weight
    if len(l2_swings) > 0:
        wls_param.leg2.swing_time = sum(l2_swings) / total_weight

    if len(l1_stance) > 0:
        wls_param.leg1.stance_time = sum(l1_stance) / total_weight
    if len(l2_stance) > 0:
        wls_param.leg2.stance_time = sum(l2_stance) / total_weight

    if len(l1_stride_intervall) > 0:
        wls_param.leg1.stride_intervall = sum(l1_stride_intervall) / total_weight
    if len(l1_stride_intervall) > 0:    
        wls_param.leg2.stride_intervall = sum(l2_stride_intervall) / total_weight

    if len(leg1_cad) > 0:
        wls_param.leg1.cadence = sum(leg1_cad) / len(leg1_cad)
    if len(leg2_cad) > 0:
        wls_param.leg2.cadence = sum(leg2_cad) / len(leg2_cad)

    if len(dst) > 0:
        wls_param.dst = sum(dst) / len(dst)
        wls_pub.publish(wls_param)

    avg_param = wls_param
    avg_param.cadence = sum(b) / len(b)
    avg_param.cadence_avg = sum(b_avg) / len(b_avg)

    avg_pub.publish(avg_param)


#periodic thread that starts parameter estimation for each sensor
def collect_data(event):

    t1 = rospy.Time.now()
    processes = []
    pipes = []
    for k in estimators.keys():
        est = estimators[k]
        win = est.get_window()
        if win is None:
            rospy.loginfo("Win is None")
        if win is not None:
            #start gait estimation process for each sensor if data available
            queue = MQ()
            proc = Process(target = est.gait_estimation, args = (win, queue))
            rospy.loginfo("Added %s Process", k)
            pipes.append(queue)
            processes.append(proc)
    for p in processes:
        p.start()
        rospy.loginfo("Started Process")
    start = rospy.Time.now()
    stamp = rospy.Time.now()
    Results = {}
    #join process, abort if one process freezes
    while (rospy.Time.now() - start).to_sec() <= 4.0:
            for i in range(len(processes)):
                try:
                    param_dict = pipes[i].get(block = True, timeout = 0.2)
                    for k in param_dict.keys():
                        topic = k
                        pub = pub_dicts[k]
                        params = param_dict[k]
                        if not 'band' in k:
                            params.header.stamp = stamp
                            rospy.loginfo("Got params for topic %s", k)
                            pub.publish(params)
                            res_key = ''
                            #add result to dictionary used for fusion
                            if 'force' in k:
                                res_key = 'force'
                            if 'leg' in k:
                                res_key = 'legs'
                            if 'toe' in k:
                                res_key = 'toe'
                            if 'shoulder' in k:
                                res_key = 'shoulder'
                            Results[res_key] = params
                        if 'band' in k:
                            #only used for debug purposes
                            rospy.loginfo("number of elements in params %d", len(params))
                            for point in params:
                                pub_dicts[k].publish(point)
                    processes[i].join()
                except Queue.Empty:
                    rospy.loginfo("Data on pipe not ready yet")           
            if any(p.is_alive() for p in processes):
                rospy.loginfo("Process sleep")
                rospy.sleep(0.01)
            else:
                break
    else:
        rospy.loginfo("!! Process join timeout")
        for p in processes:
            p.terminate()
            p.join()
    if processes:
        rospy.loginfo("Main loop execute time %.8f ", (rospy.Time.now() - t1).to_sec())
        WLS(Results)
        







if __name__ == '__main__':
    import sys
    
    rospy.init_node('gait_estimation', log_level=rospy.INFO)
    rospy.get_rostime()
    rospy.get_time()

    #initialize Estimator for each datasource
    rospy.sleep(rospy.Duration(0.25))
    est_force = EstimatorForce()
    est_leg = EstimatorLegs()
    est_toe = EstimatorToe()
    est_sh = EstimatorShoulder()
    #dictionary holding estimator for each source
    estimators['force'] = est_force
    estimators['legs'] = est_leg
    estimators['toe'] = est_toe
    estimators['shoulder'] = est_sh

    

    rospy.sleep(rospy.Duration(window_size))
    rospy.loginfo("Start Timed thread")
    #periodic thread which collects available data, and gives signal to estimate parameters
    rospy.timer.Timer(rospy.Duration(window_step), collect_data)

    while not rospy.is_shutdown():
        rospy.spin()

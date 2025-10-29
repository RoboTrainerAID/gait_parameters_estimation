#!/usr/bin/env python

import rospy
import os
import errno
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from gait_estimation_from_bag import EstimatorToeFromBag, print_metrics

class GaitEstimationFromBagNode:
    def __init__(self):
        rospy.init_node('gait_estimation_from_bag_node')
        
        # Add ROS parameters for input and output folder
        self.input_bag_folder = rospy.get_param('~input_bag_folder', '/default/input/folder/toe')
        self.output_bag_folder = rospy.get_param('~output_bag_folder', '/default/output/folder/gait')

        try:
            os.makedirs(self.output_bag_folder)
            rospy.loginfo("Created output directory: %s", self.output_bag_folder)
        except OSError as e:
            if e.errno != errno.EEXIST:
                rospy.logerr("Failed to create output directory %s: %s", self.output_bag_folder, e)
            # If directory already exists, do nothing.

        # Service to trigger the estimation
        self.srv = rospy.Service('~process', Trigger, self.process_callback)
        
        # Subscriber for the study status string (which is a prefix for the bag file name)
        self.subscriber = rospy.Subscriber(
            "/robotrainer_user_study_manager/study_status",
            String,
            self.study_status_callback,
            queue_size=10
        )
        self.study_status = ""
        
        rospy.loginfo("Gait estimation service is ready.")

    def study_status_callback(self, msg):
        """Callback to store the latest study status string."""
        if self.study_status != msg.data:
            rospy.loginfo("Study status updated to: %s", msg.data)
            self.study_status = msg.data

    def process_callback(self, request):
        """Service callback to run the gait parameter estimation."""
        response = TriggerResponse()

        if not self.study_status:
            rospy.logerr("Service call failed: No study status received yet.")
            response.success = False
            response.message = "No study status received from /robotrainer_user_study_manager/study_status topic."
            return response

        # Find the unique bag file that starts with the study_status string
        matches = [os.path.join(self.input_bag_folder, f) for f in os.listdir(self.input_bag_folder) if f.startswith(self.study_status) and f.endswith(".bag")]
        bag_file_path = matches[0] if len(matches) == 1 else ""

        if not bag_file_path:
            message = "Found {} bag files starting with '{}' in {}. Expected 1.".format(len(matches), self.study_status, self.input_bag_folder)
            rospy.logerr(message)
            response.success = False
            response.message = message
            return response
        
        rospy.loginfo("Trigger received. Starting gait estimation for: %s", bag_file_path)

        # Define output path based on the found bag file
        base_name = os.path.basename(bag_file_path)
        output_name = base_name.replace('toe_output.bag', 'gait_output.bag')
        output_bag_path = os.path.join(self.output_bag_folder, output_name)

        topics = [
            '/tf',
            '/tf_static',
            '/toe_position/left/kalman',
            '/toe_position/right/kalman',
        ]

        estimator = EstimatorToeFromBag(bag_file_path, topics)

        # --- Calculate gait parameters ---
        param_dict = estimator.gait_parameters()

        if not param_dict or '/timestamp' not in param_dict:
            rospy.logerr("Gait parameter estimation failed or returned no data.")
            response.success = False
            response.message = "Gait parameter estimation failed."
            return response

        print_metrics(param_dict)
        
        # --- Write results to a new bag file ---
        estimator.write_to_bag(output_bag_path, param_dict)
        
        rospy.loginfo("Gait estimation successful. Results written to %s", output_bag_path)
        response.success = True
        response.message = "Gait estimation successful."
        return response

if __name__ == '__main__':
    try:
        node = GaitEstimationFromBagNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

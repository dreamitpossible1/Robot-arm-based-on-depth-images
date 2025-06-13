#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# Import service definition and the original (simulated) class
from sagittarius_manipulation.srv import GetDetections, GetDetectionsResponse
from sagittarius_manipulation_lib.vision_analysis_node import VisionAnalysisNode # Assuming class is in lib
# Import custom message definition if used
# from sagittarius_manipulation.msg import DetectedObject # Example message
from geometry_msgs.msg import PoseStamped, Vector3 # Needed if DetectedObject uses them

class VisionServiceNode:
    def __init__(self):
        rospy.init_node('vision_service_node')
        # In a real system, initialize camera connections, model loading etc. here
        self.analyzer = VisionAnalysisNode() # Using the simulated version for now

        self.service = rospy.Service('/get_detections', GetDetections, self.handle_get_detections)
        rospy.loginfo("Vision Service Node Ready (Simulated).")
        rospy.spin()

    def handle_get_detections(self, req):
        rospy.loginfo("Vision Service: Received detection request.")
        response = GetDetectionsResponse()
        try:
            # Call the method to get detections (simulated or real)
            detections_list_dict = self.analyzer.get_detections() # Returns list of dicts

            # Convert the list of dicts to the list of ROS messages defined in the service
            response.detected_objects = []
            for det_dict in detections_list_dict:
                obj_msg = DetectedObject() # Replace with your actual message type if defined
                # Populate obj_msg fields from det_dict
                obj_msg.id = det_dict.get('object_id', '')
                obj_msg.tag = det_dict.get('object_tag', '')
                obj_msg.confidence = det_dict.get('confidence', 0.0)
                # Ensure pose and dimensions are correctly assigned (assuming PoseStamped and Vector3)
                if isinstance(det_dict.get('pose'), PoseStamped):
                     obj_msg.pose = det_dict['pose']
                if isinstance(det_dict.get('dimensions'), Vector3):
                     obj_msg.dimensions = det_dict['dimensions']
                response.detected_objects.append(obj_msg)

            response.success = True
            rospy.loginfo(f"Vision Service: Responding with {len(response.detected_objects)} objects.")

        except Exception as e:
            rospy.logerr(f"Vision Service error: {e}")
            response.success = False
            response.error_message = f"Internal server error: {e}"

        return response

if __name__ == "__main__":
    # Placeholder for the message definition - YOU NEED TO CREATE THIS msg file
    # This is just for the skeleton to run without the actual message definition yet.
    # In a real scenario, this definition comes from the .msg file compilation.
    class DetectedObject: 
         id = ''
         tag = ''
         confidence = 0.0
         pose = PoseStamped()
         dimensions = Vector3()

    VisionServiceNode()
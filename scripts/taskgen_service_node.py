#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# Import service definition and the original generator class
from sagittarius_manipulation.srv import GenerateTaskPlan, GenerateTaskPlanResponse
from sagittarius_manipulation_lib.task_generation import TaskGenerator # Assuming class is in lib
# Import messages needed by the service request/response
# from sagittarius_manipulation.msg import NlpResult, DetectedObject, MotionTargetDetails 
# Placeholder classes if messages aren't defined yet
class NlpResult: pass
class DetectedObject: pass
class MotionTargetDetails: is_valid=False; pose=None; tag=''; name_in_scene=''

class TaskGenServiceNode:
    def __init__(self):
        rospy.init_node('taskgen_service_node')
        self.generator = TaskGenerator()
        # Load params if needed (e.g., reachability check toggle)

        self.service = rospy.Service('/generate_task_plan', GenerateTaskPlan, self.handle_generate_task_plan)
        rospy.loginfo("Task Generation Service Node Ready.")
        rospy.spin()

    def handle_generate_task_plan(self, req):
        rospy.loginfo("TaskGen Service: Received planning request.")
        response = GenerateTaskPlanResponse()

        # --- Reconstruct input data structures ---
        # The request (req) contains fields like nlp_result and vision_detections
        # Convert these (which are ROS messages) back into the format expected 
        # by the original TaskGenerator.generate_tasks method (likely dictionaries).
        # This conversion depends heavily on your message definitions.
        # Example (Simplified):
        nlp_input_dict = {
             'NLU结果': {
                 'intent': req.nlp_result.intent,
                 'action': req.nlp_result.action,
                 'object': req.nlp_result.object,
                 'location': req.nlp_result.location,
                 # Deserialize entities if needed: json.loads(req.nlp_result.entities_json)
             }
        }
        vision_input_list_dict = []
        for detected_obj_msg in req.vision_detections:
             vision_input_list_dict.append({
                 "object_id": detected_obj_msg.id,
                 "object_tag": detected_obj_msg.tag,
                 "pose": detected_obj_msg.pose, # Assumes pose is PoseStamped
                 "dimensions": detected_obj_msg.dimensions, # Assumes dimensions is Vector3
                 "confidence": detected_obj_msg.confidence
             })
        # --- End Reconstruction ---

        try:
            primitive_tasks, target_details_dict, destination_details_dict = \
                self.generator.generate_tasks(nlp_input_dict, vision_input_list_dict)

            if primitive_tasks is None:
                response.success = False
                # Attempt to get a more specific error from the generator if possible
                response.error_message = "Task generation logic failed (check logs)." 
            else:
                response.success = True
                response.primitive_tasks = primitive_tasks

                # --- Populate response details ---
                # Convert the dictionary results back into the ROS message types defined in the service response
                if target_details_dict:
                    response.target_object_details.is_valid = True
                    response.target_object_details.pose = target_details_dict.get('pose') # Assumes Pose type
                    response.target_object_details.tag = target_details_dict.get('tag', '')
                    response.target_object_details.name_in_scene = target_details_dict.get('name_in_scene', '')
                else:
                    response.target_object_details.is_valid = False

                if destination_details_dict:
                     response.destination_details.is_valid = True
                     response.destination_details.pose = destination_details_dict.get('pose') # Assumes Pose type
                     # Add other fields if needed in MotionTargetDetails msg
                else:
                     response.destination_details.is_valid = False
                # --- End Population ---

        except Exception as e:
            rospy.logerr(f"TaskGen Service error: {e}")
            response.success = False
            response.error_message = f"Internal server error: {e}"

        return response

if __name__ == "__main__":
    TaskGenServiceNode()
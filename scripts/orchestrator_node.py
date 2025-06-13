#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import String, Bool # Added Bool for status feedback
import json # For potentially serializing complex data if needed

# Import Service and Action definitions (These need to be created in srv/ and action/ dirs)
# Assuming your package name is 'sagittarius_manipulation'
from sagittarius_manipulation.srv import ParseInstruction, ParseInstructionRequest
from sagittarius_manipulation.srv import GetDetections, GetDetectionsRequest
from sagittarius_manipulation.srv import GenerateTaskPlan, GenerateTaskPlanRequest
from sagittarius_manipulation.action import ExecuteMotionSequence, ExecuteMotionSequenceGoal
# Import custom message for detections if defined (e.g., in msg/DetectedObject.msg)
# from sagittarius_manipulation.msg import DetectedObjectArray 

# Define service and action names (consistent naming is important)
NLP_SERVICE_NAME = '/parse_instruction'
VISION_SERVICE_NAME = '/get_detections'
TASK_GEN_SERVICE_NAME = '/generate_task_plan'
MOTION_ACTION_NAME = '/execute_motion_sequence'

class OrchestratorNode:
    def __init__(self):
        rospy.init_node('orchestrator_node', anonymous=True)
        rospy.loginfo("Orchestrator Node Initializing...")

        # --- Service Clients ---
        rospy.loginfo(f"Waiting for service {NLP_SERVICE_NAME}...")
        rospy.wait_for_service(NLP_SERVICE_NAME)
        self.parse_instruction_client = rospy.ServiceProxy(NLP_SERVICE_NAME, ParseInstruction)
        rospy.loginfo(f"Connected to {NLP_SERVICE_NAME}")

        rospy.loginfo(f"Waiting for service {VISION_SERVICE_NAME}...")
        rospy.wait_for_service(VISION_SERVICE_NAME)
        self.get_detections_client = rospy.ServiceProxy(VISION_SERVICE_NAME, GetDetections)
        rospy.loginfo(f"Connected to {VISION_SERVICE_NAME}")

        rospy.loginfo(f"Waiting for service {TASK_GEN_SERVICE_NAME}...")
        rospy.wait_for_service(TASK_GEN_SERVICE_NAME)
        self.generate_task_plan_client = rospy.ServiceProxy(TASK_GEN_SERVICE_NAME, GenerateTaskPlan)
        rospy.loginfo(f"Connected to {TASK_GEN_SERVICE_NAME}")

        # --- Action Client ---
        rospy.loginfo(f"Waiting for action server {MOTION_ACTION_NAME}...")
        self.motion_action_client = actionlib.SimpleActionClient(MOTION_ACTION_NAME, ExecuteMotionSequence)
        if not self.motion_action_client.wait_for_server(timeout=rospy.Duration(10.0)):
             rospy.logerr(f"Action server {MOTION_ACTION_NAME} not available after 10s.")
             # Handle failure to connect appropriately
             # rospy.signal_shutdown("Action server not available") # Option: shutdown
             # return
        rospy.loginfo(f"Connected to {MOTION_ACTION_NAME}")

        # --- Publishers/Subscribers ---
        self.command_sub = rospy.Subscriber('/text_command', String, self.command_callback, queue_size=1)
        self.status_pub = rospy.Publisher('/orchestrator_status', String, queue_size=10) # For user feedback
        self.busy_pub = rospy.Publisher('/orchestrator_busy', Bool, queue_size=1, latch=True) # Latched status

        self.is_busy = False
        self.update_busy_status(False) # Publish initial state

        rospy.loginfo("Orchestrator Node Ready and listening on /text_command.")

    def update_busy_status(self, busy):
        self.is_busy = busy
        self.busy_pub.publish(Bool(data=busy))
        status_msg = "Busy" if busy else "Idle"
        # Optional: Also publish string status
        # self.status_pub.publish(f"Orchestrator status: {status_msg}") 

    def publish_status(self, message):
        rospy.loginfo(f"Orchestrator Status: {message}")
        self.status_pub.publish(message)

    def command_callback(self, msg):
        """Handles incoming text commands via ROS services and actions."""
        if self.is_busy:
            rospy.logwarn("Orchestrator is busy, ignoring command: '{}'".format(msg.data))
            self.publish_status(f"Busy, ignoring command: {msg.data}")
            return

        self.update_busy_status(True)
        self.publish_status(f"Received command: '{msg.data}'")
        rospy.loginfo("\n=== New Command Received: '{}' ===".format(msg.data))

        try:
            # 1. Parse Instruction (Service Call)
            self.publish_status("Parsing instruction...")
            nlp_request = ParseInstructionRequest(command=msg.data)
            nlp_response = self.parse_instruction_client(nlp_request)

            if not nlp_response.success or nlp_response.clarification_needed:
                error_msg = nlp_response.error_message or "Clarification needed."
                rospy.logwarn(f"NLP parsing failed or needs clarification: {error_msg}")
                self.publish_status(f"NLP Error: {error_msg}")
                self.update_busy_status(False)
                return

            # Convert NLP result from service response (assuming json string or similar)
            # Example: nlp_result_dict = json.loads(nlp_response.result_json)
            # For simplicity, let's assume the service response fields directly match nlp_result keys
            nlp_result_data = nlp_response.parsed_data # Assumes parsed_data is a field in the service response
            rospy.loginfo(f"NLP Result: {nlp_result_data}") # Adjust based on actual service definition

            # 2. Get Vision Context (Service Call)
            self.publish_status("Requesting object detections...")
            vision_request = GetDetectionsRequest() # Request might be empty or contain hints
            vision_response = self.get_detections_client(vision_request)

            if not vision_response.success:
                rospy.logerr("Failed to get detections from vision service.")
                self.publish_status("Vision Error: Failed to get detections.")
                self.update_busy_status(False)
                return

            detected_objects = vision_response.detected_objects # Assumes this is a list of DetectedObject messages
            rospy.loginfo(f"Vision Result: Found {len(detected_objects)} objects.")
            if not detected_objects:
                 rospy.logwarn("Vision system detected no objects.")
                 # Decide how to proceed - maybe task generation can handle this?

            # 3. Generate Task Plan (Service Call)
            self.publish_status("Generating task plan...")
            task_gen_request = GenerateTaskPlanRequest()
            # Populate request - needs careful definition of the service message!
            # Example: Assuming service takes nlp result and detections directly
            task_gen_request.nlp_result = nlp_result_data # This assumes nlp_result_data matches the srv definition
            task_gen_request.vision_detections = detected_objects

            task_gen_response = self.generate_task_plan_client(task_gen_request)

            if not task_gen_response.success:
                error_msg = task_gen_response.error_message or "Failed to generate task plan."
                rospy.logerr(f"Task Generation Failed: {error_msg}")
                self.publish_status(f"Planning Error: {error_msg}")
                self.update_busy_status(False)
                return

            primitive_tasks = task_gen_response.primitive_tasks
            target_obj_details = task_gen_response.target_object_details # Needs definition in srv
            destination_details = task_gen_response.destination_details   # Needs definition in srv

            rospy.loginfo(f"Generated Primitive Tasks: {primitive_tasks}")
            # Log details (consider converting complex objects like PoseStamped to string)
            # rospy.loginfo(f"Target Object Details: {target_obj_details}") 
            # if destination_details.is_valid: # Assuming a flag for validity
            #      rospy.loginfo(f"Destination Details: {destination_details}")


            # 4. Execute Motion Plan (Action Goal)
            self.publish_status(f"Executing sequence: {primitive_tasks}")
            goal = ExecuteMotionSequenceGoal()
            goal.primitive_tasks = primitive_tasks
            goal.target_object = target_obj_details # Assumes action goal field matches srv field type
            goal.destination = destination_details   # Assumes action goal field matches srv field type

            # Send goal with callbacks for feedback and completion
            self.motion_action_client.send_goal(
                goal,
                done_cb=self.motion_done_callback,
                active_cb=self.motion_active_callback,
                feedback_cb=self.motion_feedback_callback
            )
            rospy.loginfo("Motion goal sent to action server.")
            # Note: The callback will handle setting busy to False upon completion/failure

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            self.publish_status(f"Error: Service call failed - {e}")
            self.update_busy_status(False)
        except Exception as e:
            rospy.logerr(f"Orchestrator: An error occurred: {e}")
            self.publish_status(f"Error: Orchestrator failed - {e}")
            import traceback
            traceback.print_exc()
            self.update_busy_status(False)

    # --- Action Client Callbacks ---
    def motion_active_callback(self):
        rospy.loginfo("Motion action goal now active.")
        self.publish_status("Motion sequence started.")

    def motion_feedback_callback(self, feedback):
        # Feedback message defined in ExecuteMotionSequence.action
        rospy.loginfo(f"Motion feedback: {feedback.current_status}")
        self.publish_status(f"Motion: {feedback.current_status}")

    def motion_done_callback(self, terminal_state, result):
        # Result message defined in ExecuteMotionSequence.action
        rospy.loginfo(f"Motion action finished with state: {actionlib.GoalStatus.to_string(terminal_state)}")
        if terminal_state == actionlib.GoalStatus.SUCCEEDED and result.success:
            rospy.loginfo("Orchestrator: Task sequence execution successful.")
            self.publish_status("Task completed successfully.")
        else:
            error_msg = result.error_message or "Motion execution failed."
            rospy.logerr(f"Orchestrator: Task sequence execution failed. Reason: {error_msg}")
            self.publish_status(f"Task Failed: {error_msg}")
        
        self.update_busy_status(False) # Release busy state
        rospy.loginfo("=== Command Processing Finished (via action callback) ===")


    def run(self):
        """Keeps the node running."""
        rospy.spin()
        rospy.loginfo("Orchestrator Node Shutting Down.")

if __name__ == '__main__':
    try:
        orchestrator = OrchestratorNode()
        orchestrator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Failed to start OrchestratorNode: {e}")
        import traceback
        traceback.print_exc()
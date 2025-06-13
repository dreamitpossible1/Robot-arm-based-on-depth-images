#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
# Import action definition and the original executor class
from sagittarius_manipulation.action import ExecuteMotionSequence, ExecuteMotionSequenceFeedback, ExecuteMotionSequenceResult
from sagittarius_manipulation_lib.motion_executor import MotionExecutor # Assuming class is in lib
# Import messages used in action definition (e.g., MotionTargetDetails)
# from sagittarius_manipulation.msg import MotionTargetDetails 
# Placeholder class if message isn't defined yet
class MotionTargetDetails: is_valid=False; pose=None; tag=''; name_in_scene=''

# <<<--- Placeholder for Gripper Control --->>>
# Replace with your actual gripper control function/class
def command_gripper(action): # action could be 'open' or 'close'
    rospy.loginfo(f"SIMULATING GRIPPER: {action.upper()}")
    # Example: Publish to a topic
    # gripper_pub = rospy.Publisher('/gripper_command', String, queue_size=1)
    # gripper_pub.publish(action)
    # rospy.sleep(1.0) # Wait for gripper action
    # Or call a service:
    # try:
    #     gripper_service = rospy.ServiceProxy('/control_gripper', SetBool) # Example service type
    #     resp = gripper_service(data=(action == 'close')) # True for close, False for open
    #     rospy.sleep(1.0)
    # except rospy.ServiceException as e:
    #     rospy.logerr(f"Gripper service call failed: {e}")
    #     return False
    return True # Indicate success/failure
# <<<-------------------------------------->>>


class MotionActionServer:
    def __init__(self):
        self._node_name = 'motion_action_server'
        rospy.init_node(self._node_name)

        # Initialize the original MotionExecutor (loads params, connects to MoveIt)
        # Ensure MoveIt is initialized *before* the action server starts accepting goals
        # MotionExecutor init already handles MoveIt init if needed
        self.executor = MotionExecutor() 

        # Create the Action Server
        self._action_server = actionlib.SimpleActionServer(
            MOTION_ACTION_NAME, # Use the same name as defined in orchestrator
            ExecuteMotionSequence,
            execute_cb=self.execute_callback,
            auto_start=False # Start manually after setup
        )
        self._action_server.start()
        rospy.loginfo(f"{self._node_name} Ready.")
        rospy.spin()

    def execute_callback(self, goal):
        rospy.loginfo("Motion Action Server: Received goal.")
        feedback = ExecuteMotionSequenceFeedback()
        result = ExecuteMotionSequenceResult()

        primitive_tasks = goal.primitive_tasks
        target_object_details_msg = goal.target_object # This is a ROS msg
        destination_details_msg = goal.destination     # This is a ROS msg

        # --- Convert ROS Msg details back to dictionary for existing executor ---
        # This depends on your message definitions matching the executor's expectations
        target_obj_dict = None
        if target_object_details_msg.is_valid:
             target_obj_dict = {
                 'pose': target_object_details_msg.pose, # Assumes Pose type
                 'tag': target_object_details_msg.tag,
                 'name_in_scene': target_object_details_msg.name_in_scene
             }

        destination_dict = None
        if destination_details_msg.is_valid:
             destination_dict = {
                 'pose': destination_details_msg.pose # Assumes Pose type
             }
        # --- End Conversion ---

        if not target_obj_dict:
             result.success = False
             result.error_message = "Invalid target object details in goal."
             rospy.logerr(result.error_message)
             self._action_server.set_aborted(result)
             return

        # --- Execute Sequence with Feedback and Preemption Check ---
        overall_success = True
        # Add target object to scene before starting
        if not self.executor.add_object_to_scene(
             target_obj_dict['name_in_scene'],
             target_obj_dict['pose'],
             target_obj_dict['tag']):
             result.success = False
             result.error_message = "Failed to add target object to planning scene."
             rospy.logerr(result.error_message)
             self._action_server.set_aborted(result)
             return

        for task in primitive_tasks:
            # Check if preemption is requested
            if self._action_server.is_preempt_requested():
                rospy.loginfo("Motion Action Server: Preempted.")
                self._action_server.set_preempted()
                overall_success = False
                result.error_message = "Task preempted by user."
                # Clean up scene? Go home?
                # self.executor.remove_object_from_scene(target_obj_dict['name_in_scene']) # Example cleanup
                # if self.executor.attached_object_name: self.executor.detach_object()
                break # Exit the loop

            rospy.loginfo(f"Executing primitive: {task}")
            feedback.current_status = f"Executing: {task}"
            self._action_server.publish_feedback(feedback)

            # --- Integrate Gripper Control ---
            task_success = False
            if task == "grasp":
                 # 1. Move to grasp pose (handled by execute_primitive_task)
                 task_success = self.executor.execute_primitive_task(task, target_obj_dict, destination_dict)
                 # 2. Command gripper close *if* move was successful
                 if task_success:
                      task_success = command_gripper('close')
                      if not task_success:
                          result.error_message = "Gripper close command failed."
                          # Optional: try to detach object in scene if attach succeeded but gripper failed
                          if self.executor.attached_object_name:
                               self.executor.detach_object() 
                      else:
                          # If attach failed in execute_primitive_task but gripper closed, still a failure state
                          task_success = (self.executor.attached_object_name == target_obj_dict['name_in_scene'])
                          if not task_success: result.error_message = "Attach object failed in planning scene after grasp."

            elif task == "release":
                 # 1. Command gripper open
                 task_success = command_gripper('open')
                 # 2. Detach in planning scene *if* gripper opened successfully
                 if task_success:
                      task_success = self.executor.execute_primitive_task(task, target_obj_dict, destination_dict)
                      if not task_success: result.error_message = "Detach object failed in planning scene after release."
                 else:
                     result.error_message = "Gripper open command failed."

            else:
                # For other tasks, call the original executor directly
                task_success = self.executor.execute_primitive_task(task, target_obj_dict, destination_dict)
            # --- End Gripper Integration ---


            if not task_success:
                rospy.logerr(f"Motion Action Server: Failed to execute primitive task '{task}'. Aborting sequence.")
                overall_success = False
                if not result.error_message: # Set default error if not set by grasp/release
                     result.error_message = f"Failed primitive: {task}"
                break # Exit the loop

            rospy.sleep(0.5) # Short pause between primitives

        # --- Cleanup Scene ---
        # Remove the temporary planning scene object representation *only if*
        # the sequence failed *before* placing it, or if the task was just 'pick'.
        # If sequence succeeded for 'place', the object should logically remain (though maybe not in planning scene).
        # This cleanup needs careful thought based on desired final state.
        object_name_in_scene = target_obj_dict['name_in_scene']
        if self.executor.attached_object_name: # Failed while holding? Or just picked?
             rospy.logwarn(f"Sequence finished/aborted while object '{self.executor.attached_object_name}' still attached.")
             # Optionally try to detach or go home
             # command_gripper('open')
             # self.executor.detach_object()
        elif overall_success and 'release' in primitive_tasks:
             rospy.loginfo("Object successfully placed. Keeping it conceptually in the environment.")
             # Should it remain in the planning scene? Depends if we might interact again soon.
             # self.executor.remove_object_from_scene(object_name_in_scene) # Option: remove from scene
        else: # Failed before placing or never picked it up successfully
             rospy.loginfo(f"Removing '{object_name_in_scene}' from planning scene after failed/incomplete sequence.")
             self.executor.remove_object_from_scene(object_name_in_scene)
        # --- End Cleanup ---

        # --- Set final action result ---
        result.success = overall_success
        if overall_success:
            rospy.loginfo("Motion Action Server: Goal executed successfully.")
            self._action_server.set_succeeded(result)
        else:
            rospy.logerr(f"Motion Action Server: Goal failed. Reason: {result.error_message}")
            self._action_server.set_aborted(result) # Use aborted for failures during execution

if __name__ == '__main__':
    MotionActionServer()
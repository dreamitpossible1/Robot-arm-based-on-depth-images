#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion, Vector3 # Ensure imports
import std_msgs.msg
from math import pi
import copy

class MotionExecutor:
    def __init__(self):
        """Initialize MoveIt! Commander and load parameters."""
        # Load parameters from ROS Parameter Server
        self.planning_group = rospy.get_param('/robot/planning_group', 'sagittarius_arm')
        self.reference_frame = rospy.get_param('/robot/reference_frame', 'base_link')
        self.eef_link = rospy.get_param('/robot/eef_link', 'link_6')
        self.object_db = rospy.get_param('/object_db', {})
        self.approach_distance = rospy.get_param('/motion_executor/approach_distance', 0.10)
        self.lift_distance = rospy.get_param('/motion_executor/lift_distance', 0.10)
        self.retreat_distance = rospy.get_param('/motion_executor/retreat_distance', 0.10)
        goal_pos_tol = rospy.get_param('/motion_executor/goal_pos_tolerance', 0.01)
        goal_ori_tol = rospy.get_param('/motion_executor/goal_ori_tolerance', 0.05)
        planning_time = rospy.get_param('/motion_executor/planning_time', 5.0)

        # Initialize MoveIt only if not already done
        if not moveit_commander.roscpp_is_initialized():
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.loginfo("MoveIt Commander Initialized by MotionExecutor.")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.planning_group)

        self.move_group.set_pose_reference_frame(self.reference_frame)
        self.move_group.set_goal_position_tolerance(goal_pos_tol)
        self.move_group.set_goal_orientation_tolerance(goal_ori_tol)
        self.move_group.allow_replanning(True)
        self.move_group.set_planning_time(planning_time)

        self.attached_object_name = None # Track attached object by its name in the scene

        rospy.loginfo("Motion Executor Ready.")
        rospy.loginfo("Planning Group: {}".format(self.move_group.get_name()))
        rospy.loginfo("Reference Frame: {}".format(self.reference_frame))

    def _get_object_dimensions(self, object_tag):
        """Lookup object dimensions from loaded config."""
        dims_list = self.object_db.get(object_tag, self.object_db.get('default', [0.05]*3))
        return Vector3(x=dims_list[0], y=dims_list[1], z=dims_list[2])

    def add_object_to_scene(self, object_name, object_pose, object_tag):
        """Adds or updates object in MoveIt planning scene."""
        rospy.loginfo("Adding/Updating object '{}' (tag: {}) in planning scene.".format(object_name, object_tag))
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.reference_frame
        pose_stamped.pose = object_pose
        dimensions = self._get_object_dimensions(object_tag)

        self.scene.remove_world_object(object_name)
        rospy.sleep(0.5)
        self.scene.add_box(object_name, pose_stamped, size=tuple(self.object_db.get(object_tag, self.object_db['default']))) # Use tuple for size
        rospy.loginfo("Object '{}' added/updated.".format(object_name))
        rospy.sleep(1.0)
        return True

    def remove_object_from_scene(self, object_name):
        """Removes object from the planning scene."""
        if self.attached_object_name == object_name:
            rospy.logwarn("Cannot remove object '{}' as it is attached.".format(object_name))
            return
        rospy.loginfo("Removing object '{}' from planning scene.".format(object_name))
        self.scene.remove_world_object(object_name)
        rospy.sleep(1.0)

    def attach_object(self, object_name, object_tag):
        """Attaches object to EEF."""
        if self.attached_object_name:
            rospy.logwarn("Cannot attach '{}', already holding '{}'".format(object_name, self.attached_object_name))
            return False
        rospy.loginfo("Attaching object '{}' (tag: {}) to EEF '{}'".format(object_name, object_tag, self.eef_link))
        touch_links = self.robot.get_link_names(group=self.eef_link)
        if not touch_links: touch_links = [self.eef_link]

        if object_name not in self.scene.get_known_object_names():
            rospy.logwarn("Object '{}' not in scene, cannot attach. Add it first.".format(object_name))
            return False

        self.scene.attach_box(self.eef_link, object_name, touch_links=touch_links)
        self.attached_object_name = object_name
        rospy.sleep(1.0)
        return True

    def detach_object(self):
        """Detaches the currently held object."""
        if not self.attached_object_name:
            rospy.logwarn("Cannot detach, no object is attached.")
            return False
        rospy.loginfo("Detaching object '{}' from EEF '{}'".format(self.attached_object_name, self.eef_link))
        object_to_detach = self.attached_object_name
        self.scene.remove_attached_object(self.eef_link, name=object_to_detach)
        self.attached_object_name = None
        rospy.sleep(1.0)
        # Note: Object remains in the world at the detached location
        return True

    def go_to_pose_goal(self, pose_stamped):
        """Plans and executes motion to a target pose."""
        self.move_group.set_pose_target(pose_stamped)
        rospy.loginfo("MotionExecutor: Planning and executing to pose...")
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if success:
            rospy.loginfo("MotionExecutor: Movement successful.")
        else:
            rospy.logerr("MotionExecutor: Movement failed!")
        return success
        
    def go_home(self):
        """Moves the robot to a predefined home position."""
        home_config = rospy.get_param('/predefined_locations/home', None)
        if home_config and 'joint_values' in home_config:
             rospy.loginfo("Moving to home joint configuration.")
             self.move_group.set_joint_value_target(home_config['joint_values'])
             success = self.move_group.go(wait=True)
             self.move_group.stop()
             self.move_group.clear_pose_targets()
             return success
        else:
             rospy.logwarn("Home position not defined or invalid in config.")
             return False


    def execute_primitive_task(self, task, target_obj_details, destination_details):
        """Executes a single primitive task."""
        rospy.loginfo("--- MotionExecutor: Executing Primitive Task: {} ---".format(task))
        target_pose_stamped = PoseStamped()
        target_pose_stamped.header.frame_id = self.reference_frame
        target_pose_stamped.header.stamp = rospy.Time.now()

        current_pose = self.move_group.get_current_pose().pose
        object_pose = target_obj_details['pose']
        object_tag = target_obj_details['tag']
        object_name_in_scene = target_obj_details['name_in_scene']

        calculated_target = None

        # --- Calculate Target Pose ---
        if task == "approach_pickup":
            calculated_target = copy.deepcopy(object_pose)
            calculated_target.position.z += self.approach_distance
            # TODO: Set appropriate orientation for approach

        elif task == "grasp":
            calculated_target = copy.deepcopy(object_pose)
            # Optional: Adjust Z, set grasp orientation
            target_pose_stamped.pose = calculated_target
            if not self.go_to_pose_goal(target_pose_stamped): return False
            return self.attach_object(object_name_in_scene, object_tag)

        elif task == "lift":
            if not self.attached_object_name:
                rospy.logerr("Lift failed: Object not attached.")
                return False
            calculated_target = copy.deepcopy(current_pose)
            calculated_target.position.z += self.lift_distance

        elif task == "move_to_release_approach":
            if not self.attached_object_name:
                rospy.logerr("Move to release failed: Object not attached.")
                return False
            if not destination_details:
                rospy.logerr("Move to release failed: Destination details not provided.")
                return False
            destination_pose = destination_details['pose']
            calculated_target = copy.deepcopy(destination_pose)
            calculated_target.position.z += self.approach_distance # Approach above destination

        elif task == "release":
            if not self.attached_object_name:
                rospy.logwarn("Release called but no object attached.")
                return True
            # Assumes arm is already at the release location (e.g., after move_to_release_approach)
            return self.detach_object()

        elif task == "retreat_after_release" or task == "retreat_after_pickup":
             # Simple retreat: move up from current pose
             calculated_target = copy.deepcopy(current_pose)
             calculated_target.position.z += self.retreat_distance

        else:
            rospy.logerr("MotionExecutor: Unknown primitive task '{}'".format(task))
            return False

        # --- Execute Movement ---
        if calculated_target and task not in ["grasp", "release"]:
             target_pose_stamped.pose = calculated_target
             return self.go_to_pose_goal(target_pose_stamped)
        elif task not in ["grasp", "release"]:
             rospy.logwarn("No target calculated for primitive task '{}'".format(task))
             return False
        else:
             # Grasp/Release return status directly
             return True # If logic flowed here, means grasp/release returned True earlier

    def run_primitive_task_sequence(self, primitive_tasks, target_object_details, destination_details):
        """Executes a sequence of primitive tasks."""
        rospy.loginfo("MotionExecutor: Starting primitive task sequence: {}".format(primitive_tasks))

        object_name_in_scene = target_object_details['name_in_scene']
        self.add_object_to_scene(object_name_in_scene,
                                target_object_details['pose'],
                                target_object_details['tag'])

        overall_success = True
        for task in primitive_tasks:
            success = self.execute_primitive_task(task, target_object_details, destination_details)
            if not success:
                rospy.logerr("MotionExecutor: Failed to execute primitive task '{}'. Aborting sequence.".format(task))
                overall_success = False
                break
            rospy.sleep(0.5)

        # Clean up scene smartly
        if self.attached_object_name: # Failed while holding?
             rospy.logwarn("Sequence finished/aborted while object '{}' still attached.".format(self.attached_object_name))
             # Optionally try to detach or go home
             # self.detach_object()
        else:
             # Object was either never picked up, or successfully released.
             # If released, it should stay in the scene. If never picked, remove original target.
             # This logic needs refinement based on final state vs desired state.
             # Simple cleanup for now: remove the object representation used during planning
             # if it wasn't successfully placed (i.e., still detached at the end)
             self.remove_object_from_scene(object_name_in_scene)


        if overall_success:
            rospy.loginfo("MotionExecutor: Primitive task sequence completed successfully.")
            # Optionally go home
            # self.go_home()
        else:
            rospy.logerr("MotionExecutor: Primitive task sequence failed.")
        return overall_success

    def shutdown(self):
        """Shuts down MoveIt Commander if initialized."""
        # Check if initialized before shutting down if multiple nodes might use it
        # moveit_commander.roscpp_shutdown()
        rospy.loginfo("Motion Executor shutting down.")

# Test block removed - Orchestrator will use this class
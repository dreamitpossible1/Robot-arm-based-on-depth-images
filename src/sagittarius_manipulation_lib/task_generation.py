#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Point, Quaternion # Make sure Pose is imported

class TaskGenerator:
    def __init__(self):
        rospy.loginfo("Task Generator Initialized.")
        # Load predefined locations from parameter server
        self.predefined_locations = self._load_predefined_locations()

    def _load_predefined_locations(self):
        """Loads locations from ROS Parameter Server."""
        locations = {}
        loc_param = rospy.get_param('/predefined_locations', {})
        for name, pose_dict in loc_param.items():
            try:
                pose = Pose()
                pose.position = Point(**pose_dict.get('position', {}))
                pose.orientation = Quaternion(**pose_dict.get('orientation', {'w':1.0}))
                locations[name] = pose
            except Exception as e:
                rospy.logerr("Error loading predefined location '{}': {}".format(name, e))
        rospy.loginfo("Loaded predefined locations: {}".format(locations.keys()))
        return locations

    def find_object_in_vision(self, object_name, vision_output):
        """Finds the specified object in the vision output."""
        # Simple matching by tag/name. Needs improvement for multiple objects of same type.
        found_objects = []
        for obj in vision_output:
            if obj.get("object_tag") == object_name:
                found_objects.append(obj)

        if not found_objects:
             rospy.logwarn("Object '{}' not found in vision output.".format(object_name))
             return None
        elif len(found_objects) > 1:
             # TODO: Implement disambiguation logic (e.g., closest, context from NLP)
             rospy.logwarn("Multiple objects found for '{}'. Using the first one.".format(object_name))
             return found_objects[0]
        else:
             rospy.loginfo("Found object '{}' in vision output.".format(object_name))
             return found_objects[0]

    def get_location_pose(self, location_name, vision_output):
        """Gets the pose for a named location (predefined or from vision)."""
        # Check predefined locations first
        if location_name in self.predefined_locations:
            rospy.loginfo("Using predefined location '{}'.".format(location_name))
            return self.predefined_locations[location_name] # Returns geometry_msgs.msg.Pose

        # Check if the location name matches a detected object tag
        for obj in vision_output:
            if obj.get("object_tag") == location_name:
                rospy.loginfo("Using location of detected object '{}'.".format(location_name))
                # Return the pose of the *object* that represents the location
                return obj.get("pose").pose # Return geometry_msgs.msg.Pose

        # TODO: Handle relative locations ("物体左边") requires spatial reasoning

        rospy.logwarn("Location '{}' could not be resolved.".format(location_name))
        return None

    def generate_tasks(self, nlp_result, vision_output):
        """
        Generates a primitive task sequence based on NLP and Vision input.
        """
        intent = nlp_result.get('NLU结果', {}).get('intent')
        action = nlp_result.get('NLU结果', {}).get('action')
        object_name = nlp_result.get('NLU结果', {}).get('object')
        location_name = nlp_result.get('NLU结果', {}).get('location')

        rospy.loginfo("Generating tasks for intent: {}, object: {}, location: {}".format(
            intent, object_name, location_name))

        # --- Input Validation ---
        if not intent or intent == 'unknown':
            rospy.logerr("Task Generation Failed: Intent is unclear.")
            return None, None, None
        if not object_name:
            rospy.logerr("Task Generation Failed: Object not specified in NLP result.")
            return None, None, None
        if intent in ['put_object', 'move_object'] and not location_name:
            rospy.logerr("Task Generation Failed: Location not specified for {} intent.".format(intent))
            return None, None, None

        # --- Contextual Validation (using Vision) ---
        target_object_info = self.find_object_in_vision(object_name, vision_output)
        if not target_object_info:
            rospy.logerr("Task Generation Failed: Target object '{}' not found in scene.".format(object_name))
            return None, None, None

        target_object_details = {
            'pose': target_object_info['pose'].pose, # Extract geometry_msgs.msg.Pose
            'tag': target_object_info['object_tag'],
            'name_in_scene': "target_object" # Consistent name for MoveIt scene
            # Add dimensions if available/needed by executor
        }

        destination_details = None
        destination_pose = None
        if intent in ['put_object', 'move_object']:
            destination_pose = self.get_location_pose(location_name, vision_output)
            if not destination_pose:
                 rospy.logerr("Task Generation Failed: Destination location '{}' could not be resolved.".format(location_name))
                 return None, None, None
            destination_details = {'pose': destination_pose}

        # --- Feasibility Checks (Simplified examples) ---
        # TODO: Add actual checks: Reachability (using MoveIt IK), Collision, Stability etc.
        rospy.loginfo("Feasibility Check (Simplified): Assuming target and destination are reachable.")

        # --- Primitive Task Sequence Generation ---
        primitive_tasks = []
        if intent == 'take_object':
            primitive_tasks = [
                "approach_pickup",
                "grasp",
                "lift",
                "retreat_after_pickup" # Added retreat for consistency
            ]
            rospy.loginfo("Generated primitive tasks for take_object: {}".format(primitive_tasks))

        elif intent == 'put_object':
             # Assume object needs picking first if not already holding
             # TODO: Add state tracking (is arm holding something?)
             primitive_tasks = [
                 "approach_pickup",
                 "grasp",
                 "lift",
                 "move_to_release_approach",
                 "release",
                 "retreat_after_release"
             ]
             rospy.loginfo("Generated primitive tasks for put_object: {}".format(primitive_tasks))

        else:
             rospy.logerr("Task Generation Failed: Intent '{}' not mapped to primitive tasks.".format(intent))
             return None, None, None

        return primitive_tasks, target_object_details, destination_details

# Example Usage removed, Orchestrator will call this
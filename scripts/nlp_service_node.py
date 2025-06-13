#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# Import the service definition and the original parser class
from sagittarius_manipulation.srv import ParseInstruction, ParseInstructionResponse
from sagittarius_manipulation_lib.instruction_parser import InstructionParser # Assuming parser is in the lib
import json # If returning complex data as JSON string

class NlpServiceNode:
    def __init__(self):
        rospy.init_node('nlp_service_node')
        self.parser = InstructionParser()
        # Load params if needed (e.g., custom dictionaries)
        # self.parser.load_config(...) 

        # Create the ROS Service
        self.service = rospy.Service('/parse_instruction', ParseInstruction, self.handle_parse_instruction)
        rospy.loginfo("NLP Service Node Ready.")
        rospy.spin()

    def handle_parse_instruction(self, req):
        rospy.loginfo(f"NLP Service: Received command '{req.command}'")
        response = ParseInstructionResponse()
        try:
            # Call the original parser's method
            parse_result = self.parser.parse(req.command) 

            # Populate the response based on ParseInstruction.srv definition
            if parse_result.get('需要澄清'):
                response.success = False
                response.clarification_needed = True
                response.error_message = parse_result['需要澄清']
            else:
                response.success = True
                response.clarification_needed = False
                # Option 1: Serialize the result if complex
                # response.result_json = json.dumps(parse_result['NLU结果']) 
                # Option 2: Populate fields directly if srv definition matches
                # Example assumes ParseInstruction.srv has fields intent, action, object, location, entities_json
                nlu_data = parse_result['NLU结果']
                response.parsed_data.intent = nlu_data.get('intent', '') 
                response.parsed_data.action = nlu_data.get('action', '')
                response.parsed_data.object = nlu_data.get('object', '')
                response.parsed_data.location = nlu_data.get('location', '')
                # Serialize entities list to JSON string for simplicity
                response.parsed_data.entities_json = json.dumps(nlu_data.get('entities', []))

        except Exception as e:
            rospy.logerr(f"NLP Service error during parsing: {e}")
            response.success = False
            response.clarification_needed = False
            response.error_message = f"Internal server error: {e}"

        return response

if __name__ == "__main__":
    NlpServiceNode()
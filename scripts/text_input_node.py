#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def text_publisher():
    """Node to publish text commands entered by the user."""
    pub = rospy.Publisher('/text_command', String, queue_size=10)
    rospy.init_node('text_input_node', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    rospy.loginfo("Text Input Node Started. Enter commands:")

    while not rospy.is_shutdown():
        try:
            
            if sys.version_info.major == 3:
                command_text = input("> ")
            else:
                break

            if command_text:
                rospy.loginfo("Publishing command: '{}'".format(command_text))
                pub.publish(command_text)
            rate.sleep()
        except EOFError: # Handle Ctrl+D
            break
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    import sys
    try:
        text_publisher()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Error in text input node: {}".format(e))
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def joint_states_callback(msg):

    rospy.loginfo("Got joint_states message with efforts: %s", msg.effort)
    
    # Create a new JointState message to publish only effort info
    effort_msg = JointState()
    
    # Copy the header to keep timestamps etc.
    effort_msg.header = msg.header
    
    # Copy the joint names
    effort_msg.name = msg.name
    
    # We only care about effort, so set position and velocity to empty lists
    effort_msg.position = []
    effort_msg.velocity = []
    
    # Copy effort data
    effort_msg.effort = msg.effort
    
    # Publish the effort message
    pub.publish(effort_msg)

if __name__ == '__main__':
    rospy.init_node('effort_extractor_node')
    
    # Publisher to publish effort-only JointState
    pub = rospy.Publisher('/open_manipulator_p/effort_only', JointState, queue_size=10)
    
    # Subscriber to original joint states
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
   
    rospy.loginfo("Effort extractor node started")

    rospy.spin()

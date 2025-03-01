#!/usr/bin/env python

import rospy
from service.srv import JointStateService

def send_joint_states():
    rospy.wait_for_service('update_joint_states')
    try:
        update_joint_service = rospy.ServiceProxy('update_joint_states', JointStateService)
        response = update_joint_service([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        print(response.message)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    send_joint_states()

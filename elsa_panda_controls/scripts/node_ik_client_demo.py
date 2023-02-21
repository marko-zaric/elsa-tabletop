#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from elsa_panda_controls.srv import GoalPoseJointSpace

def calc_ik_client(x, y, z, o_x=1000.0, o_y=1000.0, o_z=1000.0, w=1000.0):
    rospy.wait_for_service('calc_ik')
    try:
        calc_ik = rospy.ServiceProxy('calc_ik', GoalPoseJointSpace)
        resp1 = calc_ik(x, y, z, o_x, o_y, o_z, w)
        print(resp1.goal_joints)
        print(resp1.success)
        return resp1.goal_joints
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        print("Requesting")
        calc_ik_client(x, y, z)
    elif len(sys.argv) == 8:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        o_x = float(sys.argv[4])
        o_y = float(sys.argv[5])
        o_z = float(sys.argv[6])
        w = float(sys.argv[7])
        print("Requesting")
        calc_ik_client(x, y, z, o_x, o_y, o_z, w)
    else:
        print(usage())
        sys.exit(1)


#! /usr/bin/env python

import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import franka_gripper.msg

def fibonacci_client(width ):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal(width = width, speed = 5)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    rospy.init_node('grabby_client')
    i = 0
    width = [0.07,0.01,0.07]
    while (i < 3):
        try:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
            result = fibonacci_client(width[i])
            print("Result:", result)
            i += 1
        except rospy.ROSInterruptException:
            print("program interrupted before completion", file=sys.stderr)
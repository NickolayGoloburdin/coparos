#! /usr/bin/env python

import rospy
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import coparos.msg


def fibonacci_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        'fibonacci', coparos.msg.AzimuthAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = coparos.msg.AzimuthGoal(lat2=44.9337332, lon2=37.3456854)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('azimuth_client')
        result = fibonacci_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion")

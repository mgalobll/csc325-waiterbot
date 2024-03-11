#!/usr/bin/env python3

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# Modified by: Luka Mgaloblishvili
# Date modified: February 28, 2024

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

# PRE-DEFINED LOCATIONS:
testing = True
if (not testing):
    # Actual values (longer range; more suceptible to errors and timeouts given the amount of empty space in CROCHET and 60 second limit)
    PICKUP = [{'x': 6.09, 'y' : -11.5}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    DROPOFF = PICKUP
    PARK = [{'x': 5.89, 'y' : -14.2}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    TABLE1 = [{'x': 4.88, 'y' : -7.35}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    TABLE2 = [{'x': 3.68, 'y' : -6.15}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    TABLE3 = [{'x': 2.17, 'y' : -6.71}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
else:
    # Testing values (shorter distances for quick tests)
    PICKUP = [{'x': 0.44, 'y' : -7.48}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    DROPOFF = PICKUP
    PARK = [{'x': 1.46, 'y' : -8.54}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    TABLE1 = [{'x': -0.488, 'y' : -7.21}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    TABLE2 = [{'x': -0.86, 'y' : -8.17}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]
    TABLE3 = [{'x': -0.328, 'y' : -8.91}, {'r1' : 0.000, 'r2' : 0.000, 'r3' : 1.000, 'r4' : 0.000}]

POS = 0
QUAT = 1

class GoToPose():
    """
    GoToPose class from Lab 6 with slight modifications that account for specific locations and
    states of the WaiterBot.
    """

    def __init__(self):
        """
        Along with original instance variables, initializes the current state of the WaiterBot,
        which is 'idle', the pre-defined locations, and the necesasry publisher to communicate the
        current state of the WaiterBot.
        """
        self.goal_sent = False
        
        self.curState = "idle"
        self.curLoc = "Park"
        self.pubState = rospy.Publisher('state', String, queue_size=10)
        self.pubLoc = rospy.Publisher('loc', String, queue_size=10)

        self.locations = {'Pick-up':PICKUP, 'Drop-off':DROPOFF, 'Park':PARK, 'Table 1':TABLE1, 'Table 2':TABLE2, 'Table 3':TABLE3}

	    # What to do if shut down (e.g. Ctrl-C or failure)
        rospy.on_shutdown(self.shutdown)
	
    	# Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Wait for the action server to come up")

	    # Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))

    def execute(self, data):
        """
        Goes to the given location and updates and communicates the states accordingly. In case it can't reach the
        destination, returns to the origin (starting point).
        """
        self.curState = "executing"
        self.pubState.publish(self.curState)

        pos = self.locations[data.data][POS]
        quat = self.locations[data.data][QUAT]
        # Initially assumes that robot makes it to desired location
        rospy.loginfo("Going to %s location", str(data.data))
        result = self.goto(pos, quat)
        if result:
            rospy.loginfo("Reached the %s location", str(data.data))
            # Updates state accordingly
            if (data.data == 'Park'):
                self.curState = "idle"
            else:
                self.curState = "listening"
            self.pubState.publish(self.curState)
            # Update the most recent location you succesfully reached
            self.curLoc = data.data
            self.pubLoc.publish(self.curLoc)
        else:
            rospy.loginfo("Failed. Returning to the %s location", str(self.curLoc))
            # If destination can't be reached, WaiterBot goes back to its origin (assumes origin can definitely be reached)
            pos = self.locations[self.curLoc][POS]
            quat = self.locations[self.curLoc][QUAT]
            self.goto(pos, quat)
            rospy.loginfo("Reached the %s location", str(self.curLoc))
            # Updates state accordingly
            if (self.curLoc == 'Park'):
                self.curState = "idle"
            else:
                self.curState = "listening"
            self.pubState.publish(self.curState)


    def calibrateAndPark(self, data):
        pos = self.locations['Park'][POS]
        # quat = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        quat = self.locations['Park'][QUAT]
        result = self.goto(pos, quat)
        if result:
            rospy.loginfo("Successfully calibrated.")
        else:
            rospy.loginfo("Failed to calibrate. Try again...")



    def goto(self, pos, quat):
        """
        Method from original codebase left exactly as it was. Makes the WaiterBot go to the specific
        point in a map
        """
        # Send a goal
        self.goal_sent = True
    
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	    # Start moving
        self.move_base.send_goal(goal)

	    # Allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('executioner', anonymous=True)
    rate = rospy.Rate(10)
    goTo = GoToPose()

    rospy.Subscriber("calibrate", String, goTo.calibrateAndPark, queue_size=10)
    rospy.Subscriber("exec_cmd", String, goTo.execute, queue_size=10)

    rospy.spin()

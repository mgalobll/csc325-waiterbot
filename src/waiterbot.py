#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from collections import deque

class WaiterBot:
    """
    WaiterBot is like a brain that connects everything - takes commands from the user, processes them
    accordingly, and orders appropriate execution of the commands.
    """
    def __init__(self):
        """
        WaiterBot is initially idle and has no commands queued up.
        """
        self.curState = "idle"
        self.commands = deque()
        self.listeningCmds = []
        self.curLoc = "Park"
        self.pub = rospy.Publisher('exec_cmd', String, queue_size=10)

    def processCommand(self, data):
        """
        Processes commands from user inputs differently, depending on which state it is in
        """
        # When it is idle, it can execute the command without adding it to the queue
        if (self.curState == "idle"):
            # DESIGN DECISION: Should it wait for a 'Thank you' to start executing orders? Or should it start executing right from the first order it gets
            # and take subsequent orders while executing current order? Issue with inconsistency in behavior?
                # CURRENT DECISION: It starts executing orders right away when given an order, so it will ignore any 'Thank you'...
            if (data.data != 'Thank you'):
                self.pub.publish(data.data)
        # When it is listening, it will add commands to the queue, unless that command is 'Thank you'
        elif (self.curState == "listening"):
            if (data.data == "Thank you"):
                # Adds the list of orders while it was listening to front of command queue
                if (len(self.listeningCmds) > 0):
                    self.commands.extendleft(reversed(self.listeningCmds))
                    self.listeningCmds.clear()
                self.executeCommand()
            else:
                # When listening, adds new commands to front of the queue to execute them first
                # NOTE: Assumes that the commands are coming from someone at that spot
                # Adding them to seperate list while listening to ensure correct order of execution
                self.listeningCmds.append(data.data)
                
        # When it is executing, it will simply add commands to the queue
        else:
            # Will ignore the 'Thank you' command when going from one place to another, as its not a "legal" response in that state
            if (data.data != "Thank you"):
                self.commands.append(data.data)

    def updateState(self, data):
        """
        Updates the state of the current robot:
        - 'idle'
        - 'listening'
        - 'executing'
        """
        self.curState = data.data

    def updateLoc(self, data):
        """
        Updates the most recent successfully reached location of the robot.
        """
        self.curLoc = data.data

    def executeCommand(self):
        """
        Sends most recent command in queue for execution. If queue of commands is empty, will
        take WaiterBot to parking spot.
        """
        if (len(self.commands) == 0):
            self.pub.publish("Park")
        else:
            curCommand = self.commands.popleft()
            self.pub.publish(curCommand)

        rate.sleep()




if __name__ == '__main__':
    rospy.init_node('waiterbot', anonymous=True)
    rate = rospy.Rate(10)
    waiterbot = WaiterBot()

    rospy.Subscriber("order_cmd", String, waiterbot.processCommand, queue_size=10)
    rospy.Subscriber("state", String, waiterbot.updateState, queue_size=10)
    rospy.Subscriber("loc", String, waiterbot.updateLoc, queue_size=10)

    rospy.spin()


    

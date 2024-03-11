#!/usr/bin/env python

import os
import subprocess
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

def initialize():   
    """
    Localizes the robot in the map and moves it to the parking location, if not there already. This method doesn't
    end w/o completing the final steps, as the command() method assumes user completes these required steps before continuing.

    Non or Semi-Automated Steps:
    1) roscore on Remote PC
    2) Setting up the Turtlebot
        - ssh into a Turtlebot
        - Edit the .bashrc file and "sb" after you save it
        - On the Turtlebot: roslaunch turtlebot3_bringup turtlebot3_robot.launch
    3) (Fully automated) Launching RViz with the map on the Remote PC:
        - roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
    4) On RViz, localizing the Turtlebot in the map using 2D Pose Estimate

    Fully Automated Steps:
    5) Wait for above localization process to complete.
    6) Have WaiterBot move to the 'Park' location
    7) Once it gets to 'Park', the initialize method has ended and WaiterBot can start accepting commands
    """
    # Get the value of the HOME environment variable
    home_path = os.environ['HOME']

    # Construct the absolute path to the map file
    map_file_path = os.path.join(home_path, 'catkin_ws/src/waiterbot/maps/map.yaml')

    # Construct the roslaunch command with the absolute path to the map file
    roslaunch_command = f'roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:={map_file_path}'

    # Launch the roslaunch command in a new terminal tab/window using gnome-terminal
    global ros_launch_process
    ros_launch_process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', roslaunch_command])

    # Define the initialization steps
    steps = [
        "Welcome to the WaiterBot! Let's get started with initialization.",
        "Please provide the '2D Pose Estimate' in the opened RViz window.",
    ]

    print("\n========== WaiterBot Initialization ==========")
    
    # Iterate over each step and provide instructions
    for step in steps:
        print("\n", step)
        input("Press Enter to continue: ")

    # Move to the parking position
    parked = 'n'
    pub = rospy.Publisher('calibrate', String, queue_size=10)
    numTry = 0
    while (parked.lower() == 'n'):
        print("\nMoving to Parking position...")
        pub.publish("Park")
        # The first publish is ignored for some reason, so I publish twice on the first iteration
        if (numTry == 0):
            time.sleep(0.5)
            pub.publish("Park")
        parked = input("Was parking successful? (y/n): ").strip()

    print("\nWaiterBot is ready to take your orders!")

def command():
    """
    Entry point for the users into the program. Initially operating on text commands, but will
    hopefully upgrade to voice commands.

    Text Command Dictionary:
    - 'Pick-up' = go to the designated pick-up location (when human employee wants something delivered)
    - 'Drop-off' = go to the designated drop-off location (when customer wants something taken away)
    - 'Thank you' = signals WaiterBot to start executing next task (if no more tasks, will go to parking)
    - 'Table [n]' = go to the designated table number n

    NOTE: At this stage, I require the customer to explicitly order the WaiterBot to clean up after it (so for it to
    go to the "drop-off" location instead of starting a new task), but I imagine there might be some logic the
    WaiterBot can use to determine what to do without the customer input (would likely involve gesture detection). 
    """
    pub = rospy.Publisher('order_cmd', String, queue_size=10)

    try:
        while not rospy.is_shutdown():
            print("\n========== WaiterBot Control Panel ==========")
            print("Available commands:")
            print("- 'Pick-up': Go to the designated pick-up location.")
            print("- 'Drop-off': Go to the designated drop-off location.")
            print("- 'Thank you': Signal WaiterBot to start executing next task.")
            print("- 'Table [n]': Go to the designated table number n.")
            print("- Press Ctrl+C to exit the program.")
            print("=============================================")
            
            command = input("\nEnter a command: ")
            
            # Check if the command is one of the predefined options
            if command in ['Pick-up', 'Drop-off', 'Thank you', 'Table 1', 'Table 2', 'Table 3']:
                pub.publish(command)
                print("Command sent:", command)
            else:
                print("Invalid command. Please enter one of the predefined commands.")

    except KeyboardInterrupt:
        # Terminate the implicitly launched roslaunch command (DOESN'T WORK)
        if ros_launch_process:
            ros_launch_process.terminate()
        print("\nExiting the program...")
        rospy.signal_shutdown("Ctrl+C pressed")


if __name__ == '__main__':
    try:
        rospy.init_node('text_input', anonymous=True)
        initialize()
        command()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

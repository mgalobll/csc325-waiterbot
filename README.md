# Robotics (CSC325) Final Project: WaiterBot

## Description
The objective of the WaiterBot is to automate the process of delivering orders to customers at tables, and potentially “cleaning up” after them.

## Commands
- “Pick-up” - going to pick-up location
- “Drop-off” - going to drop-off location
- “Table n” - going to table number n 
- “Thank you” - signal for WaiterBot to stop listening and start executing tasks in its queue

## Usage Instructions
1. Assumes DeepSpeech is installed and waiterbot is in the catkin_ws
2. Connecting PC to Turtlebot, and running the following command in the Turtlebot:
   * roslaunch turtlebot3_bringup turtlebot3_robot.launch
3. Running the following command in the PC:
   * roslaunch waiterbot waiterbot.launch
4. A new Terminal window and RViz should open up:
   * Can ignore the terminal, but should follow the initialization instructions in the main terminal window (one of which is localizing the Turtlebot in the map in RViz)
5. WaiterBot is ready to go
6. To exit out of WaiterBot:
   * Ctrl + C
   * Need to manually exit out of the newly opened terminal and RViz window for now…

**NOTE:** Can uncomment the text_input node and comment out the audio_input node in the launch file to run the WaiterBot using text commands instead of audio.


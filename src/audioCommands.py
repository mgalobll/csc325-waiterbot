#!/usr/bin/env python3
import time
import wave
import numpy as np
import os
from deepspeech import Model
from audiorec import Recorder
import rospy
from std_msgs.msg import String
from textCommands import initialize

"""
NOTE: I get the ALSA and JACK warnings in the terminal that I couldn't figure out how to get rid of. Interrupts the flow of
      instructions/messages after WaiterBot is initialized, but doesn't appear after that.
"""

class AudioCommands:
    """
    Listens to and processes audio commands, and matches them to legal commands of the WaiterBot. Sends/publishes the
    legal commands to the WaiterBot for execution.
    """
     
    def __init__(self) :
        # Initialize Recorder class
        self.recorder = Recorder()
        # Set-up publisher
        self.pub = rospy.Publisher('order_cmd', String, queue_size=10)
        self.dictionary = [
            'pick',
            'up',
            'drop',
            'off',
            'thank',
            'you',
            'table',
            'one',
            'two',
            'three'
        ]
        self.boostValues = {
            'pick': 5.0,
            'up': 5.0,
            'drop': 5.0,
            'off': 5.0,
            'thank': 5.0,
            'you': 5.0,
            'table': 5.0,
            'one': 5.0,
            'two': 5.0,
            'three': 5.0
        }
        self.commands = [
            'pick up',
            'drop off',
            'thank you',
            'table one',
            'table two',
            'table three'
        ]
        self.translate = {
            'pick up': 'Pick-up',
            'drop off': 'Drop-off',
            'thank you': 'Thank you',
            'table one': 'Table 1',
            'table two': 'Table 2',
            'table three': 'Table 3'
        }
        # Threshold for similarity (to avoid accepting non-sense commands):
        # NOTE: Seems like a pretty conservative threshold, but I tested it out and it seemed to work quite well
        self.threshold = 1
        # Initialize the DeepSpeech model
        self.initializeModel()

    def initializeModel(self):
        # Initialize the DeepSpeech model (assumed DeepSpeech is installed on the PC and in '/opt')
        self.model = Model(os.path.abspath('/opt/deepspeech-0.9.3-models.pbmm'))
        self.model.enableExternalScorer(os.path.abspath('/opt/deepspeech-0.9.3-models.scorer'))
        # Add hot words with corresponding boost values
        for word, boost in self.boostValues.items():
            self.model.addHotWord(word, boost)

    def listenProcessCommands(self):
        """
        Listens to audio commands while program is running, and sends relevant commands to the WaiterBot for processing and execution
        """
        # Path to HOME 
        home_path = os.environ['HOME']
        # Construct the absolute path to the directory where audio files will be saved
        f_name_directory = os.path.join(home_path, 'catkin_ws/src/waiterbot/audioCmds')

        print("\n========== WaiterBot Control Panel ==========")
        print("Available commands:")
        print("- 'Pick-up': Go to the designated pick-up location.")
        print("- 'Drop-off': Go to the designated drop-off location.")
        print("- 'Thank you': Signal WaiterBot to start executing next task.")
        print("- 'Table [n]': Go to the designated table number n.")
        print("- Press Ctrl+C to exit the program.")
        print("=============================================")

        while (not rospy.is_shutdown()):
            # DESIGN DECISION: Record and save new audio file each time new audio is heard. In the future, it might be worth
            # looking into storing a max certain amount of files, so the oldest would be deleted when that amount is reached.
            # (Rationale: to avoid taking up too much memory).
            audio_file = os.path.join(f_name_directory, self.recorder.record())
            audio_data = self.load_wav_file(audio_file)
            curCommand = self.stt(audio_data)
            # print(curCommand)
            # Ensures that processed command is valid
            if (curCommand in ['Pick-up', 'Drop-off', 'Thank you', 'Table 1', 'Table 2', 'Table 3']):
                self.pub.publish(curCommand)
                print("Command sent:", curCommand)
            else:
                print("Invalid command. Please try again...")
            print("\n")
            # To have some buffer between recordings:
            time.sleep(1)


    def load_wav_file(self, file_path):
        """
        Loads given audio 'wav' file and returns audio data to be passed into the DeepSpeech model
        """
        with wave.open(file_path, 'rb') as wav_file:
            with wave.open(file_path, 'rb') as wav_file:
                # Get the sample rate of the WAV file
                original_sample_rate = wav_file.getframerate()

                # Read all frames of the audio data
                audio_data = wav_file.readframes(wav_file.getnframes())
        
        # Convert the audio data to a numpy array with dtype 'int16'
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        return audio_array
    
    def levenshtein_distance(self, str1, str2):
        """
        Compares two words and returns their Levenshtein distance: smaller the distance, more similar the words.

        Source: https://github.com/RichardKelley/unr_deepspeech
        Modified by: Luka Mgaloblishvili
        """
        if len(str2) > len(str1):
            str2, str1 = str1, str2
        row_count = len(str1)
        col_count = len(str2)

        row_current = range(row_count + 1)
        for row_n in range(row_count):
            row_next = [row_n + 1]
            for col_n in range(col_count):
                delete_cost = row_current[col_n + 1] + 1
                insert_cost = row_next[col_n] + 1
                substitution_cost = row_current[col_n] + 1
                if str1[row_n] == str2[col_n]:
                    substitution_cost = row_current[col_n]

                row_next.append(min([substitution_cost, delete_cost,
                                     insert_cost]))
            row_current = row_next
        return row_current[-1]

    def stt(self, audio):
        """
        Transcribes the given audio using DeepSpeech, and associates it with the most similar, legal command
        using the Levenshtein distance. Returns 'Invalid Command' if no sufficient similarity with any legal command

        Critical Assumptions (For Now):
            - It assumes it is operating in a sufficiently quiet environment
            - It assumes that for each time it listens, it should expect a single command, as opposed to a combination
              of several commands in one go.
        
        Source: https://github.com/RichardKelley/unr_deepspeech
        Modified by: Luka Mgaloblishvili
        """
        transcription = self.model.stt(audio)

        # Invalid command if nothing was heard/given
        if (not transcription.strip()):
            return "Invalid command"

        transcription_words = transcription.split(" ")

        new_transcription = ""
        for transcribed_word in transcription_words:
            distances=[self.levenshtein_distance(transcribed_word, dict_word)
                        for dict_word in self.dictionary]
            min_dist_index = min(range(len(distances)),
                                    key=lambda x: distances[x])
            word_guess = self.dictionary[min_dist_index]
            # Enforcing threshold for first pass of Levenshtein distance on each word of supposed command
            if (distances[min_dist_index] <= self.threshold):
                new_transcription += word_guess + " "

        # In case threshold filtered out all words in the supposed command
        if not new_transcription:
            return "Invalid command"

        transcription = new_transcription
        distances = [self.levenshtein_distance(transcription, possibility)
                        for possibility in self.commands]
        min_dist_index = min(range(len(distances)),
                                key=lambda x: distances[x])
        # Enforcing threshold a second time; now with full commands
        if (distances[min_dist_index] > self.threshold):
                return "Invalid command"
        transcription_guess = self.commands[min_dist_index]
        transcription = transcription_guess

        return self.translate[transcription]

if __name__ == '__main__':
    try:
        rospy.init_node('audio_input', anonymous=True)
        initialize()
        audioCmds = AudioCommands()
        audioCmds.listenProcessCommands()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
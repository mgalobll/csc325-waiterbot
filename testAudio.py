import time
import wave
import numpy as np
import os
from deepspeech import Model
from src.audiorec import Recorder

"""
- Try integrating and testing the effectiveness of the "Levenshtein Distance" method in the repo
  for processing the speech to text output and converting it to meaningful data.
    * https://github.com/RichardKelley/unr_deepspeech
    * Run tests to see effectiveness
    * Add a method that converts the processed output into a legal command
"""


class AudioCommands:
     
    def __init__(self) :
        # Initialize Recorder class
        self.recorder = Recorder()
        # Set-up publisher
        # pub = rospy.Publisher('order_cmd', String, queue_size=10)
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
        # Initialize the DeepSpeech model
        self.initializeModel()

        # Testing threshold:
        self.threshold = 1

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

        while (True):
            audio_file = os.path.join(f_name_directory, self.recorder.record())
            audio_data = self.load_wav_file(audio_file)
            curCommand = self.stt(audio_data)
            print(curCommand)
            # if (curCommand in ['Pick-up', 'Drop-off', 'Thank you', 'Table 1', 'Table 2', 'Table 3']):
            #     print(curCommand)

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
        using the Levenshtein distance. 
        
        NOTE: ASSUMES that any command it hears is a legal command. (Rationale: we get the most similar legal word with
            the Levenshtein distance. Using certain thresholds to determine if two words are similar enough to be equal
            is quite difficult, so it can be left as future work for now)

        Source: https://github.com/RichardKelley/unr_deepspeech
        Modified by: Luka Mgaloblishvili
        """
        transcription = self.model.stt(audio)

        print("Raw Input: ", transcription)

        transcription_words = transcription.split(" ")
        # Edge Case: Invalid command if nothing was heard/given
        if (not transcription.strip()):
            return "Invalid command"

        new_transcription = ""
        for transcribed_word in transcription_words:
            distances=[self.levenshtein_distance(transcribed_word, dict_word)
                        for dict_word in self.dictionary]
            min_dist_index = min(range(len(distances)),
                                    key=lambda x: distances[x])
            word_guess = self.dictionary[min_dist_index]
            print("Levenshtein distance for ", transcribed_word)
            print(distances[min_dist_index])
            if (distances[min_dist_index] <= self.threshold):
                new_transcription += word_guess + " "

        if not new_transcription:
            return "Invalid command"

        transcription = new_transcription
        distances = [self.levenshtein_distance(transcription, possibility)
                        for possibility in self.commands]
        min_dist_index = min(range(len(distances)),
                                key=lambda x: distances[x])
        if (distances[min_dist_index] > self.threshold):
                return "Invalid command"
        transcription_guess = self.commands[min_dist_index]
        transcription = transcription_guess

        return self.translate[transcription]

if __name__ == '__main__':
    audioCmds = AudioCommands()
    audioCmds.listenProcessCommands()
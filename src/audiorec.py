#!/usr/bin/env python3

# Source: CSC325 Nexus Page

# Modified by: Luka Mgaloblishvili
# Date modified: March 4, 2024

import pyaudio
import math
import struct
import wave
import time
import os

# Increased threshold value to ignore background noise as much as possible
Threshold = 250

SHORT_NORMALIZE = (1.0/32768.0)
chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
swidth = 2

TIMEOUT_LENGTH = 2

# Get the value of the HOME environment variable
home_path = os.environ['HOME']

# Construct the absolute path to the map file
f_name_directory = os.path.join(home_path, 'catkin_ws/src/waiterbot/audioCmds')

class Recorder:
    """
    Recorder initializes default microphone on PC it is running on and provides necessary methods to utilize the microphone.

    NOTE: By default, connects to the Logitech webcam/microphone if connected to the PC. That's what I was testing it on and am
        100% certain that it works on that. Not sure about other external and/or built-in microphones.
    """

    @staticmethod
    def rms(frame):
        count = len(frame) / swidth
        format = "%dh" % (count)
        shorts = struct.unpack(format, frame)

        sum_squares = 0.0
        for sample in shorts:
            n = sample * SHORT_NORMALIZE
            sum_squares += n * n
        rms = math.pow(sum_squares / count, 0.5)

        return rms * 1000

    def __init__(self):
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=FORMAT,
                                    channels=CHANNELS,
                                    rate=RATE,
                                    input=True,
                                    output=True,
                                    frames_per_buffer=chunk)

    def record(self):
        print('Noise detected, listening to command...')
        rec = []
        current = time.time()
        end = time.time() + TIMEOUT_LENGTH

        while current <= end:

            data = self.stream.read(chunk)
            if self.rms(data) >= Threshold: end = time.time() + TIMEOUT_LENGTH

            current = time.time()
            rec.append(data)
        # Returns the file name that was saved, so it can be accessed easily
        return self.write(b''.join(rec))

    def write(self, recording):

        n_files = len(os.listdir(f_name_directory))

        fileWritten = '{}.wav'.format(n_files)

        filename = os.path.join(f_name_directory, '{}.wav'.format(n_files))

        wf = wave.open(filename, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(self.p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(recording)
        wf.close()
        # print('Written to file: {}'.format(filename))
        # print('Returning to listening')

        return fileWritten


    def listen(self):
        # print('Listening beginning')
        silent = True
        while silent:
            input = self.stream.read(chunk)
            rms_val = self.rms(input)
            if rms_val > Threshold:
                silent = False
                self.record()
                
    # Speech threshold around 125 RMS
    def testThresh(self):
        print('Testing beginning')
        while True:
            input = self.stream.read(chunk)
            rms_val = self.rms(input)
            print("RMS:",rms_val)






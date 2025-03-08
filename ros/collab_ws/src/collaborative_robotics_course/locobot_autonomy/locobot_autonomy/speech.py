#!/usr/bin/env python3
"""
Speech Processor Node
--------------------
This node handles real-time audio capture, transcription, and command parsing.
It integrates with the locobot system to process verbal commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import os
import sounddevice as sd
import wave
from google.cloud import speech_v1p1beta1 as speech
import time
import sounddevice as sd
import numpy as np

from speech_transcriber import SpeechTranscriber

class SpeechProcessorNode(Node):
    """ROS node that captures audio, transcribes it, and parses commands."""
    
    def __init__(self):
        super().__init__('speech_processor_node')
        self.get_logger().info('Starting speech processor node...')
        
        # Initialize paths
        # You may need to adjust these paths for your system
        self.json_key_path = "/home/locobot/Desktop/ME326_Project/loulou_key.json"
        self.audio_filename = "recorded_audio.wav"
        
        # Check if credentials file exists
        if os.path.exists(self.json_key_path):
            self.get_logger().info(f"Found credentials at {self.json_key_path}")
        else:
            self.get_logger().error(f"Credentials not found at {self.json_key_path}")
            # We'll continue, but speech recognition will likely fail
        
        # Initialize the speech transcriber
        self.transcriber = SpeechTranscriber()
        
        # Audio recording parameters
        self.sample_rate = 16000
        self.recording_duration = 5  # seconds
        
        # Publishers
        self.command_pub = self.create_publisher(
            String, 
            '/speech/parsed_command', 
            10
        )
        
        self.transcript_pub = self.create_publisher(
            String, 
            '/speech/raw_transcript', 
            10
        )
        
        self.object_pub = self.create_publisher(
            String, 
            '/speech/target_object', 
            10
        )
        
        self.task_pub = self.create_publisher(
            String, 
            '/speech/task_type', 
            10
        )
        
        # Subscribers
        self.record_trigger_sub = self.create_subscription(
            Bool,
            '/speech/start_recording',
            self.record_trigger_callback,
            10
        )
        
        # Flag to prevent re-recording if already in progress
        self.is_recording = False
        
        self.get_logger().info('Speech processor node initialized and ready')
    
    def record_trigger_callback(self, msg):
        """
        Handle recording trigger messages.
        
        Args:
            msg: Boolean message that triggers recording when True
        """
        if msg.data and not self.is_recording:
            self.is_recording = True
            self.get_logger().info('Starting to record audio...')
            self.record_and_process_audio()
            self.is_recording = False
    
    def record_and_process_audio(self):
        """
        Record audio from microphone, transcribe, and process the command.
        """
        try:
            #self.play_start_ringtone()

            # Record audio
            self.get_logger().info(f"Recording for {self.recording_duration} seconds...")
            recorded_data = sd.rec(
                int(self.recording_duration * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype='int16'
            )
            sd.wait()  # Wait until recording is complete

            #self.play_stop_ringtone()
            
            self.get_logger().info("Recording complete")
            
            # Save audio to file
            with wave.open(self.audio_filename, 'wb') as wf:
                wf.setnchannels(1)  # mono
                wf.setsampwidth(2)  # 2 bytes (16 bits)
                wf.setframerate(self.sample_rate)
                wf.writeframes(recorded_data.tobytes())
            
            # Read file for processing
            with open(self.audio_filename, 'rb') as audio_file:
                audio_content = audio_file.read()
            
            # Process the command
            self.get_logger().info("Transcribing and parsing command...")
            result = self.transcriber.process_command(audio_content)
            
            if not result['transcript']:
                self.get_logger().warn('No speech detected or transcription failed')
                return
            
            # Log the results
            self.get_logger().info(f"Transcript: {result['transcript']}")
            self.get_logger().info(f"Task: {result['task_type']}")
            self.get_logger().info(f"Object: {result['object']}")
            self.get_logger().info(f"Color: {result['color']}")
            self.get_logger().info(f"Destination: {result['destination']}")
            
            # Publish the results
            transcript_msg = String()
            transcript_msg.data = result['transcript']
            self.transcript_pub.publish(transcript_msg)
            
            object_msg = String()
            object_msg.data = result['object'] if result['object'] else ""
            self.object_pub.publish(object_msg)
            
            task_msg = String()
            task_msg.data = result['task_type'] if result['task_type'] else ""
            self.task_pub.publish(task_msg)
            
            # Publish JSON format with all command details
            command_msg = String()
            command_msg.data = json.dumps(result)
            self.command_pub.publish(command_msg)
            
            self.get_logger().info("Command processed and published")
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
        
    def play_start_ringtone(self):
        """Play a ringtone to indicate recording has started"""
        # Generate a short ascending tone sequence
        sample_rate = 22050
        duration = 0.5  # seconds
        frequencies = [523.25, 659.25, 783.99]  # C5, E5, G5 (C major chord ascending)
        
        # Generate the tones
        t = np.linspace(0, duration, int(sample_rate * duration), False)
        note_duration = duration / len(frequencies)
        signal = np.zeros_like(t)
        
        for i, freq in enumerate(frequencies):
            start_idx = int(i * note_duration * sample_rate)
            end_idx = int((i + 1) * note_duration * sample_rate)
            signal[start_idx:end_idx] = 0.5 * np.sin(2 * np.pi * freq * t[start_idx:end_idx])
        
        # Add a smooth envelope
        envelope = np.ones_like(signal)
        attack = int(0.01 * sample_rate)
        release = int(0.05 * sample_rate)
        envelope[:attack] = np.linspace(0, 1, attack)
        envelope[-release:] = np.linspace(1, 0, release)
        signal = signal * envelope
        
        # Play the sound
        sd.play(signal, sample_rate)
        sd.wait()

    def play_stop_ringtone(self):
        """Play a ringtone to indicate recording has stopped"""
        # Generate a short descending tone sequence
        sample_rate = 22050
        duration = 0.5  # seconds
        frequencies = [783.99, 659.25, 523.25]  # G5, E5, C5 (C major chord descending)
        
        # Generate the tones
        t = np.linspace(0, duration, int(sample_rate * duration), False)
        note_duration = duration / len(frequencies)
        signal = np.zeros_like(t)
        
        for i, freq in enumerate(frequencies):
            start_idx = int(i * note_duration * sample_rate)
            end_idx = int((i + 1) * note_duration * sample_rate)
            signal[start_idx:end_idx] = 0.5 * np.sin(2 * np.pi * freq * t[start_idx:end_idx])
        
        # Add a smooth envelope
        envelope = np.ones_like(signal)
        attack = int(0.01 * sample_rate)
        release = int(0.05 * sample_rate)
        envelope[:attack] = np.linspace(0, 1, attack)
        envelope[-release:] = np.linspace(1, 0, release)
        signal = signal * envelope
        
        # Play the sound
        sd.play(signal, sample_rate)
        sd.wait()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SpeechProcessorNode()
        rclpy.spin(node)
    except Exception as e:
        print(f'Error in speech processor node: {str(e)}')
        import traceback
        print(traceback.format_exc())
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
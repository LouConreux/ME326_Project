#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
from speech_transcriber import SpeechTranscriber

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_processing_node')
        
        # Initialize speech transcriber
        self.transcriber = SpeechTranscriber()
        
        # Publishers
        self.parsed_command_pub = self.create_publisher(String, '/speech/parsed_command', 10)
        self.transcript_pub = self.create_publisher(String, '/speech/raw_transcript', 10)
        
        # Subscribers
        self.listen_sub = self.create_subscription(Bool, '/speech/start_listening', 
                                                  self.listen_callback, 10)
        
        # In a real system, you'd subscribe to your audio source
        # For now, we'll simulate with a timer for testing
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.should_listen = False
        
    def listen_callback(self, msg):
        self.should_listen = msg.data
        if self.should_listen:
            self.get_logger().info('Started listening')
        else:
            self.get_logger().info('Stopped listening')
    
    def timer_callback(self):
        # This would be where you'd listen for audio in a real system
        # For testing, we'll use sample commands
        if self.should_listen:
            # In a real system, get audio data and pass it to transcriber
            sample_commands = [
                "find the red apple",
                "bring me the blue cube",
                "sort objects by color"
            ]
            import random
            sample_cmd = random.choice(sample_commands)
            
            # Parse the command
            result = self.transcriber.parse_command(sample_cmd)
            
            # Publish results
            cmd_msg = String()
            cmd_msg.data = json.dumps(result)
            self.parsed_command_pub.publish(cmd_msg)
            
            transcript_msg = String()
            transcript_msg.data = sample_cmd
            self.transcript_pub.publish(transcript_msg)
            
            # Turn off listening after processing one command
            self.should_listen = False

def main():
    rclpy.init()
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
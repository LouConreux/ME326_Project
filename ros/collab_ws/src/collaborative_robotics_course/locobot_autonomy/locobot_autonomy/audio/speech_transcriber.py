import os
import re
from google.cloud import speech_v1p1beta1 as speech

JSON_KEY_PATH = '/home/locobot/Group3/ME326_Project/loulou_key.json'
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = JSON_KEY_PATH

class SpeechTranscriber:
    def __init__(self, language_code='en-US', sample_rate=16000):
        """
        Initialize a SpeechTranscriber instance.

        :param language_code: The language code for transcription, e.g., 'en-US'.
        :param sample_rate: The sample rate (Hertz) of the audio file, default is 16000.
        """
        self.language_code = language_code
        self.sample_rate = sample_rate

        self.client = speech.SpeechClient()

                # Define task types and their synonyms/patterns
        self.task_mapping = {
            # Object retrieval task
            'retrieve': [
                r'\bretrieve\b', 
                r'\bbring\b', 
                r'\bfetch\b', 
                r'\bget\b.*\band\b.*\bbring\b',
                r'\blocate\b.*\band\b.*\bretrieve\b',
                r'\bfind\b.*\band\b.*\bbring\b',
                r'\btake\b.*\band\b.*\breturn\b',
                r'\bbring back\b'
            ],
            
            # Sequential task - find and place
            'sequential': [
                r'\bfind\b.*\band\b.*\bplace\b',
                r'\bpick\b.*\band\b.*\bplace\b',
                r'\blocate\b.*\band\b.*\bput\b',
                r'\btake\b.*\band\b.*\bplace\b',
                r'\bmove\b.*\bto\b',
                r'\btransfer\b'
            ],
            
            # Color sorting task
            'sort': [
                r'\bsort\b', 
                r'\bgroup\b', 
                r'\bcategorize\b',
                r'\bseparate\b.*\bby color\b',
                r'\borganize\b.*\bby color\b',
                r'\bsort\b.*\bby color\b'
            ],
            
            # Basic actions that might be part of larger tasks
            'find': [
                r'\bfind\b', 
                r'\blook for\b', 
                r'\bsearch\b', 
                r'\blocate\b', 
                r'\bwhere is\b'
            ],
            'pick': [
                r'\bpick up\b', 
                r'\bgrab\b', 
                r'\btake\b'
            ],
            'place': [
                r'\bput down\b', 
                r'\bplace\b', 
                r'\bset\b', 
                r'\bdrop\b'
            ]
        }
        
        # Compile regular expressions for faster matching
        self.task_patterns = {
            task: [re.compile(pattern, re.IGNORECASE) for pattern in patterns]
            for task, patterns in self.task_mapping.items()
        }
        
        # Common objects the robot might interact with
        self.common_objects = [
            'apple', 'orange', 'banana', 'bottle', 'cup', 'book', 'pen',
            'phone', 'ball', 'box', 'chair', 'table', 'door', 'toy',
            'cube', 'block', 'marker', 'remote', 'laptop', 'basket'
        ]
        
        # Common colors
        self.colors = [
            'red', 'blue', 'green', 'yellow', 'orange', 'purple', 
            'pink', 'brown', 'black', 'white', 'gray'
        ]


    def transcribe_audio(self, audio_content):
        """
        Uses Google Cloud Speech-to-Text to transcribe the given audio content (bytes).
        Returns the transcription as a string.

        :param audio_content: The raw bytes of the audio file to be transcribed.
        :return: A string of the combined transcription.
        """
        audio = speech.RecognitionAudio(content=audio_content)

        config = speech.RecognitionConfig(
            sample_rate_hertz=self.sample_rate,
            language_code=self.language_code,
            enable_automatic_punctuation=True,
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        )

        try:
            response = self.client.recognize(config=config, audio=audio)
            if response.results:
                transcript = response.results[0].alternatives[0].transcript
                return transcript
            return ""
        except Exception as e:
            print(f"Error in transcription: {str(e)}")
            return ""
        
    def parse_command(self, transcript):
        """
        Parse command to extract task type, target object, and other relevant information.
        
        Args:
            transcript: Transcribed speech command
            
        Returns:
            dict: Contains 'task_type', 'object', 'color', 'destination', etc.
        """
        transcript = transcript.lower().strip()
        result = {
            'task_type': None,
            'object': None,
            'color': None,
            'destination': None
        }
        
        # Find task type - prioritize complex tasks over simple actions
        task_priority = ['retrieve', 'sequential', 'sort', 'find', 'pick', 'place']
        for task in task_priority:
            patterns = self.task_patterns[task]
            for pattern in patterns:
                if pattern.search(transcript):
                    result['task_type'] = task
                    break
            if result['task_type']:
                break
        
        # Extract color
        for color in self.colors:
            if re.search(r'\b' + re.escape(color) + r'\b', transcript, re.IGNORECASE):
                result['color'] = color
                break
        
        # Extract object - first check common objects
        for obj in self.common_objects:
            if re.search(r'\b' + re.escape(obj) + r'\b', transcript, re.IGNORECASE):
                result['object'] = obj
                break
        
        # Look for destination in sequential task
        if result['task_type'] == 'sequential':
            # Look for destination after "in" or "on"
            destination_match = re.search(r'\bin\b\s+(?:the\s+)?(\w+)', transcript, re.IGNORECASE)
            if destination_match:
                result['destination'] = destination_match.group(1)
            else:
                destination_match = re.search(r'\bon\b\s+(?:the\s+)?(\w+)', transcript, re.IGNORECASE)
                if destination_match:
                    result['destination'] = destination_match.group(1)
        
        # If object not found using common objects, try to extract it
        if not result['object']:
            # Try to find object after "the" but before any preposition
            obj_match = re.search(r'\bthe\s+(?:' + '|'.join(self.colors) + r'\s+)?(\w+)(?:\s+(?:in|on|to|from))?', transcript, re.IGNORECASE)
            if obj_match:
                result['object'] = obj_match.group(1)
            
            # If still no object, just take the first noun after any verb
            if not result['object'] and result['task_type']:
                words = transcript.split()
                for i, word in enumerate(words):
                    if i < len(words) - 1 and word in ['the', 'a', 'an']:
                        result['object'] = words[i+1]
                        break
        
        return result
    
    def process_command(self, audio_content):
        """
        Process audio command by transcribing and parsing it.
        
        Args:
            audio_content: The raw bytes of the audio file
            
        Returns:
            dict: Contains parsed command information
        """
        # Transcribe the audio
        transcript = self.transcribe_audio(audio_content)
        
        if not transcript:
            return {
                'transcript': '',
                'task_type': None,
                'object': None,
                'color': None,
                'destination': None
            }
        
        # Parse the command
        parsed_command = self.parse_command(transcript)
        
        # Return combined results
        return {
            'transcript': transcript,
            **parsed_command
        }

if __name__ == "__main__":
    transcriber = SpeechTranscriber()
    
    # Example transcripts to test the parser
    test_commands = [
        "Find the red apple",
        "Locate the apple in the scene and retrieve it",
        "Find the red apple and place it in the basket",
        "Sort objects by color",
        "Group the blocks by their colors",
        "Bring me the blue cube",
        "Pick up the yellow banana and put it on the table"
    ]
    
    print("Testing command parsing:")
    print("------------------------")
    
    for cmd in test_commands:
        result = transcriber.parse_command(cmd)
        print(f"Command: {cmd}")
        print(f"Task: {result['task_type']}")
        print(f"Object: {result['object']}")
        print(f"Color: {result['color']}")
        print(f"Destination: {result['destination']}")
        print("-" * 30)
from speech_transcriber import SpeechTranscriber
from generate_prompt import GeminiClass
from vision_detection import VisionObjectDetector

class PipelinePerception:
    def __init__(self):
        """
        Initializes the Perception pipeline with the necessary components.
        """
        self.transcriber = SpeechTranscriber()
        self.gemini = GeminiClass(prompt="In the given voice transcript, identify what the object is that the user wants. Return only the object in lowercase and do not include any whitespaces, punctuation, or new lines. Here is the voice transcript: ")
        self.detector = VisionObjectDetector()

    def combined_pipeline(self, audio, image):
        """
        Processes audio and image inputs to determine the center coordinates of the object
        specified in the audio.

        Parameters:
        - audio: The audio input to be transcribed into text.
        - image: The image input in which the object should be detected.

        Returns:
        - center_coordinates: A tuple representing the (x, y) coordinates of the object's center,
        or None if the object is not found.
        """
        audio_transcription = self.transcriber.transcribe_audio(audio)
        object_name = self.gemini.generate_content(audio_transcription, True)
        object_name = object_name.strip().lower().replace(" ", "").replace("\n", "").replace("\r", "")
        center_coordinates = self.detector.find_center(image, object_name)
        return center_coordinates, object_name
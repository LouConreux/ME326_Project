from speech_transcriber import SpeechTranscriber
from generate_prompt import GeminiClass
from vision_detection import VisionObjectDetector

class PipelinePerception:
    def __init__(self, prompt=None):	
        """
        Initializes the Perception pipeline with the necessary components.
        """
        self.transcriber = SpeechTranscriber()
        self.gemini = GeminiClass(prompt=prompt)
        self.detector = VisionObjectDetector()

    def command_pipeline(self, command, image):
        """
        Processes the written command and image inputs to determine the center coordinates of the object
        specified in the command.

        Parameters:
        - command: The prompt input containing the object to be detected.
        - image: The image (in bytes) input in which the object should be detected.

        Returns:
        - center_coordinates: A tuple representing the (x, y) coordinates of the object's center,
        or None if the object is not found.
        """
        object_name = self.gemini.generate_content(command, False)
        object_name = object_name.strip().lower().replace(" ", "").replace("\n", "").replace("\r", "")
        center_coordinates = self.detector.find_center(image, object_name)
        return center_coordinates, object_name

    def audio_pipeline(self, audio, image):
        """
        Processes audio and image inputs to determine the center coordinates of the object
        specified in the audio.

        Parameters:
        - audio: The audio input to be transcribed into text.
        - image: The image (in bytes) input in which the object should be detected.

        Returns:
        - center_coordinates: A tuple representing the (x, y) coordinates of the object's center,
        or None if the object is not found.
        """
        audio_transcription = self.transcriber.transcribe_audio(audio)
        self.gemini.set_prompt("In the given voice transcript, identify what the object is that the user wants. Return only the object in lowercase and do not include any whitespaces, punctuation, or new lines. Here is the voice transcript: ")
        object_name = self.gemini.generate_content(audio_transcription, True)
        object_name = object_name.strip().lower().replace(" ", "").replace("\n", "").replace("\r", "")
        center_coordinates = self.detector.find_center(image, object_name)
        return center_coordinates, object_name
    
    def color_ranking(self, image):
        """
        Ranks the colors of the detected object in the image.

        Parameters:
        - image: The image (in bytes) input in which the object should be detected.
        - object_name: The name of the object to be detected.

        Returns:
        - color_ranking: A list of tuples representing the color ranking of the object.
        """
        colored_objects = self.detector.get_colored_objects(image)
        color_ranking_str = self.gemini.generate_from_image(image, f"In the given image, identify the colors of the detected objects, then rank the detected objects based on their color following the visible light spectrum, so first purple or blue objects and finish by the more red ones. Return the text result as a list of color-ranked objects separated by a coma only. Here are the detected objects: {colored_objects.keys()} that you need to rank")
        color_ranking = color_ranking_str.split(",")
        return color_ranking
    
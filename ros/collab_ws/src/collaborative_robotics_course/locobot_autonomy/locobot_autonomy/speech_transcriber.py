import os
from google.cloud import speech_v1p1beta1 as speech

JSON_KEY_PATH = "/home/ubuntu/Desktop/collaborative/keys/tomtom_key.json"

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
        response = self.client.recognize(config=config, audio=audio)
        transcript = response.results[0].alternatives[0].transcript
        return transcript
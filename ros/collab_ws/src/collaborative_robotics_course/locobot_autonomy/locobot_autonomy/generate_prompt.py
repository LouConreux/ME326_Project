import os
import google.generativeai as genai

JSON_KEY_PATH = "/home/ubuntu/Desktop/collaborative/keys/tomtom_key.json"
os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = JSON_KEY_PATH

class GeminiClass:
    def __init__(self, prompt=None):
        """
        Constructor for the GeminiClass.
        Initializes the generative model and sets a default prompt.

        Parameters:
        - prompt (optional): A string to set as the default prompt. Defaults to an empty string if not provided.
        """
        self.gemini_model = genai.GenerativeModel("gemini-1.5-flash")
        if prompt:
            self.prompt = prompt
        else:
            self.prompt = ""

    def set_prompt(self, prompt):
        """
        Sets or updates the prompt for the GeminiClass.

        Parameters:
        - prompt: A string that will serve as the base prompt for content generation.
        """
        self.prompt = prompt

    def generate_content(self, text, usePrompt):
        """
        Generates content using the generative model.

        Parameters:
        - text: A string input to provide to the model for content generation.
        - usePrompt: A boolean indicating whether to prepend the class's prompt to the input text.

        Returns:
        - The generated content as a string.
        """
        if usePrompt:
            text = self.prompt + text
        result = self.gemini_model.generate_content(text)
        response = result.text
        return response

    def generate_from_image(self, image_bytes, textInput):
        """
        Generates content based on an image and optional text input.

        Parameters:
        - image_bytes: The binary data of the image.
        - textInput: A string input to accompany the image for content generation.

        Returns:
        - The generated content as a string.
        """
        input = list([image_bytes, textInput])
        result = self.gemini_model.generate_content(input)
        response = result.text
        return response

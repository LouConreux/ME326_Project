import os
import io
import numpy as np
from PIL import Image
from PIL import Image as ImageDraw
from google.cloud import vision
import colorsys

JSON_KEY_PATH = "/home/ubuntu/Desktop/collaborative/keys/tomtom_key.json"

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = JSON_KEY_PATH

class VisionObjectDetector:
    def __init__(self):
        """
        Initialize the Vision client once during object creation.
        """
        self.client = vision.ImageAnnotatorClient()

    def find_center(self, image_bytes, object_name):
        """
        Finds the center of an object (e.g., "pineapple") in the provided image bytes.

        :param image_bytes: The raw bytes of the image.
        :param object_name: The target object name to search for (case-insensitive).
        :return: Tuple (pixel_x, pixel_y) of the object's approximate center, or None if not found.
        """
        image = vision.Image(content=image_bytes)
        pil_image = Image.open(io.BytesIO(image_bytes))
        width, height = pil_image.size

        response = self.client.object_localization(image=image)

        objects = response.localized_object_annotations

        for obj in objects:
            print(f"Found object: {obj.name}")
            if obj.name.lower() in object_name.lower():
                vertices = obj.bounding_poly.normalized_vertices

                x_min = vertices[0].x * width
                y_min = vertices[0].y * height
                x_max = vertices[2].x * width
                y_max = vertices[2].y * height

                center_x = (x_min + x_max) / 2
                center_y = (y_min + y_max) / 2

                return int(center_x), int(center_y)

        return None, None
    
    def annotate_image(self, image_bytes):
        """
        Detects all objects in the image and returns a PIL Image with
        bounding boxes and labels drawn for each detected object.

        :param image_bytes: The raw bytes of the image.
        :return: A PIL Image object annotated with bounding boxes and labels.
        """

        image = vision.Image(content=image_bytes)

        response = self.client.object_localization(image=image)

        objects = response.localized_object_annotations

        pil_image = Image.open(io.BytesIO(image_bytes))
        width, height = pil_image.size

        draw = ImageDraw.Draw(pil_image)
        for obj in objects:
            vertices = [(vertex.x * width, vertex.y * height) for vertex in obj.bounding_poly.normalized_vertices]
            draw.polygon(vertices, outline='red')

        return pil_image
    
    def get_colored_objects(self, image_bytes):
        """
        Detects colored objects in the image

        :param image_bytes: The raw bytes of the image.
        :return: A list of detected objects with their associated color.
        """
        colored_objects = []

        image = vision.Image(content=image_bytes)

        response = self.client.object_localization(image=image)

        objects = response.localized_object_annotations

        pil_image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
        width, height = pil_image.size
        image_np = np.array(pil_image)

        for obj in objects:
            vertices = obj.bounding_poly.normalized_vertices

            x_min = int(vertices[0].x * width)
            y_min = int(vertices[0].y * height)
            x_max = int(vertices[2].x * width)
            y_max = int(vertices[2].y * height)

            center_x = (x_min + x_max) / 2
            center_y = (y_min + y_max) / 2
            
            cropped_region = image_np[y_min:y_max, x_min:x_max]

            avg_color = np.mean(cropped_region, axis=(0, 1)).astype(int)  # (R, G, B)
            avg_color_tuple = tuple(avg_color)

            avg_color_norm = tuple(c / 255.0 for c in avg_color)
            hue, sat, val = colorsys.rgb_to_hsv(*avg_color_norm)
            hue_deg = hue * 360 

            colored_objects.append({
                "name": obj.name,
                "bounding_box": (x_min, y_min, x_max, y_max),
                "color": avg_color_tuple,
                "hue": hue_deg
            })
        return colored_objects

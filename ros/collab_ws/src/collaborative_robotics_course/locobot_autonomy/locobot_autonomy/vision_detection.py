import os
import io
import numpy as np
from PIL import Image
from PIL import Image as ImageDraw
from google.cloud import vision
import cv2

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
    
    def get_object_color(self, image_bytes, object_name):
        """
        Returns the dominant color of the object in the image.

        :param image_bytes: The raw bytes of the image.
        :return: A list of detected objects with their associated color.
        """

        color_categories = ["red", "orange", "yellow", "green", "light blue", "dark blue", "purple"]
    
        color_ranges = {
            "red": [([0, 50, 50], [10, 255, 255]), ([160, 50, 50], [180, 255, 255])],  # Red wraps around hue space
            "orange": [([11, 50, 50], [25, 255, 255])],
            "yellow": [([26, 50, 50], [35, 255, 255])],
            "green": [([36, 50, 50], [85, 255, 255])],
            "light blue": [([86, 50, 50], [110, 255, 255])],
            "dark blue": [([111, 50, 50], [130, 255, 255])],
            "purple": [([131, 50, 50], [159, 255, 255])]
        }

        image = vision.Image(content=image_bytes)

        response = self.client.object_localization(image=image)

        objects = response.localized_object_annotations

        pil_image = Image.open(io.BytesIO(image_bytes)).convert("RGB")
        width, height = pil_image.size
        image_np = np.array(pil_image)
        hsv_image = cv2.cvtColor(image_np, cv2.COLOR_RGB2HSV)


        for obj in objects:
            if object_name.lower() in obj.name.lower():
                vertices = obj.bounding_poly.normalized_vertices

                x_coords = [int(vertex.x * width) for vertex in vertices]
                y_coords = [int(vertex.y * height) for vertex in vertices]
                
                min_x, max_x = max(0, min(x_coords)), min(width, max(x_coords))
                min_y, max_y = max(0, min(y_coords)), min(height, max(y_coords))
                
                min_x, max_x = int(min_x*1.1), int(max_x*0.9)
                min_y, max_y = int(min_y*1.1), int(max_y*0.9)

                obj_region = hsv_image[min_y:max_y, min_x:max_x]
                
                if obj_region.size == 0:
                    continue
                
                avg_hsv = np.mean(obj_region, axis=(0, 1))
            
                # Determine the dominant color
                dominant_color = None
                for color, ranges in color_ranges.items():
                    for (lower, upper) in ranges:
                        lower = np.array(lower)
                        upper = np.array(upper)
                        
                        if all(lower <= avg_hsv) and all(avg_hsv <= upper):
                            dominant_color = color
                            break
                    
                    if dominant_color:
                        break
                
                # If no dominant color is found, choose the closest one
                if not dominant_color:
                    hue = avg_hsv[0]
                    min_distance = 180  # Maximum possible hue distance
                    
                    for color in color_categories:
                        for (lower, upper) in color_ranges[color]:
                            middle_hue = (lower[0] + upper[0]) / 2
                            distance = min(abs(hue - middle_hue), 180 - abs(hue - middle_hue))
                            
                            if distance < min_distance:
                                min_distance = distance
                                dominant_color = color
            bounding_box = [min_x, min_y, max_x, max_y]
            return dominant_color, bounding_box

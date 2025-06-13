#!/usr/bin/env python3
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import os

class DeepLabSegmenter:
    def __init__(self, model_path):
        # Initialize TFLite interpreter
        self.interpreter = Interpreter(model_path)
        self.interpreter.allocate_tensors()
        
        # Get model I/O details
        self.input_details = self.interpreter.get_input_details()[0]
        self.output_details = self.interpreter.get_output_details()[0]
        
        # Model-specific parameters
        self.input_shape = self.input_details['shape'][1:3]  # (height, width)
        self.is_quantized = self.input_details['dtype'] == np.uint8

    def preprocess(self, image):
        """Resize and normalize input image"""
        img = cv2.resize(image, (self.input_shape[1], self.input_shape[0]))
        
        if self.is_quantized:
            img = img.astype(np.uint8)
        else:
            img = img.astype(np.float32) / 127.5 - 1  # Normalize to [-1, 1]
            
        return np.expand_dims(img, axis=0)

    def postprocess(self, output, original_shape):
        """Convert raw output to segmentation mask"""
        seg_map = output.squeeze().astype(np.uint8)
        seg_map = cv2.resize(seg_map, (original_shape[1], original_shape[0]), interpolation=cv2.INTER_NEAREST)
        return seg_map

    def color_tolerance_mask(self, image, target_color, tolerance):
        """Create a mask for pixels within the color tolerance range"""
        lower_bound = np.array([max(target_color[0] - tolerance, 0), 
                                max(target_color[1] - tolerance, 0), 
                                max(target_color[2] - tolerance, 0)], dtype=np.uint8)
        upper_bound = np.array([min(target_color[0] + tolerance, 255), 
                                min(target_color[1] + tolerance, 255), 
                                min(target_color[2] + tolerance, 255)], dtype=np.uint8)
        
        mask = cv2.inRange(image, lower_bound, upper_bound)
        return mask

    def visualize_segmentation(self, image, seg_map):
        """Apply color map to segmentation mask"""
        COLORMAP = [
            (0, 0, 0),        # 0: background
            (128, 0, 0),      # 1: aeroplane
            (0, 128, 0),      # 2: bicycle
            (128, 128, 0),    # 3: bird
            # Add more colors per your model's classes
            (168, 57, 64)     # Example color for target segment
        ]
        
        colored_mask = np.zeros((*seg_map.shape, 3), dtype=np.uint8)
        for class_id, color in enumerate(COLORMAP):
            colored_mask[seg_map == class_id] = color
            
        # Overlay on original image
        overlay = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)

        # Apply color tolerance mask to highlight specific color
        specific_color = np.array([168, 57, 64], dtype=np.uint8)
        mask = self.color_tolerance_mask(colored_mask, specific_color, tolerance=10)  # Adjust tolerance as needed
        overlay[mask > 0] = specific_color  # Change color of the specific segment to RGB
        
        return overlay

def main():
    # Configuration
    MODEL_PATH = "/home/root/models/deeplabv3_plus_mobilenet_quantized.tflite"
    INPUT_IMAGE = "/home/root/saved_images/image.jpg"  # 1920x1080 image
    OUTPUT_DIR = "/home/root/segmentation_results"
    
    # Initialize
    segmenter = DeepLabSegmenter(MODEL_PATH)
    original_img = cv2.imread(INPUT_IMAGE)
    
    # Process image
    input_data = segmenter.preprocess(original_img)
    segmenter.interpreter.set_tensor(segmenter.input_details['index'], input_data)
    segmenter.interpreter.invoke()
    raw_output = segmenter.interpreter.get_tensor(segmenter.output_details['index'])
    
    # Post-process
    seg_map = segmenter.postprocess(raw_output, original_img.shape[:2])
    visualization = segmenter.visualize_segmentation(original_img, seg_map)
    
    # Save results
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    cv2.imwrite(f"{OUTPUT_DIR}/input.jpg", original_img)
    cv2.imwrite(f"{OUTPUT_DIR}/segmentation_mask.png", seg_map)
    cv2.imwrite(f"{OUTPUT_DIR}/overlay.jpg", visualization)
    
    print(f"Results saved to {OUTPUT_DIR}")

if __name__ == "__main__":
    main()
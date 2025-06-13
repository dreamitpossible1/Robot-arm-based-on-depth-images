#!/usr/bin/env python3
import cv2
import numpy as np
from tflite_runtime.interpreter import Interpreter
import os

# 1. Initialize model (CPU)
model_path = "/home/root/models/midas_quantized.tflite"
interpreter = Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# 2. Get model I/O details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape'][1:3]  # (height, width)

def preprocess(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (input_shape[1], input_shape[0]))
    img = img.astype(np.uint8)  # Change to UINT8 type
    return np.expand_dims(img, axis=0)  # Add batch dimension

def postprocess(depth, original_shape):
    depth_normalized = (depth - depth.min()) / (depth.max() - depth.min()) * 255
    depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_MAGMA)
    # Resize the colored depth map back to the original image size
    depth_colored_resized = cv2.resize(depth_colored, (original_shape[1], original_shape[0]))
    return depth_colored_resized

# 3. Process image
input_img = cv2.imread("/home/root/saved_images/image.jpg")
original_shape = input_img.shape[:2]  # Get original height and width
input_data = preprocess(input_img)

interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()
depth = interpreter.get_tensor(output_details[0]['index']).squeeze()

# 4. Save results
output_dir = "/home/root/depth_results"
os.makedirs(output_dir, exist_ok=True)
cv2.imwrite(f"{output_dir}/input.jpg", input_img)
cv2.imwrite(f"{output_dir}/depth_colored.jpg", postprocess(depth, original_shape))

print(f"Depth map saved to {output_dir}")

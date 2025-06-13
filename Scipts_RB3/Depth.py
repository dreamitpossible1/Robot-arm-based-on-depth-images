import cv2
import numpy as np
import tensorflow as tf  # Use tflite_runtime on embedded
import os

# Initialize MiDaS
model_path = "/home/root/models/midas_quantized.tflite"  # From Qualcomm AI Hub
interpreter = tf.lite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Input dimensions expected by your specific model
input_shape = input_details[0]['shape'][1:3]  # (height, width)

def preprocess(img):
    """Prepare image for MiDaS"""
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (input_shape[1], input_shape[0]))  # Model's expected size
    img = img.astype(np.float32) / 255.0  # Normalize to [0,1]
    return np.expand_dims(img, axis=0)  # Add batch dimension

def postprocess(depth):
    """Convert raw depth output to visualizable map"""
    depth_normalized = (depth - depth.min()) / (depth.max() - depth.min()) * 255
    return depth_normalized.astype(np.uint8)

# Load your captured image
input_img = cv2.imread("/home/root/saved_images/image.jpg")

# Run inference
input_data = preprocess(input_img)
interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()
depth_output = interpreter.get_tensor(output_details[0]['index'])

# Save results
os.makedirs("/home/root/depth_results", exist_ok=True)
cv2.imwrite("/home/root/depth_results/input.jpg", input_img)

depth_visual = postprocess(depth_output.squeeze())
cv2.imwrite("/home/root/depth_results/depth_raw.jpg", depth_visual)

# Enhanced visualization (color map)
depth_colored = cv2.applyColorMap(depth_visual, cv2.COLORMAP_MAGMA)
cv2.imwrite("/home/root/depth_results/depth_colored.jpg", depth_colored)

print("Depth estimation complete! Results saved in /home/root/depth_results")

# from transformers import AutoModelForVision2Seq, AutoProcessor
# from PIL import Image
# import torch

# import pyrealsense2 as rs
# import numpy as np
# import cv2

# def get_from_realsense():
#     # Initialize RealSense pipeline
#     pipeline = rs.pipeline()
#     config = rs.config()
#     config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#     pipeline.start(config)
#     try:
#         # Wait for a color frame
#         for _ in range(10):  # warm up
#             frames = pipeline.wait_for_frames()
#         frames = pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         color_image = np.asanyarray(color_frame.get_data())
#         # Convert to PIL Image (RGB)
#         color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
#         pil_image = Image.fromarray(color_image)
#         return pil_image
#     finally:
#         pipeline.stop()

# # Load Processor & VLA
# processor = AutoProcessor.from_pretrained("openvla/openvla-7b", trust_remote_code=True)
# vla = AutoModelForVision2Seq.from_pretrained(
#     "openvla/openvla-7b", 
#     attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
#     torch_dtype=torch.bfloat16, 
#     low_cpu_mem_usage=True, 
#     trust_remote_code=True
# ).to("cuda:0")

# # Grab image input & format prompt
# image: Image.Image = get_from_realsense()
# prompt = "In: What action should the robot take to {<INSTRUCTION>}?\nOut:"
import torch
print(torch.cuda.is_available())

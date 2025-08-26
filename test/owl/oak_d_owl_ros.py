#!/usr/bin/python3

import rclpy
from rclpy.node import Node
# from api import *
from std_msgs.msg import Float32MultiArray,Int32,Float32

import sys
import warnings
warnings.filterwarnings('ignore')

try:
    import depthai as dai
    import numpy as np
    import cv2
    import torch
    from transformers import OwlViTProcessor, OwlViTForObjectDetection
except ImportError as e:
    print(f"Error importing required packages: {e}")
    print("Please install required packages using:")
    print("pip install -r requirements.txt")
    sys.exit(1)

# Check NumPy version
np_version = np.__version__
if not (tuple(map(int, np_version.split('.'))) < (1, 25, 0)):
    print(f"Warning: NumPy version {np_version} may cause issues.")
    print("Please install NumPy version <1.25.0:")
    print("pip install 'numpy<1.25.0'")
    sys.exit(1)

# Initialize OwlViT
processor = OwlViTProcessor.from_pretrained("google/owlvit-base-patch32")
model = OwlViTForObjectDetection.from_pretrained("google/owlvit-base-patch32")
# Create pipeline
pipeline = dai.Pipeline()

# Define a color camera node
cam_rgb = pipeline.createColorCamera()
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Create output node for RGB
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")
cam_rgb.video.link(xout_rgb.input)



class OakDOwlNode(Node):
    def __init__(self):
        super().__init__('oak_d_owl_detector')
        
        # Initialize variables
        # self.texts = [["a chair", "a person", "a table", "a bottle"]]
        self.texts = [["a person"]]

        self.device = None
        self.q_rgb = None
        
        # Initialize device
        self.init_oak_d()
        
        # Create timer for processing frames
        self.create_timer(0.01, self.timer_callback)
        
        # Create publishers for detection results
        self.detection_pub = self.create_publisher(Float32MultiArray, '/oak_d_owl/detections', 10)
        
    def init_oak_d(self):
        try:
            self.device = dai.Device(pipeline)
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.get_logger().info("OAK-D camera initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize OAK-D camera: {str(e)}")
            rclpy.shutdown()
            
    def process_frame(self, image, texts):
        # Prepare image for OwlViT
        inputs = processor(images=image, text=texts, return_tensors="pt")
        
        # Perform inference
        with torch.no_grad():
            outputs = model(**inputs)
        
        # Post-process predictions
        target_sizes = torch.Tensor([image.shape[:2]])
        results = processor.post_process_object_detection(outputs, 
                                                        threshold=0.1, 
                                                        target_sizes=target_sizes)[0]
        return results
    
    def timer_callback(self):
        try:
            in_rgb = self.q_rgb.get()
            color_image = in_rgb.getCvFrame()
            
            # Resize frame to fit monitor better (scaled to 720p)
            color_image = cv2.resize(color_image, (1280, 720))

            # Convert BGR to RGB for OwlViT
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            # Process frame with OwlViT
            results = self.process_frame(rgb_image, self.texts)

            # Create message for publishing detections
            detection_msg = Float32MultiArray()
            detection_data = []

            # Draw results and prepare message data
            for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
                box = box.tolist()
                if score >= 0.3:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, box)
                    # Draw on image
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{self.texts[0][label]}: {score:.2f}", 
                            (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                            (0, 255, 0), 2)
                    
                    # Add to message data [label_id, score, x1, y1, x2, y2]
                    detection_data.extend([float(label), float(score), float(x1), float(y1), float(x2), float(y2)])
            
            # Publish detections
            detection_msg.data = detection_data
            self.detection_pub.publish(detection_msg)

            # Show images
            cv2.imshow("OAK-D OwlViT Detection", color_image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info("Shutting down...")
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OakDOwlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__=='__main__':
    main()

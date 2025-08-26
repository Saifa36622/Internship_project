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

# List of objects to detect
texts = [["a chair", "a person", "a table", "a bottle"]]

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

def process_frame(image, texts):
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

try:
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while True:
            # Get frame from OAK-D
            in_rgb = q_rgb.get()
            color_image = in_rgb.getCvFrame()
            
            # Resize frame to fit monitor better (scaled to 720p)
            color_image = cv2.resize(color_image, (1280, 720))

            # Convert BGR to RGB for OwlViT
            rgb_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            
            # Process frame with OwlViT
            results = process_frame(rgb_image, texts)

            # Draw results
            for score, label, box in zip(results["scores"], results["labels"], results["boxes"]):
                box = box.tolist()
                if score >= 0.3:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_image, f"{texts[0][label]}: {score:.2f}", 
                              (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                              (0, 255, 0), 2)

            # Show images
            cv2.imshow("OAK-D OwlViT Detection", color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    cv2.destroyAllWindows()


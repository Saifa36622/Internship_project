import sys
import warnings
warnings.filterwarnings('ignore')

try:
    import pyrealsense2 as rs
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
texts = [["a chair", "a person", "a table", "a bottle",]]

# Create and configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

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
    while True:
        # Get frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

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
        cv2.imshow("OwlViT Detection", color_image)
        cv2.imshow("Depth Image", depth_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

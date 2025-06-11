import pyrealsense2 as rs
import numpy as np
import cv2

# Create a pipeline
pipeline = rs.pipeline()

# Create a config object
config = rs.config()

# Configure the pipeline to stream color and depth data
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline
pipeline.start(config)

try:
    while True:
        # Wait for a coherent set of frames: color and depth
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Show images
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Depth Image", depth_image)

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Stop the pipeline
    pipeline.stop()
    cv2.destroyAllWindows()

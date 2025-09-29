import depthai as dai
import cv2
import numpy as np

def draw_rule_of_thirds(img, color=(0, 255, 0), thickness=1):
    h, w = img.shape[:2]
    # Lines at 1/3 and 2/3 of width and height
    xs = [w // 3, (w * 2) // 3]
    ys = [h // 3, (h * 2) // 3]
    for x in xs:
        cv2.line(img, (x, 0), (x, h), color=color, thickness=thickness)
    for y in ys:
        cv2.line(img, (0, y), (w, y), color=color, thickness=thickness)
    return img

# Create pipeline
pipeline = dai.Pipeline()

# Define a color camera node
cam_rgb = pipeline.createColorCamera()
cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam_rgb.setInterleaved(False)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# Create output node
xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")
cam_rgb.video.link(xout_rgb.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        
        # Resize frame to fit monitor better (scaled to 720p)
        frame = cv2.resize(frame, (1280, 720))
        
        # Draw rule of thirds grid
        frame_with_grid = draw_rule_of_thirds(frame.copy())
        
        # Show original and grid version side by side
        cv2.imshow("OAK-D Color with Rule of Thirds", frame_with_grid)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

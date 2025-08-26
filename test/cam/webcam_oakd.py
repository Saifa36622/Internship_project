import depthai as dai
import cv2

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
        
        cv2.imshow("OAK-D Color", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

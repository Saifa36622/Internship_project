import pyrealsense2 as rs
import numpy as np
import cv2

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
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Fix upside-down frames
        color_image = cv2.rotate(color_image, cv2.ROTATE_180)
        depth_image = cv2.rotate(depth_image, cv2.ROTATE_180)

        # Overlay grid
        draw_rule_of_thirds(color_image)
        # For depth, convert to BGR for colored lines
        depth_bgr = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2BGR)
        draw_rule_of_thirds(depth_bgr)

        cv2.imshow("Color Image with Rule of Thirds", color_image)
        cv2.imshow("Depth Image with Rule of Thirds", depth_bgr)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

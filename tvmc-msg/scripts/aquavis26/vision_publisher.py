#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
import rospy
from std_msgs.msg import String

# ==========================================
TARGET_GATE_COLOR = "Green"
# IMPORTANT: OAK-D requires a compiled .blob file, not a .pt file.
MODEL_BLOB_PATH = "best.blob" 
# ==========================================

def detect_gate_color(crop_img):
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    
    lower_red1, upper_red1 = np.array([0, 70, 50]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 70, 50]), np.array([180, 255, 255])
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), 
                              cv2.inRange(hsv, lower_red2, upper_red2))
                              
    if cv2.countNonZero(mask_green) > cv2.countNonZero(mask_red) and cv2.countNonZero(mask_green) > 50:
        return "Green"
    elif cv2.countNonZero(mask_red) > cv2.countNonZero(mask_green) and cv2.countNonZero(mask_red) > 50:
        return "Red"
    return "Unknown"

def main():
    # Initialize ROS Node
    rospy.init_node('gate_vision_publisher', anonymous=True)
    cmd_pub = rospy.Publisher('/vision/gate_cmd', String, queue_size=1)
    rate = rospy.Rate(10) # 10 Hz

    # Define DepthAI Pipeline
    pipeline = dai.Pipeline()

    # 1. Camera Node
    camRgb = pipeline.create(dai.node.ColorCamera)
    # Updated to 480x480 for higher FPS and matching your new .blob
    camRgb.setPreviewSize(480, 480) 
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camRgb.setInterleaved(False)
    camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.setFps(10)

    # 2. YOLO Detection Network Node (Running on OAK-D)
    detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
    detectionNetwork.setBlobPath(MODEL_BLOB_PATH)
    detectionNetwork.setConfidenceThreshold(0.6)
    detectionNetwork.setIouThreshold(0.5)
    
    # --- YOLOv5 Configuration ---
    detectionNetwork.setNumClasses(1) 
    detectionNetwork.setCoordinateSize(4)
    # NOTE: These are standard 640x640 anchors. If your bounding boxes 
    # look inaccurate, you may need to update these for 480x480.
    detectionNetwork.setAnchors([10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90, 156,198, 373,326])
    detectionNetwork.setAnchorMasks({
        "side52": [0, 1, 2],
        "side26": [3, 4, 5],
        "side13": [6, 7, 8]
    })
    detectionNetwork.setNumInferenceThreads(2)
    detectionNetwork.input.setBlocking(False)

    # 3. Output Links to Host
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb")
    xoutNN = pipeline.create(dai.node.XLinkOut)
    xoutNN.setStreamName("nn")

    # Link nodes
    camRgb.preview.link(detectionNetwork.input)
    camRgb.preview.link(xoutRgb.input)
    detectionNetwork.out.link(xoutNN.input)

    with dai.Device(pipeline) as device:
        # Fetch queues from device
        qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        
        rospy.loginfo(f"Vision Node Started on OAK-D VPU. Targeting {TARGET_GATE_COLOR} Gate.")
        
        while not rospy.is_shutdown():
            inRgb = qRgb.tryGet()
            inDet = qDet.tryGet()
            
            if inRgb is None or inDet is None:
                continue
                
            frame = inRgb.getCvFrame()
            detections = inDet.detections
            
            frame_h, frame_w = frame.shape[:2]
            cam_center_x, cam_center_y = frame_w // 2, frame_h // 2
            
            target_gate_box = None 
            
            # Process detections coming from OAK-D
            for det in detections:
                # OAK-D outputs normalized bounding box coordinates (0.0 to 1.0)
                x1 = int(det.xmin * frame_w)
                y1 = int(det.ymin * frame_h)
                x2 = int(det.xmax * frame_w)
                y2 = int(det.ymax * frame_h)
                
                # Keep within bounds
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(frame_w, x2), min(frame_h, y2)
                
                gate_crop = frame[y1:y2, x1:x2]
                
                # Ensure crop is valid before passing to OpenCV
                if gate_crop.size > 0 and detect_gate_color(gate_crop) == TARGET_GATE_COLOR:
                    target_gate_box = (x1, y1, x2, y2)
                    break 
            
            command = "SEARCH" # Default fallback
            
            if target_gate_box:
                x1, y1, x2, y2 = target_gate_box
                gate_center_x, gate_center_y = (x1 + x2) // 2, (y1 + y2) // 2
                gate_area = (x2 - x1) * (y2 - y1)
                
                tolerance = 40 
                target_area = (frame_w * frame_h) * 0.4 
                
                # STRICT PRIORITY (One DoF at a time)
                if gate_center_y < cam_center_y - tolerance:
                    command = "HEAVE_UP"
                elif gate_center_y > cam_center_y + tolerance:
                    command = "HEAVE_DOWN"
                elif gate_center_x < cam_center_x - tolerance:
                    command = "SWAY_LEFT"
                elif gate_center_x > cam_center_x + tolerance:
                    command = "SWAY_RIGHT"
                elif gate_area < target_area:
                    command = "SURGE_FORWARD"
                else:
                    command = "PASS_THROUGH"
                    
                # Visuals
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (gate_center_x, gate_center_y), 5, (0, 0, 255), -1)

            # Publish to ROS
            cmd_pub.publish(command)
            cv2.putText(frame, f"CMD: {command}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("AUV Vision", frame)
            
            if cv2.waitKey(1) == ord('q'):
                break
                
            rate.sleep()
            
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

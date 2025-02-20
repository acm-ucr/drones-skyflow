import cv2
import numpy as np
from ultralytics import YOLO

# Load YOLOv8 model
model = YOLO('best.pt')  # Path to your trained YOLOv8 model

# Initialize webcam capture (or replace with video file path)
#cap = cv2.VideoCapture(10.42.0.121:7123)  # Use 0 for default webcam, or replace with video file path

#http://10.42.0.121:7123/index.html

url = "http://10.42.0.121:7123/stream"
cap = cv2.VideoCapture(url)

while True:
    # Capture frame from webcam
    ret, frame = cap.read()
    if not ret:
        break

    # Perform inference using YOLOv8
    results = model(frame)

    # Parse the results
    boxes = results[0].boxes  # Get detected bounding boxes

    if len(boxes) > 0:
        # Convert boxes to a list of (x1, y1, x2, y2, area)
        box_list = []
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])  
            area = (x2 - x1) * (y2 - y1)  # Calculate area of the box
            box_list.append((x1, y1, x2, y2, area, box))

        # Identify the largest (closest) bounding box
        largest_box = max(box_list, key=lambda x: x[4])  # Select box with max area

        # Draw the bounding boxes: Red for the closest, Green for others
        for (x1, y1, x2, y2, area, box) in box_list:
            color = (0, 255, 0)  # Default color: Green

            if (x1, y1, x2, y2) == (largest_box[0], largest_box[1], largest_box[2], largest_box[3]):
                color = (0, 0, 255)  # Red for the largest box

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        # Get the center of the largest (closest) bounding box
        x1, y1, x2, y2, _, _ = largest_box
        box_center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

        # Determine movement direction
        frame_width = frame.shape[1]
        if box_center[0] < frame_width // 2:
            direction = "Move Right"
            cv2.arrowedLine(frame, box_center, (box_center[0] + 100, box_center[1]), (0, 0, 255), 5)
        else:
            direction = "Move Left"
            cv2.arrowedLine(frame, box_center, (box_center[0] - 100, box_center[1]), (0, 0, 255), 5)

        # Display movement instruction
        cv2.putText(frame, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("YOLOv8 Detection", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture object and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

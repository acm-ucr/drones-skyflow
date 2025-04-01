import cv2
import numpy as np
import threading
import queue
from ultralytics import YOLO

# Load YOLO model
model = YOLO('bestn.pt')  

# Stream URL
url = "http://10.42.0.121:7123/stream.mjpg"
cap = cv2.VideoCapture(url)

# Frame queue
frame_queue = queue.Queue(maxsize=5)
stop_threads = False

def capture_frames():
    """ Continuously capture frames and add to queue. """
    global stop_threads
    while not stop_threads:
        ret, frame = cap.read()
        if ret:
            if not frame_queue.full():
                frame_queue.put(frame)

def process_frames():
    """ Perform YOLO detection on frames and display in the main thread. """
    global stop_threads
    while not stop_threads:
        if not frame_queue.empty():
            frame = frame_queue.get()

            if frame is None or frame.size == 0:
                continue

            # Run YOLO detection
            results = model(frame)
            boxes = results[0].boxes  

            if len(boxes) > 0:
                box_list = []
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)
                    box_list.append((x1, y1, x2, y2, area, box))

                # Find the largest box
                largest_box = max(box_list, key=lambda x: x[4])

                for (x1, y1, x2, y2, area, box) in box_list:
                    color = (0, 255, 0)  # Green for other boxes
                    if (x1, y1, x2, y2) == (largest_box[0], largest_box[1], largest_box[2], largest_box[3]):
                        color = (0, 0, 255)  # Red for largest box

                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Get center of largest box
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

            # Display frame in the main thread
            cv2.imshow("YOLOv8 Detection", frame)

        # This keeps the window responsive
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_threads = True
            break

    cap.release()
    cv2.destroyAllWindows()

# Start threads
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

# Run processing in the **main thread**
process_frames()

# Cleanup
stop_threads = True
capture_thread.join()

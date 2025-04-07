import cv2
import numpy as np
import threading
import queue
import paho.mqtt.client as mqtt
from ultralytics import YOLO

# MQTT Configuration
MQTT_BROKER = "broker.emqx.io"  # Public MQTT broker
MQTT_PORT = 1883
MQTT_TOPIC = "drone/movement"

# Initialize MQTT Client
client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()

# Load YOLO model
model = YOLO('bestn.pt')  

# Open camera
url = "http://10.42.0.107:7123/stream.mjpg"
cap = cv2.VideoCapture(0)

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
    """ Perform YOLO detection on frames and send MQTT commands. """
    global stop_threads
    while not stop_threads:
        if not frame_queue.empty():
            frame = frame_queue.get()

            if frame is None or frame.size == 0:
                continue

            # Run YOLO detection
            results = model(frame)
            boxes = results[0].boxes  
            results = model(frame, verbose=False)  # Disable logs
            # if results[0].boxes:  # Only print if detections exist
            #     print(results)

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
                    command = "RIGHT"
                    cv2.arrowedLine(frame, box_center, (box_center[0] + 100, box_center[1]), (0, 0, 255), 5)
                else:
                    direction = "Move Left"
                    command = "LEFT"
                    cv2.arrowedLine(frame, box_center, (box_center[0] - 100, box_center[1]), (0, 0, 255), 5)

                # Publish command to MQTT
                try:
                    if command!=oldcommand:
                        oldcommand=command
                        client.publish(MQTT_TOPIC, oldcommand)
                except:
                    oldcommand=command

                # Display movement instruction
                cv2.putText(frame, direction, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Display frame in the main thread
            cv2.imshow("YOLOv8 Detection", frame)

        # Keep window responsive
        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_threads = True
            break

    cap.release()
    cv2.destroyAllWindows()
    client.loop_stop()
    client.disconnect()

# Start threads
capture_thread = threading.Thread(target=capture_frames, daemon=True)
capture_thread.start()

# Run processing in the **main thread**
process_frames()

# Cleanup
stop_threads = True
capture_thread.join()

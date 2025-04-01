# This code runs on the rasp pi 0 with the IMX219 Arducam. 

import io
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput
import logging
import socketserver
import sys
import pickle
import struct ### new code
from threading import Condition
from http import server
#from paho.mqtt import client as mqtt_client

PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""

broker = 'broker.emqx.io'
port = 1883
topic = "Drone Commands"
# Generate a Client ID with the subscribe prefix.
client_id = f'subscribe-{"jared"}'

# def connect_mqtt() -> mqtt_client:
#     def on_connect(client, userdata, flags, rc):
#         if rc == 0:
#             print("Connected to MQTT Broker!")
#         else:
#             print("Failed to connect, return code %d\n", rc)

#     client = mqtt_client.Client(client_id)
#     # client.username_pw_set(username, password)
#     client.on_connect = on_connect
#     client.connect(broker, port)
#     return client


# def subscribe(client: mqtt_client):
#     def on_message(client, userdata, msg):
#         print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

#     client.subscribe(topic)
#     client.on_message = on_message

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))

picam2.set_controls({"AfMode": 2})  # 1 = Auto, 2 = Continuous, 0 = Manual

output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))


def run(): 
    # client = connect_mqtt()
    # subscribe(client)
    # client.loop_start()

    print("Entered camera loop")
    try:
        address = ('', 7123)
        server = StreamingServer(address, StreamingHandler)
        server.serve_forever()
    finally:
        picam2.stop_recording()
        # client.loop_end()

if __name__== '__main__':
    run()
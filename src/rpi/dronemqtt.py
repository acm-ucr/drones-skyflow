import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = "broker.emqx.io"  
MQTT_PORT = 1883
MQTT_TOPIC = "drone/movement"

def on_message(client, userdata, message):
    command = message.payload.decode()
    print(f"Received command: {command}")

    if command == "LEFT":
        # Add drone control code to move left
        print("Moving Left")
    elif command == "RIGHT":
        # Add drone control code to move right
        print("Moving Right")

# Initialize MQTT Client
client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.subscribe(MQTT_TOPIC)

print(f"Listening for drone commands on topic: {MQTT_TOPIC}")
client.loop_forever()

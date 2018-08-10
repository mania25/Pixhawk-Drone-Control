import paho.mqtt.client as mqtt
import time

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("topic-3851330d4cfad095ef2f0c23be43d75e")
    blink()

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

def on_publish(client, userdata, msg):
    print("msg: " + str(msg))

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def blink():
    print("PUBLISHING DATA")
    client.publish("iot/live","device-3851330d4cfad095ef2f0c23be43d75e")
    time.sleep(1)
    return

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.on_subscribe = on_subscribe
client.username_pw_set("bbff39d0d3066758ffe55666762b3c8b150295b848cb6c871b79f2fff36c79fb", "50acea3098359517297e08040dc6bfc371d044190be6527c1ac29e078cbe8313")

client.connect("147.75.93.178", 1883, 60)

client.loop_forever()
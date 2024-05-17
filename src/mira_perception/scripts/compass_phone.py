import paho.mqtt.client as mqtt

broker_address = "192.168.214.33"  # Replace with the broker address
port = 1883
topic = "compass"

def on_message(client, userdata, message):
    print("Received compass data:", message.payload.decode())

client = mqtt.Client()
client.on_message = on_message

client.connect(broker_address, port)

client.subscribe(topic)
client.loop_forever()
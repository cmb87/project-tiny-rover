import paho.mqtt.client as paho

import logging
import time
import json
# =============================================
# Set logging formats
logging.basicConfig(
    level=logging.INFO,
    format=("[%(filename)8s] [%(levelname)4s] :  %(funcName)s - %(message)s"),
)

# =============================================
# Initialize Thermometer

#dhtDevice = adafruit_dht.DHT11(board.D2) #  GPIO

# =============================================
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("esp/esp32cam-drone1/status")

    print("Subscribed!")

def on_publish(client,userdata,result):             #create function for callback
    print("published!")


BROKER = config.server["mqttbroker"]
PORT = config.server["mqttport"]

logging.info("Connecting to broker")


client = paho.Client()
client.username_pw_set(<user>, <pw>)
client.on_connect = on_connect
client.on_publish = on_publish   
client.on_message = on_message



client.connect(BROKER, PORT)
                       #assign function to callback
client.loop_start()     
logging.info("Starting Tank")


while True:
    
    val = input("Please enter a string:\n")
    
    if val == "p":
        logging.info("Photo")
        ret = client.publish("esp/esp32cam-drone1/photo", "1") #publish  {timestamp}
        print(ret)

    elif val == "k":
        logging.info("stop")
        ret = client.publish("esp/esp32cam-drone1/stop", "323") #publish  {timestamp}

    elif val == "w":
        logging.info("forward")
        ret = client.publish("esp/esp32cam-drone1/forward", "323") #publish  {timestamp}

    elif val == "s":
        logging.info("backward")
        ret = client.publish("esp/esp32cam-drone1/backward", "323") #publish  {timestamp}

    elif val == "a":
        logging.info("left")
        ret = client.publish("esp/esp32cam-drone1/left", "323") #publish  {timestamp}

    elif val == "d":
        logging.info("right")
        ret = client.publish("esp/esp32cam-drone1/right", "323") #publish  {timestamp}


    time.sleep(0.5)
pahop

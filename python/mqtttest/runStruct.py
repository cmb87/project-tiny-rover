import paho.mqtt.client as paho
import struct
import logging
import time
import json
import random
import cv2
import numpy as np
# =============================================
# Set logging formats
logging.basicConfig(
    level=logging.INFO,
    format=("[%(filename)8s] [%(levelname)4s] :  %(funcName)s - %(message)s"),
)

# =============================================
class DriveStruct(object):
    x1 = 0
    x2 = 0
    img = np.zeros((480,640,3),dtype=np.uint8)

control = DriveStruct()
droneid = "esp32cam-drone1"
# =============================================
# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.payload))
    nparr = np.frombuffer(msg.payload, np.uint8)
    control.img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    print("img received")


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("esp32/cam_0")

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
    
    if control.img is not None:
        cv2.imshow('Demo', control.img) 

    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break


    # val = input("Please enter a string:\n")
    
    # if val == "p":

    #     message = [random.randint(1000, 2000), random.randint(1000, 2000)]
    #     print(message)
    #     buf = struct.pack('i' * len(message), *message)

    #     ret = client.publish(f"esp/{droneid}/control", buf) #publish  {timestamp}

    # if val == "l":
    #     ret = client.publish(f"esp/{droneid}/light", "1") #publish  {timestamp}


cv2.destroyAllWindows()
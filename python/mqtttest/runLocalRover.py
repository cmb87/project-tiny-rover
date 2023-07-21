import pygame
from time import sleep
import paho.mqtt.client as paho
import struct
import numpy as np
import logging

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

control = DriveStruct()

droneid = "esp32cam-drone1"

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


# =============================================
logging.info("Starting PiGame")
pygame.init()
window = pygame.display.set_mode((100, 100))

clock = pygame.time.Clock()
font = pygame.font.SysFont('Comic Sans MS', 30)

rect = pygame.Rect(0, 0, 20, 20)
rect.center = window.get_rect().center
vel = 2
run = True

# =========================

try:
    logging.info("Starting Control Loop")

        
    while run:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
            # print(pygame.key.name(event.key))
                pass

        keys = pygame.key.get_pressed()
        
        steer = 500*(keys[pygame.K_RIGHT] - keys[pygame.K_LEFT])
        gas = 500*(keys[pygame.K_DOWN] - keys[pygame.K_UP])
            
        
        control.x1 = int(np.clip(1500 + gas + steer, 1000, 2000))
        control.x2 = int(np.clip(1500 + gas - steer, 1000, 2000))
        
        msg = [control.x1, control.x2]
        buf = struct.pack('i' * len(msg), *msg)

        ret = client.publish(f"esp/{droneid}/control", buf) #publish  {timestamp}

        print(steer, gas, msg)

except KeyboardInterrupt:
    pass

pygame.quit()
exit()
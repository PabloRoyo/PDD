import json
import threading

import paho.mqtt.client as mqtt
import time
def on_connect(client, userdata, flags, rc):
    global connected
    if rc==0:
        print("connected OK Returned code=",rc)
        connected = True
    else:
        print("Bad connection Returned code=",rc)



def on_message(client, userdata, message):
    global estado
    global destinoAlcanzado

    if message.topic == 'controlador/consola/connected':
        estado = 'connected'
    if message.topic == 'controlador/consola/armed':
        estado = 'armed'
    if message.topic == 'controlador/consola/flying':
        estado = 'flying'
    if message.topic == 'controlador/consola/landed':
        estado = 'landed'


broker_address = "broker.hivemq.com"
broker_port = 1883

client = mqtt.Client("consola")

client.on_connect = on_connect
client.on_message = on_message

client.connect(broker_address, broker_port)
client.subscribe('controlador/consola/#')
client.loop_start()

print ('empiezo')
estado = 'disconnected'

client.publish('consola/controlador/connect')
while estado != 'connected':
    pass

client.publish('consola/controlador/arm')
while estado != 'armed':
    pass
print ('armado')

client.publish('consola/controlador/takeOff')
while estado != 'flying':
    pass
print ('volando')


time.sleep (5)

print ('empezamos a navegar')
client.publish('consola/controlador/startGo')
print ('vamos al norte durante 15 segundos')
client.publish('consola/controlador/go', 'North')
time.sleep (15)
client.publish('consola/controlador/stopGo')
print ('vuelve a casa')
client.publish('consola/controlador/RTL')
while estado != 'landed':
    pass

print ('en tierra')


while True:
    pass
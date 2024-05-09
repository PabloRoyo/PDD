import threading
import time

import paho.mqtt.client as mqtt
import json
from Dron import Dron


def publish_telemetry_info (telemetry_info):
    global sending_topic, client
    client.publish(sending_topic + '/telemetryInfo', json.dumps(telemetry_info))
def publish_parameters (parameters):
    global sending_topic, client
    print ('los publico en '+ sending_topic + '/parameters')

    client.publish(sending_topic + '/parameters', json.dumps(parameters))

def publish_event (event):
    global sending_topic, client
    client.publish(sending_topic + '/'+event)
    print ('he publicado: ',sending_topic + '/'+event )



def on_message(cli, userdata, message):

    global  sending_topic, client
    global dron

    splited = message.topic.split("/")
    origin = splited[0] # aqui tengo el nombre de la aplicación que origina la petición
    command = splited[2] # aqui tengo el comando

    sending_topic = "controlador/" + origin # lo necesitaré para enviar las respuestas

    if command == 'connect':
        print ('vamos a conectar')
        connection_string = 'tcp:127.0.0.1:5763'
        baud = 115200
        dron.connect(connection_string, baud)
        publish_event('connected')

    if command == 'arm':
        dron.arm()
        publish_event('armed')

    if command == 'takeOff':
        dron.takeOff (5, blocking=False,  callback=publish_event, params='flying')
        print (" ya he ordenado despegar")

    if command == 'RTL':
        dron.RTL(blocking=False,  callback=publish_event, params='landed')
        print (" ya he ordenado retornar")


    if command == 'startGo':
        dron.startGo()
        print ('Preparado para navegar')

    if command == 'stopGo':
        dron.stopGo()
        print ('Se acabó la navegación')

    if command == 'go':
        direction = message.payload.decode("utf-8")
        print ('vamos a ', direction)
        dron.go(direction)


def on_connect(client, userdata, flags, rc):
    global connected
    if rc==0:
        print("connected OK Returned code=",rc)
        connected = True
    else:
        print("Bad connection Returned code=",rc)


broker_address = "broker.hivemq.com"
broker_port = 1883

client = mqtt.Client("controlador")
dron = Dron()
client.on_message = on_message
client.on_connect = on_connect

client.connect(broker_address, broker_port)
client.subscribe('+/controlador/#')
print ('controlador esperando peticiones')
client.loop_forever()


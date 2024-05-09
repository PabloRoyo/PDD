import json
import math
import threading
import time
from pymavlink import mavutil
from modules.dron_goto import _goto, goto
from modules.dron_RTL_Land import _goDown
from modules.dron_arm import _arm
from modules.dron_takeOff import  _takeOff



def upload_mission(self,  wploader: list) -> bool:
    """
    Upload the mission to the UAV.

    Args:
        master (mavutil.mavlink_connection): The MAVLink connection to use.
        wploader (list): The waypoint loader list.

    Returns:
        bool: True if the mission is successfully uploaded, False otherwise.
    """
    # Clear any existing mission from vehicle
    print('Clearing mission')
    self.vehicle.mav.mission_clear_all_send(self.vehicle.target_system, self.vehicle.target_component)

    if not self.verify_ack('Error clearing mission'):
        return False

    # Send waypoint count to the UAV
    self.vehicle.waypoint_count_send(len(wploader))

    # Upload waypoints to the UAV
    return self.send_waypoints(wploader)


def verify_ack(self, error_msg: str) -> bool:
    """
    Verifies the ack response.

    Args:
        master (mavutil.mavlink_connection): The MAVLink connection to use.
        error_msg (str): The error message to log if ack verification fails.

    Returns:
        bool: True if ack verification successful, False otherwise.
    """
    ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
    if ack.type != 0:
        print(f'{error_msg}: {ack.type}')
        return False
    return True


def send_waypoints(self, wploader: list) -> bool:
    """
    Send the waypoints to the UAV.

    Args:
        master (mavutil.mavlink_connection): The MAVLink connection to use.
        wploader (list): The waypoint loader list.

    Returns:
        bool: True if waypoints are successfully sent, False otherwise.
    """
    for i in range(len(wploader)):
        msg = self.vehicle.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=3)
        if not msg:
            print('No waypoint request received')
            return False
        print(f'Sending waypoint {msg.seq}/{len(wploader) - 1}')
        self.vehicle.mav.send(wploader[msg.seq])

        if msg.seq == len(wploader) - 1:
            break

    return self.verify_ack('Error uploading mission')



def ack (self, keyword):
    print ('***** -> ' + str (self.vehicle.recv_match(type = keyword, blocking = True)))


def _executeMission(self, mission, callback=None, params = None):
    '''La mision debe especificarse en json, de acuerdo con el formato de este ejemplo:
    {
        "takeOffAlt": 5,
        "waypoints":
            [
                {
                    'lat': 41.2763410,
                    'lon': 1.9888285,
                    'alt': 12
                },
                {
                    'lat': 41.27623,
                    'lon': 1.987,
                    'alt': 14
                }
            ]

    }
    El dron armará, despegara hasta la altura indicada, navegará por los waypoints y acabará
    con un RTL
    '''


    takOffAlt = mission['takeOffAlt']

    waypoints =  mission['waypoints']
    print ('waypoints ', waypoints)

    # preparamos la misión para cargarla en el dron

    wploader = []
    seq = 0  # Waypoint sequence begins at 0
    # El primer wp debe ser la home position.
    # Averiguamos la home position
    self.vehicle.mav.command_long_send(
        self.vehicle.target_system,
        self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)

    msg = self.vehicle.recv_match(type='HOME_POSITION', blocking=True)
    msg = msg.to_dict()
    lat = msg['latitude']
    lon = msg['longitude']
    alt = msg['altitude']

    # añadimos este primer waypoint a la mision
    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        0, 0, seq, 0, 16, 0, 0, 0, 0, 0, 0,
        lat,
        lon,
        alt
    ))
    seq = 1

    # El siguiente elemento de la mision debe ser el comando de takeOff, en el que debemos indicar una posición
    # que será también la home position

    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        self.vehicle.target_system, self.vehicle.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, True,
        0, 0, 0, 0,
        lat, alt, takOffAlt
    ))
    seq = 2

    # Ahora añadimos los waypoints de la ruta
    for wp in waypoints:
        wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            self.vehicle.target_system, self.vehicle.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, True,
            0,0,0,0,
            int(wp["lat"] * 10 ** 7), int(wp["lon"]* 10 ** 7), int(wp["alt"])
        ))
        seq += 1  # Increase waypoint sequence for the next waypoint
    # añadimos para acabar un RTL
    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        self.vehicle.target_system, self.vehicle.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, True,
        0, 0, 0, 0,
        0,0,0
    ))
    print ('lo tengo')

    self.upload_mission (wploader)


    '''# borramos la mision que pueda tener el autopiloto en este momento
    print('Clearing mission')
    self.vehicle.mav.mission_clear_all_send(self.vehicle.target_system, self.vehicle.target_component)
    ack = self.vehicle.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
 

    print ('limpiado')

    # Indicamos el número de items de la misión
    self.vehicle.waypoint_count_send(len(wploader))
    print ('enviaod el contador')

    # Y ahora enviamos los items a medida que nos los pida
    for i in range(len(wploader)):
        msg = self.vehicle.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True, timeout=3)
        if not msg:
            print('No waypoint request received')
            return False
        print(f'Sending waypoint {msg.seq}/{len(wploader) - 1}')
        self.vehicle.mav.send(wploader[msg.seq])

        if msg.seq == len(wploader) - 1:
            break'''
    # Finalmente ejecutamos la misión
    self.state = 'arming'
    self._arm()
    self.vehicle.mav.command_long_send(
        self.vehicle.target_system,
        self.vehicle.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0)
    # Esperamos hasta que la mision acabe
    time.sleep(10)
    while True:
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            msg = msg.to_dict()
            alt = float(msg['relative_alt'] / 1000)
            print(alt)
            if alt < 0.5:
                break
            time.sleep(2)

    if callback != None:
        if self.id == None:
            if params == None:
                callback()
            else:
                callback(params)
        else:
            if params == None:
                callback(self.id)
            else:
                callback(self.id, params)



def executeMission2 (self):
    '''connection_string = 'tcp:127.0.0.1:5763'
    baud = 115200
    the_connection = mavutil.mavlink_connection(connection_string, baud)
    the_connection.wait_heartbeat()'''
    print ('vamos')
    #print(f"Heartbeat from system (system {the_connection.target_system} component {the_connection.target_component})")
    the_connection = self.vehicle
    mission = {
        "takeOffAlt": 5,
        "waypoints":
            [
                {
                    'lat': 41.2763410,
                    'lon': 1.9888285,
                    'alt': 12
                },
                {
                    'lat': 41.2762977,
                    'lon': 1.9885401,
                    'alt': 14
                }
            ]

    }

    takOffAlt = mission['takeOffAlt']

    waypoints = mission['waypoints']
    print (waypoints)

    # preparamos la misión para cargarla en el dron

    wploader = []
    seq = 0  # Waypoint sequence begins at 0
    # El primer wp debe ser la home position.
    # Averiguamos la home position
    self.vehicle.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_GET_HOME_POSITION,
        0, 0, 0, 0, 0, 0, 0, 0)
    print ('pido home')

    msg = self.vehicle.recv_match(type='HOME_POSITION', blocking=True)
    msg = msg.to_dict()
    lat = msg['latitude']
    lon = msg['longitude']
    alt = msg['altitude']
    print (' ya teng home')

    # añadimos este primer waypoint a la mision
    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        0, 0, seq, 0, 16, 0, 0, 0, 0, 0, 0,
        lat,
        lon,
        alt
    ))
    seq = 1

    # El siguiente elemento de la mision debe ser el comando de takeOff, en el que debemos indicar una posición
    # que será también la home position

    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        the_connection.target_system, the_connection.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, True,
        0, 0, 0, 0,
        lat, alt, takOffAlt
    ))
    seq = 2

    # Ahora añadimos los waypoints de la ruta
    for wp in waypoints:
        wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
            the_connection.target_system, the_connection.target_component, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, True,
            0, 0, 0, 0,
            int(wp["lat"] * 10 ** 7), int(wp["lon"] * 10 ** 7), int(wp["alt"])
        ))
        seq += 1  # Increase waypoint sequence for the next waypoint
    # añadimos para acabar un RTL
    wploader.append(mavutil.mavlink.MAVLink_mission_item_int_message(
        the_connection.target_system, the_connection.target_component, seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, True,
        0, 0, 0, 0,
        0, 0, 0
    ))
    print ('ya esta cargado')
    print('Clearing mission')
    the_connection.mav.mission_clear_all_send( the_connection.target_system,  the_connection.target_component)

    ack = the_connection.recv_match(type='MISSION_ACK', blocking=True)
    print(ack)
    ''' if ack.type != 0:
        print('error cargando la mision')
        return False'''

    # Send waypoint count to the UAV
    the_connection.waypoint_count_send(len(wploader))

    # Upload waypoints to the UAV

    for i in range(len(wploader)):

        msg = the_connection.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST'], blocking=True)

        print(f'Sending waypoint {msg.seq}/{len(wploader) - 1}')
        the_connection.mav.send(wploader[msg.seq])

        if msg.seq == len(wploader) - 1:
            break
    print ('espero ack')

    ack = the_connection.recv_match(type='MISSION_ACK', blocking=True)

    print(ack)
    '''if ack.type != 0:
        print('error cargando la mision')
        return False'''
    print ('vamos')

    mode = 'GUIDED'
    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    arm_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    the_connection.motors_armed_wait()

    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0)





def executeMission(self,flightPlan, blocking=True, callback=None, params = None):
    if blocking:
        self.executeMission2()
    else:
        y = threading.Thread(target=self.executeMission2)
        #y = threading.Thread(target=self.executeMission2, args=[flightPlan, callback, params])
        y.start()
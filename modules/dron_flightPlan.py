import json
import math
import threading
import time
from pymavlink import mavutil
from modules.dron_goto import _goto, goto
from modules.dron_RTL_Land import _goDown
from modules.dron_arm import _arm
from modules.dron_takeOff import  _takeOff



def _executeFlightPlan(self, flightPlan, callback=None, params = None):
    altitude = 6
    waypoints = json.loads(flightPlan)
    print ('vaypoints: ', waypoints)

    self.state = 'arming'
    self._arm()
    print ('armado')

    self._takeOff(altitude)
    print ('vamos a empezar')


    for wp in waypoints [0:]:
        self._goto(float(wp['lat']),float(wp['lon']), float(wp['alt']))
        print('reached')

    print ('empezamos RTL')
    self.state = 'retornando'
    self._goDown ('RTL')
    self.state = 'conectado'
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



def executeFlightPlan(self,flightPlan, blocking=True, callback=None, params = None):
    if blocking:
        self._executeFlightPlan(flightPlan)
    else:
        y = threading.Thread(target=self._executeFlightPla, args=[flightPlan, callback, params])
        y.start()
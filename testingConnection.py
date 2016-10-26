
if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"
import socket
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import json
import argparse
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from optparse import OptionParser
import threading
from twisted.internet import task
#global variable
last_attitude_cache = None
last_location_cache = None
last_gps_cache = None
last_battery_cache = None
last_velocity_cache = None 
last_heading_cache = None 
last_systemstatus_cache = None
usrb_object = None
sample_rate = 1000000
base_freq = None
#-------------Class for Pixhaw Obj ------------
class BaseJsonObj2:
    def jdefault(o):
        if isinstance(o, set):
            return list(o)
        return o.__dict__

    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)
#-------------End class for Pixhaw Obj ------------
#-------------Class for Pixhaw Obj ------------
class BaseJsonObj:

    def __init__(self):
        print ''
    def jdefault(o):
        if isinstance(o, set):
            return list(o)
        return o.__dict__

    def to_JSON(self):
        return json.dumps(self, default=lambda o: o.__dict__,
                          sort_keys=True, indent=4)
#-------------End class for Pixhaw Obj ------------

class DroneServer(DatagramProtocol):
    vehicle = None
   
    isDroneKitStreaming = False
    DroneKitHost = ""
    DroneKitPort = ""
    
    countRotate = 0
    loopCall = None
    rotateRelative = None
    rotateHeading = None
    rotateIsHolding = False
    rotateIsExecuting = False
    global usrb_object 
    global gnu_message
    
    def velocity_callback(droneServerObj,self,attr_name, value):
        global last_velocity_cache
        if value != last_velocity_cache:
            print ' CALLBACK: velocity changed to', value           
            last_velocity_cache = value
    def status_callback(droneServerObj,self,attr_name, value):
        global last_systemstatus_cache
        if value != last_systemstatus_cache:
            print ' CALLBACK: status changed to', value            
            last_systemstatus_cache = value        
    def heading_callback(droneServerObj,self,attr_name, value):
        global last_heading_cache
        if value != last_heading_cache:
            print ' CALLBACK: heading changed to', value
            last_heading_cache = value
    def battery_callback(droneServerObj,self,attr_name, value):
        global last_battery_cache
        if value != last_battery_cache:
            print ' CALLBACK: battery changed to', value           
    def gps0_callback(droneServerObj,self,attr_name, value):
        global last_gps_cache
        if value != last_gps_cache:
            print ' CALLBACK: gps changed to', value
            last_gps_cache = value        
    def location_callback(droneServerObj,self,attr_name, value):
        global last_location_cache
        if value != last_location_cache:
            print ' CALLBACK: location changed to', value
            last_location_cache = value
    def attitude_callback(droneServerObj,self,attr_name, value):
        global last_attitude_cache
        if value != last_attitude_cache:
            print ' CALLBACK: ATTITUDE changed to', value
            last_attitude_cache = value      
    def startProtocol(self):        
        try:
            print "Started application"
            print "Init dronekit object" 
            #connection_string = None
            #connection_string = 'com3' #window
            #connection_string = '/dev/cu.usbmodem1' #macos
            connection_string = '/dev/ttyACM0' #ubuntu
            sitl = None
            #Start SITL if no connection string specified
            if not connection_string:
                import dronekit_sitl
                sitl = dronekit_sitl.start_default()
                connection_string = sitl.connection_string()
            #Connect to vehicle
            print '\nConnecting to vehicle on: %s' % connection_string
            vehicle = connect(connection_string,wait_ready=True,baud=57600)
            vehicle.wait_ready('autopilot_version')         
    
            self.vehicle = vehicle
           
            #mapping attribute change
            self.vehicle.add_attribute_listener('velocity', self.velocity_callback)
            self.vehicle.add_attribute_listener('battery', self.battery_callback)
            self.vehicle.add_attribute_listener('gps_0', self.gps0_callback)               
            self.vehicle.add_attribute_listener('location.global_frame',self.location_callback)
            self.vehicle.add_attribute_listener('system_status', self.status_callback)
            self.vehicle.add_attribute_listener('heading', self.heading_callback)
            self.vehicle.add_attribute_listener('attitude', self.attitude_callback)
            
           
        except [[KeyboardInterrupt]]:            
            self.vehicle.remove_attribute_listener('velocity', self.velocity_callback)
            self.vehicle.remove_attribute_listener('battery', self.battery_callback)
            self.vehicle.remove_attribute_listener('gps_0', self.gps0_callback)               
            self.vehicle.remove_attribute_listener('location.global_frame',self.location_callback)
            self.vehicle.remove_attribute_listener('system_status', self.status_callback)
            self.vehicle.remove_attribute_listener('heading', self.heading_callback)
            self.vehicle.remove_attribute_listener('attitude', self.attitude_callback)
            self.vehicle.close()
            pass
    
    def datagramReceived(self, datagram, (host, port)):
        #print "Datagram %s received from %s : %s" % (repr(datagram), host,port)
        udpObj = json.loads(datagram)
        if udpObj['Command'] == "Start":
            print "Start stream data"        


droneServer = DroneServer() 
reactor.listenUDP(8891, droneServer)
reactor.run()
            

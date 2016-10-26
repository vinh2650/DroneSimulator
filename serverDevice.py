
if __name__ == '__main__':
    import ctypes
    import sys
    if sys.platform.startswith('linux'):
        try:
            x11 = ctypes.cdll.LoadLibrary('libX11.so')
            x11.XInitThreads()
        except:
            print "Warning: failed to XInitThreads()"
import numpy as np
import socket
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import json
import argparse
from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor
from gnuradio import uhd
from gnuradio import analog
from gnuradio import blocks
from gnuradio import eng_notation
from gnuradio import gr
from gnuradio import wxgui
from gnuradio.eng_option import eng_option
from gnuradio.fft import window
from gnuradio.filter import firdes
from gnuradio.wxgui import fftsink2
import nois_global_variable
from gnuradio.wxgui import forms
from grc_gnuradio import wxgui as grc_wxgui
from optparse import OptionParser
import wx
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
    pho = None
   
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
            #print ' CALLBACK: velocity changed to', value
            droneServerObj.pho.Vehicle['Velocity'] = value
            droneServerObj.sendDataToClient()
            last_velocity_cache = value
    def status_callback(droneServerObj,self,attr_name, value):
        global last_systemstatus_cache
        if value != last_systemstatus_cache:
            #print ' CALLBACK: status changed to', value
            droneServerObj.pho.SystemStatus = value
            droneServerObj.sendDataToClient()
            last_systemstatus_cache = value        
    def heading_callback(droneServerObj,self,attr_name, value):
        global last_heading_cache
        if value != last_heading_cache:
            #print ' CALLBACK: heading changed to', value
            droneServerObj.pho.Vehicle['Heading'] = value
            droneServerObj.sendDataToClient()
            last_heading_cache = value
    def battery_callback(droneServerObj,self,attr_name, value):
        global last_battery_cache
        if value != last_battery_cache:
            #print ' CALLBACK: battery changed to', value
            droneServerObj.pho.Vehicle['Battery'] = {'Voltage': value.voltage,
                    'Current': value.current,
                    'Level': value.level}
            droneServerObj.sendDataToClient()
            last_battery_cache = value
    def gps0_callback(droneServerObj,self,attr_name, value):
        global last_gps_cache
        if value != last_gps_cache:
            #print ' CALLBACK: gps changed to', value
            droneServerObj.pho.GpsInfo = {
                'Eph': value.eph,
                'Epv': value.epv,
                'Fix_Type': value.fix_type,
                'Satellites_Visible': value.satellites_visible,
                }
            droneServerObj.sendDataToClient()
            last_gps_cache = value        
    def location_callback(droneServerObj,self,attr_name, value):
        global last_location_cache
        if value != last_location_cache:
            #print ' CALLBACK: location changed to', value
            droneServerObj.pho.LocationGlobal = {'Lat': value.lat, 'Lon': value.lon,
                                    'Alt': value.alt}
            droneServerObj.sendDataToClient()
            last_location_cache = value
    def attitude_callback(droneServerObj,self,attr_name, value):
        global last_attitude_cache
        if value != last_attitude_cache:
            #print ' CALLBACK: ATTITUDE changed to', value
            droneServerObj.pho.Attitude = {
                'Yaw': value.yaw,
                'Pitch': value.pitch,
                'Roll': value.roll    
                }
            droneServerObj.sendDataToClient()
            last_attitude_cache = value        
            
    def sendDataToClient(self):
        if self.isDroneKitStreaming == True:
            dataSend = BaseJsonObj2()
            dataSend.DroneData = {
                'Drone': str(self.pho.to_JSON())
                }
            #dataSend.GnuData = {
            #    'Gnu' : str(self.gnu_message)
            #    }
            self.transport.write(str(dataSend.to_JSON()), (self.DroneKitHost, 5591))
        #else:
            #print "just record data, not streaming"
    def broadcastGnuMessage(self,message):
        self.gnu_message = message 
        self.transport.write(message, (self.DroneKitHost, 5590))
    def executeRotate(self):
        if self.rotateIsHolding is True:
            return
	print 'rotate'
        self.countRotate +=1
        if self.rotateRelative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            self.rotateHeading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)        
        if self.countRotate == 18:
            self.loopCall.stop()
            self.countRotate = 0
            self.rotateIsHolding = False
	    self.rotateIsExecuting = False
    def startRotateDevice(self,heading, relative=False):
        self.rotateHeading = heading
        self.rotateRelative = relative
	self.loopCall = task.LoopingCall(self.executeRotate)
        self.loopCall.start(2.0) # call every 2 seconds
    def stopRotateDevice(self):
	if self.rotateIsExecuting is True:
            self.loopCall.stop()
	    self.rotateIsExecuting = False
	    self.rotateIsHolding = False
    def unholdRotateDevice(self):
	self.rotateIsHolding = False	
    def holdRotateDevice(self):
        self.rotateIsHolding = True
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
            #init pixhaw obj
            self.pho = BaseJsonObj()
            self.pho.Vehicle = {'Velocity': self.vehicle.velocity, 'Heading': self.vehicle.heading,
                  'Battery': {'Voltage': self.vehicle.battery.voltage,
                  'Current': self.vehicle.battery.current,
                  'Level': self.vehicle.battery.level}}
            self.pho.LocationGlobal = {'Lat': self.vehicle.location.global_frame.lat,
                          'Lon': self.vehicle.location.global_frame.lon,
                          'Alt': self.vehicle.location.global_frame.alt}
            self.pho.GpsInfo = {
                'Eph': self.vehicle.gps_0.eph,
                'Epv': self.vehicle.gps_0.epv,
                'Fix_Type': self.vehicle.gps_0.fix_type,
                'Satellites_Visible': self.vehicle.gps_0.satellites_visible,
                    }
            self.pho.SystemStatus = {
                'State': self.vehicle.system_status.state    
                }
            self.pho.Attitude = {
                'Yaw': self.vehicle.attitude.yaw,
                'Pitch': self.vehicle.attitude.pitch,
                'Roll': self.vehicle.attitude.roll    
                }
            #mapping attribute change
            self.vehicle.add_attribute_listener('velocity', self.velocity_callback)
            self.vehicle.add_attribute_listener('battery', self.battery_callback)
            self.vehicle.add_attribute_listener('gps_0', self.gps0_callback)               
            self.vehicle.add_attribute_listener('location.global_frame',self.location_callback)
            self.vehicle.add_attribute_listener('system_status', self.status_callback)
            self.vehicle.add_attribute_listener('heading', self.heading_callback)
            self.vehicle.add_attribute_listener('attitude', self.attitude_callback)
            print 'started GNU thread'
            gnuThread.start()
           
        except [[KeyboardInterrupt]]:
            usrb_object.stop()
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
            self.DroneKitPort = port
            self.DroneKitHost = host
            self.isDroneKitStreaming = True
            self.sendDataToClient()
        if udpObj['Command'] == "Stop":
            print "Stop stream data"    
            self.isDroneKitStreaming = False
        if udpObj['Command'] == "Configure":
            print "Configure" 
            sample_rate = udpObj["Sample_Rate"]
            base_freq = udpObj["Base_Freq"]
            self._samp = sample_rate
            self._freq = base_freq
            nois_global_variable.getuhd_usrp_source_0().set_samp_rate(self._samp)
	    nois_global_variable.getWxgui_fftsink().set_sample_rate(self._samp)
	    nois_global_variable.getuhd_usrp_source_0().set_center_freq(self._freq, 0)
	    nois_global_variable.getWxgui_fftsink().set_baseband_freq(self._freq)
        if udpObj['Command'] == "AutoRotate":
            print "Rotate"
	    self.rotateIsExecuting = True
            self.startRotateDevice(20)
        if udpObj['Command'] == "Hold":
            print "Hold"
            self.holdRotateDevice()
	if udpObj['Command'] == "Unhold":
	    print "Unhold"
	    self.unholdRotateDevice()
        if udpObj['Command'] == "StopRotate":
            print "Stop rotate"
	    self.stopRotateDevice()
	if udpObj['Command'] == "Ping":
            print "PING"
class usrp_block(grc_wxgui.top_block_gui):
    def __init__(self,droneServer = None):
        self.droneServer = droneServer;
        grc_wxgui.top_block_gui.__init__(self, title="Usrp Block")
       
        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 1e6
        self.freq = freq = 105.3e6

        ##################################################
        # Blocks
        ##################################################
        self.wxgui_fftsink2_0 = fftsink2.fft_sink_c(
        	self.GetWin(),
        	baseband_freq=freq,
        	y_per_div=10,
        	y_divs=10,
        	ref_level=0,
        	ref_scale=2.0,
        	sample_rate=samp_rate,
        	fft_size=1024,
        	fft_rate=15,
        	average=False,
        	avg_alpha=None,
        	title="FFT Plot",
            	functionCallbackWhenHandleMessageSink = self.callBackWhenHandleMessage,
        	peak_hold=False,
        )
        self.Add(self.wxgui_fftsink2_0.win)
        self.uhd_usrp_source_0 = uhd.usrp_source(
        	",".join(("", "")),
        	uhd.stream_args(
        		cpu_format="fc32",
        		channels=range(1),
        	),
        )
        self.uhd_usrp_source_0.set_samp_rate(samp_rate)
        self.uhd_usrp_source_0.set_center_freq(freq, 0)
        self.uhd_usrp_source_0.set_gain(15, 0)
        self.uhd_usrp_source_0.set_antenna("RX2", 0)

       
        nois_global_variable.setuhd_usrp_source_0(self.uhd_usrp_source_0)  
        nois_global_variable.setWxgui_fftsink(self.wxgui_fftsink2_0) 

        ##################################################
        # Connections
        ##################################################
        self.connect((self.uhd_usrp_source_0, 0), (self.wxgui_fftsink2_0, 0))    

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.uhd_usrp_source_0.set_samp_rate(self.samp_rate)
        self.wxgui_fftsink2_0.set_sample_rate(self.samp_rate)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.uhd_usrp_source_0.set_center_freq(self.freq, 0)
        self.wxgui_fftsink2_0.set_baseband_freq(self.freq)

    def callBackWhenHandleMessage(self,message):
        self.droneServer.broadcastGnuMessage(message)

         
def initGnuThread(ucb=usrp_block, options=None):
    usrb_object = ucb(droneServer)
    usrb_object.Start(True)
    usrb_object.Wait()   



droneServer = DroneServer() 
gnuThread = threading.Thread(target=initGnuThread,args=(''))
reactor.listenUDP(8891, droneServer)
reactor.run()
            

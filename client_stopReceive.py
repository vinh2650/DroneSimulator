

import socket
from dronekit import connect, VehicleMode
import time
import json
import argparse

from twisted.internet.protocol import DatagramProtocol
from twisted.internet import reactor

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
class StopObj(DatagramProtocol):

    def startProtocol(self):
        sendObj =BaseJsonObj()
        sendObj.Command= "Stop"
        self.transport.write(sendObj.to_JSON(), ("127.0.0.1", 8891))
       
    def datagramReceived(self, datagram, address):
        print "Datagram %s received from %s" % (repr(datagram), repr(address))


reactor.listenMulticast(5591, StopObj(), listenMultiple=True)
reactor.run()

#!/usr/bin/python

from xaal.lib import Engine, Device, tools, config
from xaal.lib import tools
from xaal.lib.core import Engine
import ujson as json
import sys
import config
import re

logger = tools.get_logger("dumper",'DEBUG') 
def getalerte(msg):
    #msg.dump() 
    if msg.is_notify and (msg.action == "alert" or msg.action == "attributesChange"):
        body = str(msg.get_parameters())
        print body
        #body = "{u'falls': [{u'delay': 18, u'x': 4.7420089999999995, u'y': 1.958904, u'zone': u'0355'}]}"
        if "'x'" in body:
            split_res = body.split("'x': ")
            x = float(split_res[1][0:4])
            print "x : %f" % (x)
        if "'y'" in body:
            split_res = body.split("'y': ")
            y = float(split_res[1][0:4])
            print "y : %f" % (y)
   
class xAALProxy:

    def __init__(self):

        self.addr = tools.get_random_uuid()
        #print("My UUID is : %s" % self.addr)
        
        self.engine = Engine()      
        self.dev = Device("cli.experimental",self.addr)
        self.engine.add_device(self.dev)
        #self.eng.add_rx_handler(self.parse_answer)

    def sendmsg(self, target, action, key=None, value=None):
    
        addr = config.getConfigInfo(target, "xaaladdr") #hdrware addr of the equpmt
        print addr       

        self.engine.send_request(self.dev,[addr],action,None);


        #addr 224.0.29.200, port 1235
        #self.engine.get_network_connector().send(data)

    def recvmsg(self):
        data = self.engine.receive_msg()
        return data
        
    def filtmsg(self):
        self.engine.add_rx_handler(getalerte)  
        self.engine.run()

# simple test
if __name__ == '__main__':
    proxy = xAALProxy()
    proxy.sendmsg("lamp1", "off")
    proxy.sendmsg("lamp2", "off")
    '''<Message (0x7f2dc6860fd0)> b558fbb9-8a86-43a2-8e8e-5829d571973e falldetector.basic notify alert {u'falls': [{u'delay': 1, u'x': 4.614206, u'y': 5.012999, u'zone': u'0355'}]} 
        <Message (0x7f2dc67f2610)> b558fbb9-8a86-43a2-8e8e-5829d571973e falldetector.basic notify attributesChange {u'falls': [{u'delay': 4, u'x': 3.507991, u'y': 6.840183, u'zone': u'0355'}]}'''
        
    test = "<Message (0x7f2dc6860fd0)> b558fbb9-8a86-43a2-8e8e-5829d571973e falldetector.basic notify alert {u'falls': [{u'delay': 1, u'x': 4.614206, u'y': 5.012999, u'zone': u'0355'}]} "
    if ("notify" in test) and ("falls" in test):
        print "have test"
    body = str({u'falls': [{u'delay': 18, u'x': 4.7420089999999995, u'y': 1.958904, u'zone': u'0355'}]})
    #split_res = re.split(',|:',body)
    #print split_res[4], split_res[6]
    split_res = body.split("'x': ")
    print float(split_res[1][0:4])
#    if "u'x'" in body:

        
    while True:
        msg =  proxy.filtmsg()
    print "mission terminated"

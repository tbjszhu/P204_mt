#!/usr/bin/python

from xAAL.core import Engine
from xAAL.message import Message
from xAAL.message import MessageFactory
from xAAL import tools
import ujson as json
import sys
import config

class xAALProxy:

    def __init__(self):

        self.uuid = tools.get_random_uuid()
        print("My UUID is : %s" % self.uuid)
        
        self.app = Engine()


    def sendmsg(self, target, action, key=None, value=None):
    
        addr = config.getConfigInfo(target, "xaaladdr") #hdrware addr of the equpmt
        
        msg = Message()
        msg.set_targets([addr])
        msg.set_devtype('cli.experimental')
        msg.set_msgtype('request')
        msg.set_action(action)
        if(key != None and value != None) :
            body = {key:value}
            msg.set_body(body)
        msg.set_source(self.uuid)
        #msg.set_cipher_key("")
        #msg.set_signature()
        txt = {"header":msg.get_header(), "body":msg.get_body()}
        data = json.encode(txt)
        #addr 224.0.29.200, port 1235
        self.app.get_network_connector().send(data)

    def recvmsg(self):

        data = self.app.get_network_connector().get_data()
        return data

# simple test
if __name__ == '__main__':
    proxy = xAALProxy()
    while True :
        while (True) :
            msg = proxy.recvmsg()
            if msg != None :
                break
        print msg
        print "\n"
    print "mission terminated"

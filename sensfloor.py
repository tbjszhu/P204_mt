#!/usr/bin/python

from xaal.lib.core import Engine
import math

class Sensfloor():
    
    def __init__(self, engine):
        #threading.Thread.__init__(self)
        self.engine = engine
        self.engine.add_rx_handler(self.parse_msg)
        self.cpos = []
        self.spos = []
        self.threadName = "sensfloor"
        self.falldetected = False

    def parse_msg(self,msg):
        if msg:
            self.cpos = self.getFallPosition(msg)
            if self.cpos != None:
                self.spos = self.transformCartesianToPolar(3.5, 7, self.cpos[0], self.cpos[1])
                
    def getFallPosition(self,msg):
        if msg:
            if msg.is_notify and (msg.action == "alert" or msg.action == "attributesChange"):
                body = str(msg.get_parameters())
                #print body #body = "{u'falls': [{u'delay': 18, u'x': 4.7420089999999995, u'y': 1.958904, u'zone': u'0355'}]}"
                if "'x'" in body and "'y'" in body:
                    self.falldetected = True
                    split_res = body.split("'x': ")
                    x = float(split_res[1][0:4])
                    split_res = body.split("'y': ")
                    y = float(split_res[1][0:4])
                    print "[Sensfloor INFO] x : %f, y : %f" % (x,y)
                    return [x, y]   
                      
    def transformCartesianToPolar(self, x_org, y_org, x, y):                                
        distance = math.sqrt((x_org - x) * (x_org - x) + (y_org -y) * (y_org -y))
        if x != x_org:
            direction =  math.atan((y_org - y)/(x_org - x))
        else:
            direction = math.pi/2
            
        if direction > 0:
            direction = math.pi/2 - direction
        else:
            direction = (-1) * (math.pi/2 + direction)       
        #print "[Sensfloor INFO] distance : %f, direction : %f" % (distance, direction * 180 / math.pi) 
        return [distance, direction]

    def run(self):
        print "Sensfloor Start-->"
        try:
            while not self.falldetected:
                self.engine.loop()
                
        except KeyboardInterrupt:
            print "Sensfloor Quit"            
                
    def blockingRun(self):
        print "Sensfloor Start-->"
        while True:   
            #x_org = 3.5, y_org = 7
            self.cpos = self.getFallPosition()
            if self.cpos != None:
                self.spos = self.transformCartesianToPolar(3.5, 7, self.cpos[0], self.cpos[1])            
                print "[Sensfloor INFO] x : %f, y : %f distance %f, direction %f" % (self.cpos[0], self.cpos[1], self.spos[0], self.spos[1])
                
if __name__== '__main__':
    floor = Sensfloor()  

    try:
	    floor.blockingRun()
    except KeyboardInterrupt:
        print "Program Quit"

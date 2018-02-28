# This test demonstrates how to use the ALPhotoCapture module.
# Note that you might not have this module depending on your distribution
import os
import sys
import time
from naoqi import ALProxy

# Replace this with your robot's IP address
IP = "10.77.3.19"
PORT = 9559

# Create a proxy to ALPhotoCapture
try:
  photoCaptureProxy = ALProxy("ALPhotoCapture", IP, PORT)
except Exception, e:
  print "Error when creating ALPhotoCapture proxy:"
  print str(e)
  exit(1)
  
photoCaptureProxy.setResolution(2)
photoCaptureProxy.setPictureFormat("jpg")
path = "./recordings"
print photoCaptureProxy.takePicture(path, "image",False)
#time.sleep(1)
print photoCaptureProxy.takePictures(3, path, "image")  

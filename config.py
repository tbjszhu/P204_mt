from ConfigParser import ConfigParser
import os

CONF_DIR = "/home/user/P204/nao_xaal/"
CONF_NAME = "nao_xaal.conf"

def getConfigFile():
    return os.path.join(CONF_DIR, CONF_NAME)

def loadConfigFile(filepath):
    if os.path.exists(filepath):
        cfg = ConfigParser()
        cfg.read(filepath)
        return cfg
    return None

def getConfigInfo(section, info):
    configFile = getConfigFile()
    cfg = loadConfigFile(configFile)
    if cfg != None:
        # TODO exception ? 
        return cfg.get(section, info)
    print "Config file doesn't exist!"
    return None

if __name__ == "__main__":
    print "NAO mail compte : ", getConfigInfo("NAO mail", "compte")
    print "Contact list : ", getConfigInfo("Contact list", "user1")



#!/usr/bin/python

import smtplib
from email.MIMEMultipart import MIMEMultipart
from email.MIMEText import MIMEText

import config

class Mail:

    def __init__(self, text):
        self.fromaddr = config.getConfigInfo("NAOmail","compte")
        self.toaddr = config.getConfigInfo("Contact list","user1")
        self.msg = MIMEMultipart()
        self.msg['Subject'] = "Robot assistant"
        self.msg['From'] = self.fromaddr
        self.msg['To'] = self.toaddr
        self.msg.attach(MIMEText(text))

    def sendMail(self):
        srv = config.getConfigInfo("ServerSMTP", "server")
        self.s = smtplib.SMTP(srv) # 465,587)
        #self.s.ehlo()
        #self.s.starttls()
        #self.s.ehlo()
        pwd = config.getConfigInfo("NAOmail", "pwd")
        self.s.login(self.fromaddr, pwd)
        self.s.sendmail(self.fromaddr, self.toaddr, self.msg.as_string())
        self.s.quit()


# unit test
if __name__ == '__main__':
    link = "localhost:8080"
    topic = "nao_robot/camera/top/camera/image_raw"
    text = "The assistance robot and smart house has detected a fall. To check, please see the video from this address " + link
    text = text + "/stream_viewer?topic=" + topic
    try:
        mail = Mail(text)
        mail.sendMail()
    except:
        print "mail send failed"        

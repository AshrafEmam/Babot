import subprocess


class servo:
    right = 150
    left  = 150
    def __init__(self):
        subprocess.call('sudo /home/pi/PiBits/ServoBlaster/user/servod',shell=True)
        self.setRightServo(0)
        self.setLeftServo(0)
        
    def roll(self, vit, ste): # speed = 5 to -5
        self.right = int(150 - vit * 3 + ste * 3)
        self.left  = int(150 + vit * 3 + ste * 3)       

    def setRightServo(self,speed):
        ss = self.right + speed
        servoStr ="0=%u\n" % (ss)
        #print str(self.right)+'@'+servoStr
        with open("/dev/servoblaster", "wb") as f:
            f.write(servoStr)
               
    def setLeftServo(self,speed):
       ss = self.left + speed
       if ss > 250:
          ss = 250
       elif ss < 50:
           ss = 50
       servoStr ="1=%u\n" % (ss)
       with open("/dev/servoblaster", "wb") as f:
            f.write(servoStr) 

    def forward (self, speed):
       self.setLeftServo(speed)       
       speed = -speed
       self.setRightServo(speed)

    def backward(self,speed):
       self.setRightServo(speed)
       speed = -speed
       self.setLeftServo(speed)
   
    def stopall(self):
       self.setRightServo(0)
       self.setLeftServo(0)
       

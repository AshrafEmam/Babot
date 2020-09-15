import socket
from threading import Thread
import sys
import locale
import time
 

HOST = ''      # Symbolic name meaning all available interfaces
PORT = 5555    # Arbitrary non-privileged port
Packet = 1024  # Packet size

def shutdown_thread(t):
    print "shutting down"
    t.shutdown()
    while t.isAlive():
        time.sleep(.1)
    print "done shutting down"
    sys.exit()

    
class Server(Thread):
    con_down = False

    def __init__(self, numP = 3.0, numI = 0.0, numD=2.0, numG=3.0, numS = -1.80, SP = 0, ST = 0):
        super(Server , self).__init__()
        self.setAll(numP, numI, numD, numG, numS, SP, ST)
        self.start_server()

    def start_server(self):        
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        print 'Socket created'
        try:
            self.server.bind((HOST, PORT))
        except socket.error , msg:
            print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()
        print 'Socket bind complete'
        self.server.listen(4)
        print 'Socket now listening'

    def shutdown(self):
        self.con_down = True

    def getall(self):
        z =(self.numP, self.numI, self.numD, self.numG, self.numS, self.SP, self.ST)
        return z

    def setAll(self, numP, numI, numD, numG, numS, SP, ST):
        self.numP = numP
        self.numI = numI
        self.numD = numD
        self.numG = numG
        self.numS = numS
        self.SP = SP
        self.ST = ST
        
    def setKp(self, numP):
        self.numP = numP

    def setKi(self, numI):
        self.numI = numI

    def setKd(self, numD):
        self.numD = numD

    def setGain(self, numG):
        self.numG = numG

    def setPoint(self, numS):
        self.numS = numS

    def setSpeed(self, SP):
        self.SP = SP

    def setSteer(self, ST):
        self.ST = ST
        
    def run(self):
        #now keep talking with the client
        while not self.con_down:
            #wait to accept a connection - blocking call
            conn, addr = self.server.accept()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            initval = str(self.numP)+','+str(self.numI)+','+str(self.numD)+','+str(self.numG)+','+str(self.numS)+','+str(self.SP)+','+str(self.ST)
            conn.sendall(initval)           
            self.connected = True
            while self.connected:    
                data = conn.recv(Packet)
                reply = 'OK...' + data
                if not data:
                    print 'No Data received, Disconnecting' + data
                    connected = False
                    sys.exit()
                    break
                conn.sendall(reply)
                if data[0] == 'P':   # Kp
                    s = data[4:]
                    self.numP = locale.atof(s)
                elif data[0] == 'I': # Ki
                    s = data[4:]
                    self.numI = locale.atof(s)
                elif data[0] == 'D': # Kd
                    s = data[4:]
                    self.numD = locale.atof(s)
                elif data[0] == 'G': # Servo Gain
                    s = data[4:]
                    self.numG = locale.atof(s)
                elif data[0] == 'S': # set point
                    s = data[4:]
                    self.numS = locale.atof(s)
                elif data[0] == 'A': # speed
                    s = data[4:]
                    self.SP = int(locale.atof(s))
                elif data[0] == 'B': # steer
                    s = data[4:]
                    self.ST = int(locale.atof(s))                    
                elif data[0] == 'X': # close connection
                    conn.close()
                    self.connected = False
                elif data[0] == 'Q': # shutdown thread
                    conn.close()
                    self.connected = False
                    self.con_down = True
        self.server.close()
        shutdown_thread(self)
# Line to be added to main subroutine
#    s = Server()
#    s.start()
#    send_with_netcat("None genuine without this seal!")
#    shutdown_thread(s)       

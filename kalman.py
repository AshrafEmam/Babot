#!/usr/bin/python
from array import *
class Kalman:
    P= [[0,0],[0,0]]
#-----------------------------------------------------------------------------
    def __init__(self, QA = 0.001, QB = 0.003, R = 0.03):
         self.QA = QA
         self.QB = QB
         self.RM = R
         self.P[0][0] = 0.0
         self.P[0][1] = 0.0
         self.P[1][0] = 0.0
         self.P[1][1] = 0.0
         self.angle   = 0.0
         self.bias    = 0.0
#-----------------------------------------------------------------------------
    def set_R(self, Measure_R):
        self.RM = Measure_R
#-----------------------------------------------------------------------------
    def set_QB(self, Q_bias):
        self.QB = Q_bias
#-----------------------------------------------------------------------------
    def set_QA(self, Q_Angle):
        self.QA = Q_Angle
#-----------------------------------------------------------------------------
    def setAngle(self, newAngle):
        self.angle = newAngle
#-----------------------------------------------------------------------------
    def getAngle(self, newAngle, newRate, timediff):
        # step_1
        rate = newRate - self.bias
        self.angle = self.angle + timediff * rate
        # step 2
        self.P[0][0] += timediff *(timediff * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.QA)
        self.P[0][1] -= timediff * self.P[1][1]
        self.P[1][0] -= timediff * self.P[1][1]
        self.P[1][1] += self.QB  * timediff
        # step 3
        y = newAngle - self.angle
        # step 4
        s = self.P[0][0] + self.RM
        # step 5
        K0 = self.P[0][0]/s
        K1 = self.P[1][0]/s
        # step 6
        self.angle += K0 * y
        self.bias  += K1 * y
        # step 7
        P00 = self.P[0][0]
        P01 = self.P[0][1]
        self.P[0][0] -= K0 * P00
        self.P[0][1] -= K0 * P01
        self.P[1][0] -= K1 * P00
        self.P[1][1] -= K1 * P01
        return self.angle

        

from machine     import Timer, Pin
import time
from micropython import alloc_emergency_exception_buf
from math        import acos, pi
from machine import Pin, Timer
import time
import machine, onewire, ds18x20, os
from machine import UART

from time import sleep

from machine import Pin, PWM
from time import sleep

import time
from machine import ADC


# https://stackoverflow.com/questions/55744324/ac-dimmer-using-micropython
# https://forums.raspberrypi.com/viewtopic.php?t=314801
# https://github.com/RobotDynOfficial/RBDDimmer/blob/master/src/esp8266/RBDmcuESP8266.cpp

class Dimmer:
    def __init__(self, user_dimmer_pin, zc_dimmer_pin):
        print("Dimmer PIN : %d" % user_dimmer_pin)
        print("ZC PIN : %d" % zc_dimmer_pin)
                        
        self.dimmerGPIO = Pin(user_dimmer_pin, Pin.OUT)
        self._zc = Pin(zc_dimmer_pin,  Pin.IN)
        self.dimmerGPIO.value(0)
        self.dimmerCycles = 120  # <-- one second
        self.dimPower = 0
        self.nextEvaluation = 1
        self.powerSettingsChanged = True
        self.updatePower()
        
    def begin(self):
        self.setPower(0)
        self._zc.irq(trigger = Pin.IRQ_RISING, handler = self.onZC_ISR)        
        
    def setPower(self, power):         
        if power == self.dimPower:
            return
        
        if power < 0:
            power = 0
        if power > 100:
            power = 100
              
        print("Power: %d" % power)        
        self.dimPower = power
        self.powerSettingsChanged = True
        
        
    def updatePower(self):                
        self.onCycles = (self.dimPower / 100) * self.dimmerCycles
        self.offCycles = self.dimmerCycles - self.onCycles
        print("UPDATE_POWER - Cycles: %d/%d" % (self.onCycles, self.offCycles))
        
        self.remainingOn = self.onCycles
        self.remainingOff = self.offCycles
                
        self.powerSettingsChanged = False    
        
    
    def getPower(self):
        return self.dimPower
       
    def onZC_ISR(self, pin):
        
        if self.powerSettingsChanged:
            self.updatePower()
            
        if self.remainingOn > 0:  
            self.remainingOn = self.remainingOn - 1              
            self.dimmerGPIO.value(1)
            
        elif self.remainingOff > 0:
            self.dimmerGPIO.value(0)                
            self.remainingOff = self.remainingOff - 1                    
            
        #print("%d %d" %(self.remainingOff, self.remainingOn))
        
        if 0 == self.remainingOff and 0 == self.remainingOn:
            self.remainingOn = self.onCycles
            self.remainingOff = self.offCycles
            print("reset")
    

    
# try:
#     #from dimmer import Dimmer
#     dimmer = Dimmer(11, 10)
#     dimmer.begin()
#     dimmer.setPower(0)
#     power = 0
    
#     dimmer_percent = 0
#     while True:
#         dimmer.setPower(dimmer_percent) 
#         dimmer_percent = dimmer_percent + 5
#         print("Dimmer : %d" % dimmer_percent)
#         sleep(3)
     

# except KeyboardInterrupt:
#     h = Pin(11, Pin.OUT)
#     h.value(0)
#     print("Debugger Stopped..")

    

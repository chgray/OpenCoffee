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
        self.toggle_state = False
              
        self.dimOutPin = user_dimmer_pin
        self.dimZCPin = zc_dimmer_pin
        self.dimCounter = 0
        self.zeroCross = 0
     
        self.togMin = 0
        self.togMax = 1
        self.pulseWidth = 3
        self.toggleCounter = 0
        self.toggleReload = 25
        self._timer  = Timer()
                
        self.dimmerGPIO = Pin(user_dimmer_pin, Pin.OUT)
        self._zc     = Pin(zc_dimmer_pin,  Pin.IN)
        self.dimmerGPIO.value(0)
        
        
    def begin(self):
        self.setPower(0)
        self._zc.irq(trigger = Pin.IRQ_RISING, handler = self.onZC_ISR)        
        
    def setPower(self, power):        
        if (power >= 99):
            power = 99
        print("Power: %d" % power)
        
        self.dimPower = power        
    
    def getPower(self):
        return self.dimPower
       
    def onZC_ISR(self, pin):
        self.zeroCross = 1    
    
   
    
    print ("Hello")

# h = Pin(11, Pin.OUT)
# while True:
#     h.toggle()
#     sleep(0.0001)

try:
    #from dimmer import Dimmer
    dimmer = Dimmer(11, 10)
    dimmer.begin()
    power = 25
    while True:
        if power > 100:
            power = 85
            
        #dimmer.setPower(power)
        #power += 2
        #if dimmer.value >= 1:
        #    dimmer.value = 0
        sleep(1)


except KeyboardInterrupt:
    h = Pin(11, Pin.OUT)
    h.value(0)
    print("Debugger Stopped..")

    
except Exception as e:
    h = Pin(11, Pin.OUT)
    h.value(0)
    print("Debugger Stopped : general exception...")
    print(e)



print ("Goodbye!")
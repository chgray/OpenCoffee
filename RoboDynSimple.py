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
        self.dimmerCycles = 12  # <-- one second
        
    def begin(self):
        self.setPower(0)
        self._zc.irq(trigger = Pin.IRQ_RISING, handler = self.onZC_ISR)        
        
    def setPower(self, power):        
        if (power >= 99):
            power = 99
        print("Power: %d" % power)
        
        self.dimPower = power
        
        self.onCycles = (power / 100) * self.dimmerCycles
        self.offCycles = self.dimmerCycles - self.onCycles
        print("Cycles: %d/%d" % (self.onCycles, self.offCycles))
        
        self.on = True
        self.nextEvaluation = self.onCycles        
          
    
    def getPower(self):
        return self.dimPower
       
    def onZC_ISR(self, pin):
        self.nextEvaluation -= 1
        if 0 == self.nextEvaluation:
            if self.on:
                print("On!")
                self.on = False
                self.nextEvaluation = self.offCycles
                self.dimmerGPIO.value(0)
            else:
                print("Off!")
                self.on = True
                self.nextEvaluation = self.onCycles
                self.dimmerGPIO.value(1)
    

# h = Pin(11, Pin.OUT)
# while True:
#     h.toggle()
#     print("Flip")
#     sleep(1)
    
try:
    #from dimmer import Dimmer
    dimmer = Dimmer(11, 10)
    dimmer.begin()
    dimmer.setPower(25)
    power = 0
    while True:
        if power > 100:
            power = 85
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
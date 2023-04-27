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


class DimmerModes():
    NORMAL_MODE = 1
    TOGGLE_MODE = 2
    
class DimmerState():
    ON = 1
    OFF = 2

# https://stackoverflow.com/questions/55744324/ac-dimmer-using-micropython
# https://forums.raspberrypi.com/viewtopic.php?t=314801
# https://github.com/RobotDynOfficial/RBDDimmer/blob/master/src/esp8266/RBDmcuESP8266.cpp

class Dimmer:
    def __init__(self, user_dimmer_pin, zc_dimmer_pin):
        print("Dimmer PIN : %d" % user_dimmer_pin)
        print("ZC PIN : %d" % zc_dimmer_pin)
        self.toggle_state = False
        
        self.dimPulseBegin= 1
        self.dimOutPin = user_dimmer_pin
        self.dimZCPin = zc_dimmer_pin
        self.dimCounter = 0
        self.zeroCross = 0
        self.dimMode = DimmerModes.NORMAL_MODE
        self.togMin = 0
        self.togMax = 1
        self.pulseWidth = 3
        self.toggleCounter = 0
        self.toggleReload = 25
        self._timer  = Timer()
        #self.dimmerGPIO = Pin(user_dimmer_pin, Pin.OUT)
        self._zc     = Pin(zc_dimmer_pin,  Pin.IN)
        
    def begin(self, dimmer_mode, on_off):
        self.dimMode = dimmer_mode
        self.dimState = on_off       
        self._zc.irq(trigger = Pin.IRQ_RISING, handler = self.onZC_ISR)
        #self._timer.init(freq = 5000, mode = Timer.PERIODIC, callback = self.onTimerISR)
        
    def setPower(self, power):        
        if (power >= 99):
            power = 99
        print("Power: %d" % power)
        
        self.dimPower = power
        self.dimPulseBegin = self.powerBuf(power)       
        print("PulseBegin: %d" % self.dimPulseBegin)
    
    def getPower(self):
        if self.dimState == DimmerState.ON:
            return self.dimPower
        else:
            return 0
        
    def setState(self, state):    
        self.dimState = state    
    
    def getState(self):        
        if self.dimState == DimmerState.ON: 
            ret = True
        else:
            ret = False
        return ret
    
    def changeState(self):
        if self.dimState == DimmerState.ON:
            self.dimState = DimmerState.OFF
        else:
            self.dimState = DimmerState.ON
            
    def powerBuf(self, value):
        return (100 - value)# + 1
            
    def toggleSettings(self, minValue, maxValue):
        if maxValue > 99:
            maxValue = 99
        if minValue < 1:        
            minValue = 1
            
        self.dimMode = DimmerModes.TOGGLE_MODE
        self.togMin = self.powerBuf(maxValue)
        self.togMax = self.powerBuf(minValue)
        self.toggleReload = 50

    def getMode(self):
        return self.dimMode
    
    def setMode(self, DIMMER_MODE):
        self.dimMode = DIMMER_MODE
    
    def onZC_ISR(self, pin):
        if self.dimState == DimmerState.ON:
            self.zeroCross = 1    
    
    def onTimerISR(self, _timer):  
        if self.zeroCross != 1:
            return        
        
        self.dimCounter += 1
        
        if self.dimMode == DimmerModes.TOGGLE_MODE:
            #*****
            #* TOGGLE DIMMING MODE
            #*****/
            if self.dimPulseBegin >= self.togMax:	            
                # if reach max dimming value 
                self.togDir = False	# downcount				
            
            if self.dimPulseBegin <= self.togMin:
                #if reach min dimming value 
                self.togDir = True	# upcount
            
            if self.toggleCounter == self.toggleReload:            
                if self.togDir == True:
                    self.dimPulseBegin+=1
                else:
                    self.dimPulseBegin-=1
                   
  
        #*****
        #* DEFAULT DIMMING MODE (NOT TOGGLE)
        #*****/
        if self.dimCounter >= self.dimPulseBegin:
            self.dimmerGPIO.value(1)

        if self.dimCounter >= self.dimPulseBegin + self.pulseWidth:    
            self.dimmerGPIO.value(0)
            self.zeroCross = 0
            self.dimCounter = 0        
       
        if self.toggleCounter >= self.toggleReload:
            self.toggleCounter = 1	    
    
    print ("Hello")

# h = Pin(11, Pin.OUT)
# while True:
#     h.toggle()
#     sleep(0.0001)

try:
    #from dimmer import Dimmer
    dimmer = Dimmer(11, 10)
    dimmer.begin(DimmerModes.NORMAL_MODE, DimmerState.ON)
    power = 100
    while True:
        if power > 100:
            power = 85
            
        dimmer.setPower(power)
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
    print("Debugger Stopped..")



print ("Goodbye!")
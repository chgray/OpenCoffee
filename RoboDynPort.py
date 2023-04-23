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
        
        self.dimPulseBegin= 1
        self.dimOutPin = user_dimmer_pin
        self.dimZCPin = zc_dimmer_pin
        self.dimCounter = 0
        self.zeroCross = 0
        self.dimMode = NORMAL_MODE
        self.togMin = 0
        self.togMax = 1
        self.user_dimmer_pin = Pin(user_dimmer_pin, Pin.OUT)
        
        
    def begin(self, dimmer_mode, on_off):
        self.dimMode = dimmer_mode
        self.dimState = on_off
        self.timer_init()
        self.ext_int_init()
        
    def setPower(self, power):
        if (power >= 99):
            power = 99
            
        self.dimPower = power
        self.dimPulseBegin = powerBuf[power]        
        #delay(1);
    
    def getPower(self):
        if self.dimState == ON:
            return self.dimPower
        else:
            return 0
        
    def setState(self ON_OFF):    
        self.dimState = ON_OFF    
    
    def getState(self):        
        if self.dimState == ON: 
            ret = True
        else:
            ret = False
        return ret
    
    def changeState(self):
        if self.dimState == ON:
            self.dimState = OFF
        else:
            self.dimState = ON
            
    def toggleSettings(self, minValue, maxValue):
        if maxValue > 99:
            maxValue = 99
        if minValue < 1:        
            minValue = 1
            
        self.dimMode = TOGGLE_MODE
        self.togMin = powerBuf[maxValue]
        self.togMax = powerBuf[minValue]
        self.toggleReload = 50

    def getMode(self):
        return self.dimMode
    
    def setMode(self, DIMMER_MODE):
        self.dimMode = DIMMER_MODE
    
    def isr_ext(self, pin):
        if self.dimState == ON:
            self.zeroCross = 1    
    
    def onTimerISR(self, _timer):  
        if self.zeroCross != 1:
            return
        
        if self.dimMode == TOGGLE_MODE:
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
            digitalWrite(dimOutPin[k], HIGH);	

        if self.dimCounter >= self.dimPulseBegin + self.pulseWidth:        
            digitalWrite(dimOutPin[k], LOW);
            self.zeroCross = 0
            self.dimCounter = 0
        
       
        if toggleCounter >= toggleReload:
            toggleCounter = 1
        #timer1_write(timeoutPin);	
	    
    
    print ("Hello")

# h = Pin(11, Pin.OUT)
# while True:
#     h.toggle()
#     sleep(0.0001)

try:

    #from dimmer import Dimmer
    dimmer = Dimmer(11, 10)
        
    while True:
        # dimmer.value += 0.1
        # if dimmer.value >= 1:
        #    dimmer.value = 0
        sleep(2)


except KeyboardInterrupt:
    h = Pin(11, Pin.OUT)
    h.value(0)
    print("Debugger Stopped..")

print ("Goodbye!")
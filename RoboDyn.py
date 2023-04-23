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
class Dimmer:
    def __init__(self, pwm_pin, zc_pin, fpulse = 4000):
        print("INPUT PIN : %d" % zc_pin)
        print("OUTPUT PIN : %d" % pwm_pin)
        alloc_emergency_exception_buf(100)
        self._cnt    = 0
        self._freq   = 0
        self._timer  = Timer()
        self._mode   = Timer.ONE_SHOT
        self._pwm    = Pin(pwm_pin, Pin.OUT)
        self._fpulse = fpulse
        self._ppulse = 100.0 / fpulse + 0.11
        self._zc     = Pin(zc_pin,  Pin.IN)
        self._val    = 1
        self._zc.irq(trigger = Pin.IRQ_RISING, handler = self._zeroDetectIsr)
        self.nextPrint = 0
        self.hits = 0
        self.off = True
        self.timerStart = 0
    
    def _zeroDetectIsr(self, pin):
        self.hits = self.hits + 1
        #if 0 != self.timerStart:
            #print("Done")
            #machine.reset()
            
        #print("hits")
        if time.ticks_ms() > self.nextPrint:
            print("Hits: %d, %d, %d, %d" % (self.hits, time.ticks_ms(), self._cnt, self._freq))
            self.hits = 0            
            self.nextPrint = time.ticks_ms() + 1000        
        
        if 0 == self._freq:
            #print("ON_X")
            self._pwm.on()
            return
        if 0 > self._freq:
            #print("OFF_X")
            self._pwm.off()
            return
        
        self._cnt += 1
        
        if 1 == self._cnt:
            #print("Turning on timer! %f" % self._freq)    
            self.timerStart = time.ticks_ms()        
            self._timer.init(freq = self._freq, mode = self._mode, callback = self._dimmDelayIsr)
    
    
    def _dimmDelayIsr(self, _timer):        
        #print("Delay %d" % (time.ticks_ms() - self.timerStart))
        self.timerStart = 0
        
        if self._cnt == 1:
            self._pwm.on()
            #self.off = False
            #print("ON_Y : %d" % self._cnt)
            self._timer.init(freq = self._fpulse, mode = self._mode, callback = self._dimmDelayIsr)
        else:
            #print("OFF_Y")
            self._pwm.off()
            #self.off = True        
        self._cnt = 0
    
   
    @property
    def value(self):
        return self._val
    
    
    @value.setter
    def value(self, p):
        p = min(1, max(0, p))
        
        if not self._val == p:
            print("Setting Value: _val=%f,  p=%f, pi=%f" % (self._val, p, pi))    
            self._val = p
            p         = acos(1 - p * 2) / pi
            
            print("PPULSE: %f, %f " % (self._ppulse, p))
            if p < self._ppulse:
                f = -1
            elif p > 0.99:
                f = 0
            else:
                f = 100 / (1 - p)
                        
            self._freq = int(f)
            print("Setting FREQ to %f " % self._freq)
        else:
            print("NOT SETTING, values are the same: _val=%f,  p=%f" % (self._val, p))
        
        return self._val
    
    
    print ("Hello")

# h = Pin(11, Pin.OUT)
# while True:
#     h.toggle()
#     sleep(0.0001)

try:

    #from dimmer import Dimmer
    dimmer = Dimmer(11, 10)
    dimmer.value = 12000
        
    while True:
        # dimmer.value += 0.1
        # if dimmer.value >= 1:
        #    dimmer.value = 0
        dimmer.value += 10
        print("Value : %f" % dimmer.value)
        sleep(2)


except KeyboardInterrupt:
    h = Pin(11, Pin.OUT)
    h.value(0)
    print("Debugger Stopped..")

print ("Goodbye!")
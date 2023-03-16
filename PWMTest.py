from machine import Pin, Timer
import time
from time import sleep

from machine import Pin, PWM
from time import sleep

global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/


#while True:
#    led = Pin(25, Pin.OUT)
#    led.toggle()
#    print("0")
#    sleep(1)


minHoldTime = 250
t0 = time.ticks_ms()
percent = 1

def blink(timer):
    global percent
    p = (percent / (100/5))
    m = (percent % (100/5))
    d = (int)(p) * 200
    duration = (float)(percent) * (float)(2000) / (float)(100)
    percent = percent + 1
    print("x %d, %d, %f, %f" % (percent, (int)(p),d, m))
    if percent > 100:
        percent = 0

timer = Timer()
timer.init(period=100, mode=Timer.PERIODIC, callback=blink)
sleep(5000)


#heater_pid = PWM(Pin(25))
#heater_pid.freq(10)
#heater_pid.duty_u16((int)(65535/4) * 4)
#sleep(5000)

dimmer_percent = 0
def SetMotorPercent(percent):
    
    7545,  1.46
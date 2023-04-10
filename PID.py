from machine import Pin, Timer
import time
import machine, onewire, ds18x20
from lcd1602 import *
from PID_lib import *

from time import sleep

global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/

print("Running.")

led = Pin(25, Pin.OUT)
button = Pin(12, Pin.IN, Pin.PULL_DOWN)
timer = Timer()

print("--------------------------------")
count = 0
temp=0.1
heating=1

# PID
# https://github.com/gastmaier/micropython-simple-pid

t0= time.ticks_ms()

def simulate_temp_change(timer):
    global temp
    global t0
    
    if 0 != heating:
        temp = (temp + 0.5)
        led.value(1)
    else:
        temp = (temp - 0.25)
        led.value(0)
        
    if temp < 0:
        temp = 0
    elif temp > 300:
        temp = 300
    
    t= time.ticks_ms() - t0 # t is CPU seconds elapsed (floating point)
    print("%f, %f" % (t, temp))


timer.init(freq=10, mode=Timer.PERIODIC, callback=simulate_temp_change)



# http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
pid = PID(1, 0.1, 0.05, setpoint=211, scale='ms')

pid.output_limits = (0, 1)    # Output value will be between 0 and 10
pid.set_auto_mode(True, last_output=0)
# Assume we have a system we want to control in controlled_system
# v = controlled_system.update(0)

while True:
    
    # Compute new output from the PID according to the systems current value
    control = pid(temp)
    
    #print("Temp: %f  Control: %d" % (temp, control))
    heating = control
    
    sleep(.1)
    
    b = button.value()
    if 0 == button.value():
        print("Button")
        temp = temp - 2
    
    # Feed the PID output to the system and get its current value
    # v = controlled_system.update(control)
















while 1:
    b = button.value()
    
    try:
        count = count + 1
        print("Temp : %f" % (temp))
        
        if temp > 3:
            heating = 0
            
        sleep(2)
        if 0 == button.value():
            print("button")
            count += 100
            sleep(1)
            
    except Exception as e:
        print(e)
        print("Not good..")
        
        
    
 










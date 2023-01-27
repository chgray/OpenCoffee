from machine import Pin, Timer
import time
import machine, onewire, ds18x20
from lcd1602 import *
from Max6675 import *
from PID_lib import *

from time import sleep


global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/

print("Running.")

led = Pin(25, Pin.OUT)
heater = Pin(26, Pin.OUT)
button = Pin(12, Pin.IN, Pin.PULL_DOWN)
timer = Timer()




t0= time.ticks_ms()
count = 0
temp=150
heating=1

def simulate_temp_change(timer):
    global temp
    global t0
    
    if 0 != heating:
        temp = (temp + 2)
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


#timer.init(freq=10, mode=Timer.PERIODIC, callback=simulate_temp_change)

so = Pin(17, Pin.IN)
sck = Pin(15, Pin.OUT)
cs = Pin(16, Pin.OUT)
max = MAX6675(sck, cs , so)


# https://docs.sunfounder.com/projects/thales-kit/en/latest/micropython/liquid_crystal_display.html#what-more

# IOExpander
#https://www.ti.com/lit/ds/symlink/pcf8574.pdf?ts=1627006546204&ref_url=https%253A%252F%252Fwww.google.com%252F
# LCD
# https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf


lcd = LCD()
lcd.openlight()
lcd.clear()
state=1

heater.value(0)

lcd.message('Setting up Temp')
lcd.message('Found DS devices')
lcd.message('Temperature (C)')
lcd.clear()

# http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
goalTemp = 80
pid = PID(0.5, 60, 2, setpoint=goalTemp, scale='ms')
pid.output_limits = (0, 1)    # Output value will be between 0 and 10
pid.set_auto_mode(True, last_output=0)


heater.value(0)
count = 0

while True:
    b = button.value()
    #heater.toggle()
    try:
        #max.refresh()
        temp = max.read()
        #temp = (temp * (9/5)) + 32
        
        # Compute new output from the PID according to the systems current value
        control = pid(temp)
        
        heating = control
        heater.value(control)
    
    
        t= time.ticks_ms() - t0 # t is CPU seconds elapsed (floating point)
       
        print("Val, %d, %f, %d, %d, %d" % (t, temp, count, control, goalTemp))
        lcd.clear()
        lcd.write(0, 0, ('%f, %d' % (temp, control)))
        lcd.write(0, 1, ("T:%d" % t))
        
        #p, i, d = pid.components 
        #lcd.write(0, 1, "%d, %d, %d" % (p, i, d))
        
        #lcd.write(0,1, str(count))
        count = count + 1
        
        if 0 == button.value():
            #heater.toggle()
            goalTemp += 5
            print("GoalTemp: %d" % goalTemp)
            pid = PID(0.5, 60, 2, setpoint=goalTemp, scale='ms')
            pid.output_limits = (0, 1)    # Output value will be between 0 and 10
            pid.set_auto_mode(True, last_output=0)

            
            #count += 100
            sleep(1)
            
    except Exception as e:
        print(e)
        print("Not good..")
        
    sleep(1)
        
        
    
lcd.clear()
lcd.message("BYE")
heater.value(0)
print("BYE")
 

#timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)








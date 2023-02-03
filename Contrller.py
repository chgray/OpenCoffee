from machine import Pin, Timer
import time
import machine, onewire, ds18x20, os
from lcd1602 import *
from Max6675 import *
from PID_lib import *

from time import sleep

from machine import Pin, PWM
from time import sleep

global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/


#https://github.com/miketeachman/micropython-rotary

print("Running.")

#led = Pin(25, Pin.OUT)
heater = Pin(26, Pin.OUT)
#button = Pin(12, Pin.IN, Pin.PULL_DOWN)
button = Pin(0, Pin.IN, Pin.PULL_DOWN)
timer = Timer()

count = 0
temp=150
heating=1

def fileExists(path):
    try:
        os.stat(path)
        return True
    except OSError:
            return False
        

def simulate_temp_change(timer):
    global temp
    global t0
    global secRemaining
    
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




lcd.message('Setting up Temp')
lcd.message('Found DS devices')
lcd.message('Temperature (C)')
lcd.clear()


from Rotary import Rotary
import time
from RotaryIRQ import RotaryIRQ

rotary = RotaryIRQ(pin_num_clk=2, 
              pin_num_dt=1, 
              min_val=0, 
              max_val=110, 
              reverse=False, 
              range_mode=RotaryIRQ.RANGE_BOUNDED)
              


#p = 0.16
fileIndex = 0
fileName = ("longRun.%d.csv" % fileIndex)
while fileExists(fileName):
    #os.remove(fileName)
    fileIndex = fileIndex + 1
    fileName = ("longRun.%d.csv" % fileIndex)
    print("Seeking unique log file %s" % fileName)

print("Found file: %s" % fileName)

f = open(fileName, 'w')
t_init = time.ticks_ms()

f.write("Time, Time, Temp, Control, Control, P, I, D\r\n")
  

# 0.2, 0, 2 is pretty good (slow ramp, but holds well)

p = 0.06 #pX / 100
i = 0
# http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
goalTemp = 105
#i = 0.1 # most likely half this
d = 3

pid = PID(p,i, d, setpoint=goalTemp, scale='ms')
pid.output_limits = (0, 1)    # Output value will be between 0 and 10
pid.set_auto_mode(True, last_output=0)
    
heater_time = t0
heaterOn = False
heater.value(0)
rotary.set(goalTemp)
count = 0
t0 = time.ticks_ms()
switch_time = t0


def toggle_heater(timer):
    global switch_time
    global heater
    delta = switch_time - time.ticks_ms()
    
    if delta > 0:
        print("Delta %d, %d" % (delta, heaterOn))
        #if heaterOn:
        #    heater.value(1)
        #else:
        #    heater.value(0)
            
            

timer.init(freq=10, mode=Timer.PERIODIC, callback=toggle_heater)

        
try:
    nextPrintTime = 0
    tOld = time.ticks_ms()
    while True :        
        f.flush()        
        try:
            b = button.value()            
            
            display=False
            lcdDisplay = False
            
            if goalTemp != rotary.value():
                goalTemp = rotary.value()
                print("New Set Point : %d" % goalTemp)                
                lcdDisplay = True
                
            pid.setpoint = goalTemp
            
            #sleep(5)
            #print("Delta %d" % (time.ticks_ms() - tOld))
            tOld = time.ticks_ms() 
              
            #sleep(0.05)
            if time.ticks_ms() > switch_time:                
                temp = max.read()
                t = time.ticks_ms() - t0 # t is CPU seconds elapsed (floating point)
                t_start = time.ticks_ms() - t_init            
            
                print("Delta : %d (%d)" % (time.ticks_ms() - switch_time, heaterOn))
            
                # Compute new output from the PID according to the systems current value
                control = pid(temp)            
                duration = control * 2000      
                                
                if duration < 250 and duration != 0:
                    duration = 250
                sleepTime = 2000 - duration                                                   
                
                if heaterOn:
                    switch_time = time.ticks_ms() + (sleepTime)
                    if sleepTime != 0:
                        heater.value(0)                                                 
                    heaterOn = False
                else:                    
                    switch_time = time.ticks_ms() + (duration)
                    if duration != 0:
                        heater.value(1)
                       
                    heaterOn = True
                    display = True
                   
                              
            if (time.ticks_ms() > nextPrintTime) or display:
                lcdDisplay = True
                nextPrintTime = time.ticks_ms() + 2000
                #print("Duration %d, sleepUntil %d, sleepTime %d, heaterOn %d" % (duration, switch_time, sleepTime, heaterOn))    
                print("Val, %d, %d, %f, %f, %d, %f, %f, %f" % (t_start, t, temp, control, control, p, i, d))          
                #f.write("%d, %d, %f, %f, %d, %f, %f, %f\r\n" % (t_start, t, temp, control, control, p, i, d))
                
            if lcdDisplay:
                lcd.clear()
                lcd.write(0, 0, ('Goal: %d(C), %f' % (goalTemp,control)))
                lcd.write(0, 1, ("%f" % temp))
                #print("Next Switch %d" % (switch_time - time.ticks_ms()))
                #switch_time = time.ticks_ms() + 100000
                
            #heater.value(1)
            #sleep(duration / 1000)
            #if sleepTime != 0:
            #    heater.value(0)
            #    sleep(sleepTime / 1000)
                
            
            count = count + 1
            
            if 0 == button.value():
                goalTemp += 5
                print("GoalTemp: %d" % goalTemp)
                pid = PID(p, i, d, setpoint=goalTemp, scale='ms')
                pid.output_limits = (0, 1)    # Output value will be between 0 and 10
                pid.set_auto_mode(True, last_output=0)
                sleep(.25)
                
        except OSError:
            heater.value(0)
            #f.write("Crashed")            
            print("OSError")
            

except KeyboardInterrupt:
    heater.value(0)
    f.write("Crashed")
    print(e)
    print("Debugger Stopped..")
    
#except Exception as e:
#    heater.value(0)
#    print("CRASHED")
#    #sys.print_exception(e)
#    f.write("Crashed")
#    print("Not good..")
        
sleep(.25)
        
        
    
lcd.clear()
lcd.message("BYE")
heater.value(0)
print("BYE")
 

#timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)








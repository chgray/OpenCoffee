from machine import Pin, Timer
import time
import machine, onewire, ds18x20, os
from lcd1602 import *
from Max6675 import *
from PID_lib import *
from machine import UART

from time import sleep

from machine import Pin, PWM
from time import sleep

from Rotary import Rotary
import time
from RotaryIRQ import RotaryIRQ


global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/
#https://github.com/miketeachman/micropython-rotary


uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))
uart.init(bits=8, parity=None, stop=2)

uart.write("Welcome to OpenCoffee\r\n")
while True:
    uart.write("BOOTED_X")
    print("Booted")
    sleep(1)
    #machine.restart()


print("123_Running.")

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
    os.remove(fileName)
    fileIndex = fileIndex + 1
    fileName = ("longRun.%d.csv" % fileIndex)
    print("Seeking unique log file %s" % fileName)

print("Found file: %s" % fileName)

f = open(fileName, 'w')
t_init = time.ticks_ms()

f.write("Time, Time, Temp, Control, Control, P, I, D\r\n")
  

# 0.2, 0, 2 is pretty good (slow ramp, but holds well)

p = 0.3 #pX / 100
i = 0
# http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
goalTemp = 105.3
#i = 0.1 # most likely half this
d = 2

with open('PID.config') as configFile:
    value = configFile.readlines()
    print("Value: [%s] - %s" % (value, value[0]))
    bits = value[0].split(",")
    print(bits[0])
    print(bits[1])
    print(bits[2])
    p = (float)(bits[0]) / 10
    i = (float)(bits[1]) / 10
    d = (float)(bits[2]) / 10


pid = PID(p,i, d, setpoint=goalTemp, scale='ms')
pid.output_limits = (0, 1)    # Output value will be between 0 and 10
pid.set_auto_mode(True, last_output=0)
    

heater.value(0)
rotary.set(goalTemp)
count = 0
t0 = time.ticks_ms()

control = 0.0
heater_time = t0
switch_time = t0
heater_cycle_time = t0
heater_off_time = t0


def toggle_heater(timer):
    global heater
    global heater_off_time
    global heater_cycle_time
         
    # Decide if we need to turn off the heater due to our % time being up
    if heater_off_time != -1 and time.ticks_ms() > heater_off_time: 
        delta = time.ticks_ms() 
        #print(" ON_Delta %d, %f" % (heater_off_time - delta, control))
        heater_off_time = -1
    
    # Decide if we're beyond the cycle time period 
    if time.ticks_ms() > heater_cycle_time: 
        delta = time.ticks_ms() 
        #print("OFF_Delta %d, %f" % (heater_cycle_time - delta, control))
        heater_cycle_time = -1
    
    # Recompute stats if we're there
    if heater_cycle_time == -1:
        duration = control * 5000      
                                
        #if duration < 50 and duration != 0:
        #    duration = 50
        sleepTime = 5000 - duration   
        
        heater_off_time = time.ticks_ms() + duration
        heater_cycle_time = time.ticks_ms() + (duration + sleepTime)
        
        # Turn On Heater
        if duration != 0:
            print("ON : %d, %d" % (duration, sleepTime))
            heater.value(1)
            
    if -1 == heater_off_time:
        #if heater.value() == 1:
        #    print("OFF")
        heater.value(0)
            
timer.init(freq=100, mode=Timer.PERIODIC, callback=toggle_heater)

        
try:
    nextPrintTime = 0
    t_start = time.ticks_ms()
    mode = 0
    while True :        
        f.flush()  
        t = time.ticks_ms()      
        try:
            b = button.value()  
            display=False
            lcdDisplay = False 
            updatePID = False   
            updateConfig = False
            
            if mode == 0:
                if goalTemp != rotary.value():
                    goalTemp = rotary.value()
                    print("New Set Point : %d" % goalTemp)                
                    lcdDisplay = True 
                    updatePID = True
                    updateConfig = True
                    pid.set_auto_mode(True)
            elif mode == 1:
                if p != rotary.value()/10:
                    p = rotary.value()/10
                    print("New P : %f" % p)                
                    lcdDisplay = True
                    updatePID = True
                    updateConfig = True
            elif mode == 2:
                if i != rotary.value()/10:
                    i = rotary.value()/10
                    print("New I : %d" % i)                
                    lcdDisplay = True
                    updatePID = True
                    updateConfig = True
            elif mode == 3:
                if d != rotary.value()/10:
                    d = rotary.value()/10
                    print("New D : %d" % d)                
                    lcdDisplay = True
                    updatePID = True
                    updateConfig = True
            elif mode == 4:
                if control != rotary.value()/100:
                    control = rotary.value()/100
                    print("Manual Control : %f, %d" % (control, rotary.value()))               
                    lcdDisplay = True
                    updatePID = True
                    updateConfig = True   
                    pid.set_auto_mode(False)                 
                                        
            if time.ticks_ms() > switch_time:                
                temp = max.read()            
                pid.setpoint = goalTemp    
                # Compute new output from the PID according to the systems current value
                
                if mode != 4:
                    control = pid(temp) 
                else:
                    control = rotary.value()
                    
                switch_time = time.ticks_ms() + 2000
                #print("Recalculated.")                   
                              
            if (time.ticks_ms() > nextPrintTime) or display:
                lcdDisplay = True
                nextPrintTime = time.ticks_ms() + 2000
                print("Val, %d, %d, %d, %f, %f, %f, %f, %f    " % (t - t_start, t, goalTemp, temp, control, p, i, d))          
                #f.write("%d, %d, %d, %f, %f, %f, %f, %f\r\n" % (t - t_start, t, goalTemp, temp, control, p, i, d))
                
                msg = ("%d, %d, %d, %f, %f, %f, %f, %f\r\n" % (t - t_start, t, goalTemp, temp, control, p, i, d))
                uart.write(msg.encode('utf-8'))
                
                
            if updatePID:
                pid = PID(p, i, d, setpoint=goalTemp, scale='ms')
                pid.output_limits = (0, 1)    # Output value will be between 0 and 10
                pid.set_auto_mode(True, last_output=0)
                print("PID UPDATED")
                
            if 0 == button.value():
                mode = mode + 1
                if mode > 4:
                    mode = 0              
                lcdDisplay = True
                            
                if 0 == mode:                 
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=110, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(goalTemp)
                if 1 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=40, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(p*10)
                if 2 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=40, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(i * 10)
                if 3 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=40, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(d * 10)
                if 4 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set((int)(control*100))
                
                sleep(.25)                
           
           
            if lcdDisplay:
                lcd.clear()
                if 0 == mode:
                    lcd.write(0, 0, ('Goal: %d(C), %f' % (goalTemp,control)))
                    lcd.write(0, 1, ("%f" % temp))    
                if 1 == mode:
                    lcd.write(0, 0, ("P = %f" % (p)))
                if 2 == mode:
                    lcd.write(0, 0, ("I = %f" % (i)))
                if 3 == mode:
                    lcd.write(0, 0, ("D = %f" % (d)))
                if 4 == mode:
                    lcd.write(0, 0, "*MANUAL HEATER*")
                    lcd.write(0, 1, ("%f %f(C)" % (control, temp)))
                    
                    
            if updateConfig:
                with open("PID.config", 'w') as save:
                    save.write("%d,%d,%d" % (p*10, i*10, d*10))
            count = count + 1
            

                
        except OSError:
            heater.value(0)
            timer.deinit()
            #f.write("Crashed")            
            print("OSError")
            

except KeyboardInterrupt:
    heater.value(0)
    timer.deinit()
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







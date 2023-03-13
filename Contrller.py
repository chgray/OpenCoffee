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
from machine import ADC


global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/
#https://github.com/miketeachman/micropython-rotary

pressureADC = ADC(28)
#while True:
#        v = pressure.read_u16()
#        print(v)
#        sleep(0.25)
        

g_MaxTemp = 130
lcd = LCD()
lcd.openlight()
lcd.clear()
#lcd = 0
def lcd_write(x, y, msg):
    if 0 != lcd:
        lcd.write(x,y,msg)
        
def lcd_clear():
    if 0 != lcd:
        lcd.clear()
    
    

uart = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9), timeout=1000, timeout_char=1000)
uart.init(bits=8, parity=None, stop=2)

uart.write("Welcome to OpenCoffee\r\n")
#while True:
#    uart.write("BOOTED_X")
#    print("Booted")
#    sleep(1)
#    #machine.restart()


print("123_Running.")

dimmer_percent = 0
def SetMotorPercent(percent):
    global dimmer_percent   
    v = (int)((percent / 100) * 65536)
    print("Setting Dimmer : %d, %d" % (percent, v)) 
    dimmer.duty_u16(v) # duty cycle 50% of 16 bit number
    dimmer_percent = percent
    

dimmer = PWM(Pin(18))
dimmer.freq(1000000)
dimmer.duty_u16(0) # duty cycle 50% of 16 bit number
SetMotorPercent(0)


#led = Pin(25, Pin.OUT)
heater = Pin(26, Pin.OUT)
#button = Pin(12, Pin.IN, Pin.PULL_DOWN)
button = Pin(0, Pin.IN, Pin.PULL_DOWN)
timer = Timer()

uart.write("heater and buttons setup")
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
uart.write("Temp Sensor Setup\r\n")

# https://docs.sunfounder.com/projects/thales-kit/en/latest/micropython/liquid_crystal_display.html#what-more

# IOExpander
#https://www.ti.com/lit/ds/symlink/pcf8574.pdf?ts=1627006546204&ref_url=https%253A%252F%252Fwww.google.com%252F
# LCD
# https://www.openhacks.com/uploadsproductos/eone-1602a1.pdf


state=1

rotary = RotaryIRQ(pin_num_clk=2, 
              pin_num_dt=1, 
              min_val=0, 
              max_val=1100, 
              reverse=False, 
              range_mode=RotaryIRQ.RANGE_BOUNDED)
uart.write("RotaryIRQ setup\r\n")


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


# http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-sample-time/
if fileExists('PID.config'):
    with open('PID.config') as configFile:
        value = configFile.readlines()
        print("Value: [%s] - %s" % (value, value[0]))
        bits = value[0].split(",")
        print(bits[0])
        print(bits[1])
        print(bits[2])
        p = (float)(bits[0]) / 100
        i = (float)(bits[1]) / 100
        d = (float)(bits[2]) / 100
        goalTemp = (float)(bits[3]) / 10
else:
    p = 0.1 #pX / 100
    i = 0
    d = 1
    goalTemp = 1044

print("P=%f, I=%f, D=%f,  goalTemp=%f" % (p,i,d,goalTemp))
pid = PID(p,i, d, setpoint=goalTemp, scale='ms')
pid.output_limits = (0, 1)    # Output value will be between 0 and 10
pid.set_auto_mode(True, last_output=0)
    

heater.value(0)
rotary.set(goalTemp*10)
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
            pressure = pressureADC.read_u16()
            display=False
            lcdDisplay = False 
            updatePID = False   
            updateConfig = False
            
            if(uart.any()):
                data = uart.readline().strip().decode('utf-8')
                
                if not data.startswith("tick"):
                    #print("Serial : %d" % len(data))
                    print("UART GOT : [%s]\r\n" % data)
                    
                    commands = data.split("\n")
                    for command in commands:                                        
                        if command.startswith("TEMP:"):
                            mode = 0
                            goalTemp = float(command[5:])
                            rotary.set(goalTemp * 10)
                            
                        elif command.startswith('kill'):
                            print("KILLING!")
                            uart.write("KILLING!!!")
                            os.remove("main.py")
                            machine.reset()
                            
                        elif command.startswith('reboot'):
                            uart.write("Rebooting")
                            machine.reset()
                            
                        elif command.startswith("PID:"):
                            mode = 0
                            bits = command[4:].split(",")
                            print(bits[0])
                            print(bits[1])
                            print(bits[2])
                            p = (float)(bits[0])
                            i = (float)(bits[1])
                            d = (float)(bits[2])
                            goalTemp = (float)(bits[3])
                            rotary.set(goalTemp * 10)
                                
                        
                        
            if mode == 0:
                if goalTemp != rotary.value()/10:
                    goalTemp = rotary.value()/10
                    print("New Set Point : %f" % goalTemp)                
                    lcdDisplay = True 
                    updatePID = True
                    updateConfig = True
                    pid.set_auto_mode(True)
            elif mode == 1:
                if p != rotary.value()/100:
                    p = rotary.value()/100
                    print("New P : %f" % p)                
                    lcdDisplay = True
                    updatePID = True
                    updateConfig = True
            elif mode == 2:
                if i != rotary.value()/100:
                    i = rotary.value()/100
                    print("New I : %f" % i)                
                    lcdDisplay = True
                    updatePID = True
                    updateConfig = True
            elif mode == 3:
                if d != rotary.value()/100:
                    d = rotary.value()/100
                    print("New D : %f" % d)                
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
            elif mode == 5:
                if dimmer_percent != rotary.value():
                    dimmer_percent = rotary.value()
                    print("New Pressure Point : %d" % dimmer_percent)                
                    lcdDisplay = True   
                    SetMotorPercent(dimmer_percent)       
                                        
            if time.ticks_ms() > switch_time:                
                temp = max.read()
                if temp > g_MaxTemp:
                    goalTemp = 0
                    lcd_clear()
                    lcd_write(0, 0, ('OVER_TEMP'))
                    heater.value(0)
                    timer.deinit()
                    raise Exception('OVER TEMP')
                    
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
                nextPrintTime = time.ticks_ms() + 500
                print("Val, %d, %d, %f, %f, %f, %f, %f, %f    " % (((t - t_start)/100)*10, goalTemp, pressure, temp, control, p, i, d))          
                #f.write("%d, %d, %d, %f, %f, %f, %f, %f\r\n" % (t - t_start, t, goalTemp, temp, control, p, i, d))
                
                msg = ("STAT,%d, %d, %f, %f, %f, %f, %f, %f\r\n" % (((t - t_start)/100)*10, goalTemp, pressure, temp, control, p, i, d))
                uart.write(msg.encode('utf-8'))
                
                
            if updatePID:
                pid = PID(p, i, d, setpoint=goalTemp, scale='ms')
                pid.output_limits = (0, 1)    # Output value will be between 0 and 10
                pid.set_auto_mode(True, last_output=0)
                print("PID UPDATED")
                
            if 0 == button.value():
                mode = mode + 1
                if mode > 5:
                    mode = 0              
                lcdDisplay = True
                            
                if 0 == mode:                 
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=1100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(goalTemp * 10)
                if 1 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=400, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(p * 100)
                if 2 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=400, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(i * 100)
                if 3 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=400, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set(d * 100)
                if 4 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set((int)(control*100))
                if 5 == mode:
                    rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
                    rotary.set((int)(dimmer_percent))
                
                sleep(.25)                
           
           
            if lcdDisplay:
                lcd_clear()
                if 0 == mode:
                    gt = (float)(goalTemp)
                    lcd_write(0, 0, ('Goal: %f(C), %f' % (gt,control)))
                    lcd_write(0, 1, ("%d, %d" % (temp, pressure)))    
                if 1 == mode:
                    lcd_write(0, 0, ("P = %f" % (p)))
                if 2 == mode:
                    lcd_write(0, 0, ("I = %f" % (i)))
                if 3 == mode:
                    lcd_write(0, 0, ("D = %f" % (d)))
                if 4 == mode:
                    lcd_write(0, 0, "*MANUAL HEATER*")
                    lcd_write(0, 1, ("%f %f(C)" % (control, temp)))
                if 5 == mode:
                    lcd_write(0, 0, "*MANUAL PUMP*")
                    lcd_write(0, 1, ("%d" % (dimmer_percent)))
                    
            if updateConfig:
                with open("PID.config", 'w') as save:
                    save.write("%d,%d,%d,%d" % (p*100, i*100, d*100, goalTemp*10))
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
    #print(e)
    print("Debugger Stopped..")
    
except Exception as e:
    heater.value(0)
    print("CRASHED")
    #sys.print_exception(e)
    f.write("Crashed")
    print("Not good..")
        
sleep(.25)
        
        
    
#lcd_clear()
#lcd_message("BYE")
heater.value(0)
print("BYE")
 

#timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)








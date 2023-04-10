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
from ads1x15 import ADS1015


global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/
#https://github.com/miketeachman/micropython-rotary

#pressureADC = ADC(28)
#while True:
#        v = pressure.read_u16()
#        print(v)
#        sleep(0.25)        

g_MaxTemp = 130

groupheadSolenoid = cs = machine.Pin(13, Pin.OUT)


pressure_sda = machine.Pin(20)#, machine.Pin.OUT)
pressure_scl = machine.Pin(21)#, machine.Pin.OUT)
pressureI2C = machine.SoftI2C(sda=pressure_sda, scl=pressure_scl, freq=100000)
pressureSensor = ADS1015(pressureI2C)
pressureSensor.gain = 0


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


print("123_Running.")

dimmer_percent = 0
def SetMotorPercent(percent):    
    global dimmer_percent          
    v = (int)((percent / 100) * 65536)       
    dimmer.duty_u16(v) # duty cycle 50% of 16 bit number
    dimmer_percent = percent
    

dimmer = PWM(Pin(18))
dimmer.freq(60) #1000000)

#https://www.amazon.com/gp/product/B06Y1DT1WP/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&psc=1
#50Hz or 60Hz PWM input signal up to 10kHz Input voltage level up to VCC (0-3.3V / 0-5V) 50HZ LED
dimmer.duty_u16(0) # duty cycle 50% of 16 bit number
SetMotorPercent(0)

heater = Pin(26, Pin.OUT)
button = Pin(0, Pin.IN, Pin.PUrrLL_DOWN)
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

def Dec2(d):
    d = d * 100
    d = (int)(d)
    d = d / 100
    return d


def convertPressure(measure):  
    # 0.5 V is 0
    # 5.0 V is 5V
    # Output: 0.5V–4.5V linear voltage output. 0 psi outputs 0.5V, 100 psi outputs 2.5V, 200 psi outputs 4.5V
    
    delta = 6.144 / 2048 #6.144 comes from PGA_6_144V (max value for 11 bits)
    v = measure * delta          
    p = v - 0.5
    pDelta = 200 / 4 #from alg above
    psi = p * pDelta
    bar = psi * 0.0689475729     
    return bar
    

def LoadConfigFile():
    global p
    global i
    global d
    global goalTemp
    
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
              max_val=3, 
              reverse=True, 
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


# Load Config file
LoadConfigFile()



print("P=%f, I=%f, D=%f,  goalTemp=%f" % (p,i,d,goalTemp))
pid = PID(p,i, d, setpoint=goalTemp, scale='ms')
pid.output_limits = (0, 1)    # Output value will be between 0 and 10
pid.set_auto_mode(True, last_output=0)
print("PID setup!")  

pressurePid = PID(250, 0, 0, setpoint=9, scale='ms')
pressurePid.output_limits = (60, 100)    # Output value will be between 0 and 10
pressurePid.set_auto_mode(True, last_output=0)
print("Pressure PID setup!")   
 

heater.value(0)
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
        heater_off_time = -1
    
    # Decide if we're beyond the cycle time period 
    if time.ticks_ms() > heater_cycle_time: 
        delta = time.ticks_ms() 
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
            heater.value(1)
            
    if -1 == heater_off_time:
        heater.value(0)
            
timer.init(freq=100, mode=Timer.PERIODIC, callback=toggle_heater)
print("Heater Toggle timer setup!")
        
try:
    nextPrintTime = 0
    t_start = time.ticks_ms()
    mode = 0
    
    t_resetTempTime = 0
    
    while True :        
        # f.flush()  
        t = time.ticks_ms() 
       
        if t_resetTempTime!=0 and t > t_resetTempTime:
            print("Reset Temperature")
            LoadConfigFile()
            t_resetTempTime = 0       
         
        pressure = convertPressure(pressureSensor.read(0))
                
        if 0 == button.value():
            print ("Button Down!! in mode %d" % (mode))
            dimmer_percent = pressurePid(pressure)
            print("Pressure: %f,   Dimmer: %d, Goal: %d" % (pressure, dimmer_percent, pressurePid.setpoint))
            SetMotorPercent(dimmer_percent) 
            
            if mode == 0 or mode == 1:
                groupheadSolenoid.value(1)
            elif mode == 3:
                goalTemp = 115
                t_resetTempTime = time.ticks_ms() + (2*60*1000)
            else:
                groupheadSolenoid.value(0)               
        else:
            SetMotorPercent(0)
            groupheadSolenoid.value(0)               
            
        display=False
        lcdDisplay = False 
        updatePID = False   
        updateConfig = False
        
        #
        # Process UART
        #
        # if(uart.any()):
        #     data = uart.readline().strip().decode('utf-8')
        #     if not data.startswith("tick"):
        #         #print("Serial : %d" % len(data))
        #         print("UART GOT : [%s]\r\n" % data)
                
        #         commands = data.split("\n")
        #         for command in commands:                                        
        #             if command.startswith("TEMP:"):
        #                 mode = 0
        #                 goalTemp = float(command[5:])
        #                 rotary.set(goalTemp * 10)
                        
        #             elif command.startswith('kill'):
        #                 print("KILLING!")
        #                 uart.write("KILLING!!!")
        #                 os.remove("main.py")
        #                 machine.reset()
                        
        #             elif command.startswith('reboot'):
        #                 uart.write("Rebooting")
        #                 machine.reset()
                        
        #             elif command.startswith("PID:"):
        #                 mode = 0
        #                 bits = command[4:].split(",")
        #                 print(bits[0])
        #                 print(bits[1])
        #                 print(bits[2])
        #                 p = (float)(bits[0])
        #                 i = (float)(bits[1])
        #                 d = (float)(bits[2])
        #                 goalTemp = (float)(bits[3])
        #                 rotary.set(goalTemp * 10)
        
        # Change modes
        if mode != rotary.value():
            mode = rotary.value()
            print("MODE: %d" % (mode))
        
            # Preinfuse
            if mode == 0:  #PreInfuse                         
                pressurePid.setpoint = 1
        
            if mode == 1: #Pull Shot                                
                pressurePid.setpoint = 9
                
            if mode == 2: #Steam              
                pressurePid.setpoint = 0
                
            if mode == 3: #Water           
                pressurePid.setpoint = 100
        
        # if mode == 0:
        #     if goalTemp != rotary.value()/10:
        #         goalTemp = rotary.value()/10
        #         print("New Set Point : %f" % goalTemp)                
        #         lcdDisplay = True 
        #         updatePID = True
        #         updateConfig = True
        #         pid.set_auto_mode(True)                
        #     if pressurePid.setpoint != 9:
        #         pressurePid.setpoint = 9
        # elif mode == 1:
        #     if p != rotary.value()/100:
        #         p = rotary.value()/100
        #         print("New P : %f" % p)                
        #         lcdDisplay = True
        #         updatePID = True
        #         updateConfig = True
        # elif mode == 2:
        #     if i != rotary.value()/100:
        #         i = rotary.value()/100
        #         print("New I : %f" % i)                
        #         lcdDisplay = True
        #         updatePID = True
        #         updateConfig = True
        # elif mode == 3:
        #     if d != rotary.value()/100:
        #         d = rotary.value()/100
        #         print("New D : %f" % d)                
        #         lcdDisplay = True
        #         updatePID = True
        #         updateConfig = True
        # elif mode == 4:
        #     if control != rotary.value()/100:
        #         control = rotary.value()/100
        #         print("Manual Control : %f, %d" % (control, rotary.value()))               
        #         lcdDisplay = True
        #         updatePID = True
        #         updateConfig = True   
        #         pid.set_auto_mode(False)    
        # elif mode == 5:
        #     if dimmer_percent != rotary.value():
        #         dimmer_percent = rotary.value()
        #         print("New Pressure Point : %d" % dimmer_percent)                
        #         lcdDisplay = True   
        #         SetMotorPercent(dimmer_percent)       
        # elif mode == 6:
        #     lcdDisplay = True  
        #     updatePID = True  
        #     if pressurePid.setpoint != 1:
        #         pressurePid.setpoint = 1
           
                   
        #
        # When enough time has gone by, read and update sensors
        #            
        if time.ticks_ms() > switch_time: 
            temp = max.read()
            pressure = convertPressure(pressureSensor.read(0))
            print("P:%f, T:%d" % (pressure, temp))
            
            if temp > g_MaxTemp:
                goalTemp = 0
                lcd_clear()
                lcd_write(0, 0, ('OVER_TEMP'))
                heater.value(0)
                SetMotorPercent(0)
                timer.deinit()
                raise Exception('OVER TEMP')
                
            pid.setpoint = goalTemp    
            # Compute new output from the PID according to the systems current value            
            if mode != 4:
                control = pid(temp) 
            else:
                control = rotary.value()
                
            switch_time = time.ticks_ms() + 2000                       
               
        #
        # Update serial
        # 
        if (time.ticks_ms() > nextPrintTime) or display:            
            lcdDisplay = True
            nextPrintTime = time.ticks_ms() + 250
            #print("Val,  %d, %d, D:%d, P:%f, %f, %f, %f, %f, %f    " % (((t - t_start)/100)*10, goalTemp, dimmer_percent, pressure, temp, control, p, i, d))          
            #f.write("%d, %d, %d, %f, %f, %f, %f, %f\r\n" % (t - t_start, t, goalTemp, temp, control, p, i, d))            
            msg = ("STAT,%d, %d, %d, %f, %f, %f, %f, %f, %f\r\n" % (((t - t_start)/100)*10, goalTemp, dimmer_percent, pressure, temp, control, p, i, d))
            uart.write(msg.encode('utf-8'))
                
        if updatePID:
            pid = PID(p, i, d, setpoint=goalTemp, scale='ms')
            pid.output_limits = (0, 1)    # Output value will be between 0 and 10
            pid.set_auto_mode(True, last_output=0)
            print("PID UPDATED")
            
        # if 0 == button.value():
        #     mode = mode + 1
        #     if mode > 6:
        #         mode = 0              
        #     lcdDisplay = True
                        
        #     if 0 == mode:                
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=1100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set(goalTemp * 10)                
        #         groupheadSolenoid.value(1)   
        #     if 1 == mode:
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=400, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set(p * 100)
        #         mode = 6                
        #         groupheadSolenoid.value(0)   
        #     if 2 == mode:
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=400, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set(i * 100)
        #         mode = 6               
        #         groupheadSolenoid.value(0)   
        #     if 3 == mode:
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=400, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set(d * 100)
        #         mode = 6
        #         groupheadSolenoid.value(0)
        #     if 4 == mode:
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set((int)(control*100))
        #         mode = 6
        #         groupheadSolenoid.value(0)
        #     if 5 == mode:
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set((int)(dimmer_percent))
        #         mode = 6
        #         groupheadSolenoid.value(0)
        #     if 6 == mode:
        #         rotary = RotaryIRQ(pin_num_clk=2, pin_num_dt=1,  min_val=0,max_val=100, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED)
        #         rotary.set((int)(dimmer_percent))
        #         groupheadSolenoid.value(1)
        #     sleep(.25)                
        
        
        if lcdDisplay:
            lcd_clear()    
            gt = (float)(goalTemp)
            
            if 0 == mode:
                lcd_write(0, 0, ('PreInfuse: %d/%d(C)' % (temp, gt)))
                lcd_write(0, 1, ("%s/%d" % (Dec2(pressure), pressurePid.setpoint))) 
            if 1 == mode:                
                lcd_write(0, 0, ('Pull: %d/%d(C)' % (temp, gt)))
                lcd_write(0, 1, ("%s/%d" % (Dec2(pressure), pressurePid.setpoint))) 
            if 2 == mode:
                lcd_write(0, 0, ('NOT-IMPL Steam %d/%d(C)' % (temp, gt)))
                lcd_write(0, 1, ("%s/%d" % (Dec2(pressure), pressurePid.setpoint))) 
            if 3 == mode:
                left = t_resetTempTime - time.ticks_ms()
                lcd_write(0, 0, ('Water: %d/%d(C) - %d' % (temp, gt, left)))
                lcd_write(0, 1, ("%s/%d" % (Dec2(pressure), pressurePid.setpoint))) 
                
        if updateConfig:
            with open("PID.config", 'w') as save:
                save.write("%d,%d,%d,%d" % (p*100, i*100, d*100, goalTemp*10))
        count = count + 1
                              

except KeyboardInterrupt:
    heater.value(0)
    SetMotorPercent(0)
    timer.deinit()
    f.write("Crashed")
    #print(e)
    print("Debugger Stopped..")
    
except Exception as e:
    heater.value(0)
    SetMotorPercent(0)
    print("CRASHED")
    print(str(e))
    f.write("Crashed")
    print("Not good..")

       
sleep(.25)
        
        
heater.value(0)
SetMotorPercent(0)
print("BYE")

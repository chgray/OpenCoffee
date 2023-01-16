from machine import Pin, Timer
import time
import machine, onewire, ds18x20
from lcd1602 import *

from time import sleep


global lcd
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/

print("Running.")

led = Pin(25, Pin.OUT)
button = Pin(12, Pin.IN, Pin.PULL_DOWN)
timer = Timer()

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
ds_pin = machine.Pin(2) 
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
 
roms = ds_sensor.scan()
 
lcd.message('Found DS devices')
lcd.message('Temperature (C)')




print("--------------------------------")
while True:
    b = button.value()
    
    try:
        ds_sensor.convert_temp()
        #sleep(1)
        for rom in roms:
            lcd.write(0,0, str(ds_sensor.read_temp(rom)))
        
        
        #print("Button: %d" % (b))
        if 0 == button.value():
            led.toggle()
            lcd.message("hello")
            if 1 == state:
                print("CLEAR")
                lcd.clear()
                
            elif 2 == state:
                lcd.send_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON )
                lcd.clear()
                lcd.message("display_ctl + on")
                
            elif 3 == state:
                lcd.send_command(LCD_DISPLAYCONTROL )
               
                
            elif 4 == state:
                lcd.send_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON )
          
            state = state + 1
            sleep(1)
    except:
        print("Not good..")
        
        
    



 
while True:
  ds_sensor.convert_temp()
  #sleep(1)
  for rom in roms:
    print(ds_sensor.read_temp(rom))
  #sleep(0.25)
 

def blink(timer):
    led.toggle()

#timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)







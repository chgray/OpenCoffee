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
sleep(2)
lcd.clear()


print("--------------------------------")
count = 0
while count < 1000:
    b = button.value()
    
    try:
        ds_sensor.convert_temp()
        #sleep(1)
        for rom in roms:
            lcd.write(0,0, str(ds_sensor.read_temp(rom)))
        
        lcd.write(0,1, str(count))
        count = count + 1
        
        if 0 == button.value():
            led.toggle()
            count += 100
            sleep(1)
            
    except Exception as e:
        print(e)
        print("Not good..")
        
        
    
lcd.clear()
lcd.message("BYE")
print("BYE")
 
def blink(timer):
    led.toggle()

#timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)








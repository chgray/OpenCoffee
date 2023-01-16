from machine import Pin, Timer
import time
import machine, onewire, ds18x20, boo
from time import sleep
# https://microcontrollerslab.com/ds18b20-raspberry-pi-pico-micropython-tutorial/


led = Pin(25, Pin.OUT)
button = Pin(12, Pin.IN, Pin.PULL_DOWN)
timer = Timer()

print('Setting up Temp')
ds_pin = machine.Pin(2) 
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
 
roms = ds_sensor.scan()
 
print('Found DS devices')
print('Temperature (°C)')
 
while True:
  ds_sensor.convert_temp()
  #sleep(1)
  for rom in roms:
    print(ds_sensor.read_temp(rom))
  #sleep(0.25)
 

def blink(timer):
    led.toggle()

#timer.init(freq=2.5, mode=Timer.PERIODIC, callback=blink)

while True:
    b = button.value()
    #print("Button: %d" % (b))
    if 0 == button.value():
        print("Button")
        led.toggle()
        time.sleep(.25)
        



from machine import Pin, Timer
import time
from time import sleep

from machine import Pin, PWM
from time import sleep
from ads1x15 import ADS1015

#from distutils.core import setup
#https://github.com/adafruit/Adafruit_CircuitPython_ADS1x15


print("Booted!")

sda = machine.Pin(20)#, machine.Pin.OUT)
scl = machine.Pin(21)#, machine.Pin.OUT)
onboard_led = machine.Pin(25, machine.Pin.OUT)


pressureI2C = machine.SoftI2C(sda=sda, scl=scl, freq=100000)
I2C_ADDR = pressureI2C.scan()

print("i2c")
print(I2C_ADDR)


pressure = ADS1015(pressureI2C)

while True:
    value = pressure.read(0)
    print(value)





pressure = ADS1015(pressureI2C)
value = pressure.read(0)

print ("Back")
print (value)


sda.value(1)
scl.value(0)
while True:
    sda.toggle()
    scl.toggle()
    onboard_led.toggle()
    print("flipX")
    sleep(1)
    
    


pressure = ADS1015(pressureI2C)
value = pressure.read(0)


print("Bye!")
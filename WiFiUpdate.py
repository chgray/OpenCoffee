import network, os
import socket
import time
import ubinascii
import hashlib
import binascii
import sys
import _thread
from WifiCreds import *

from machine import UART
from machine import Pin, Timer
from machine import Pin, Timer
import time

import machine, onewire, ds18x20, select

from time import sleep
from machine import Pin

from machine import WDT
import urequests

print ("Starting")

fixedLed = Pin("LED", Pin.OUT)
wdt = WDT(timeout=8000) #timeout is in ms
timer = Timer()

#fixedLed.value(1)
#sleep(2)
#fixedLed.value(0)

uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)


startTime = time.ticks_ms() 

def fileExists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False

def pokeWatchDog():
    global wdt
    global fixedLed
    global startTime
    
    if startTime + (6 * 60 *  1000) < time.ticks_ms():
        print("RESETING")        
        machine.reset()
    
    #led = Pin("LED", Pin.OUT)
    #fixedLed.value(0)
    #sleep(1)
    #print("WatchDog Poke")
    #fixedLed.value(1)
    wdt.feed() #resets countdown
    

def pokeWatchDogTimer(t):
    #print("Watchdog Timer")
    #fixedLed.toggle()
    pokeWatchDog()


print("Initing Timer")
timer.init(mode=Timer.PERIODIC, period=200, callback=pokeWatchDogTimer)
print("...timer inited")

class WifiLog(object):
    m_connected = False
    m_Exit = False

    def close_connection(self):

        if self.m_connected:
            print("Closing Socket:")
            self.m_socket.close()

        self.m_connected = False

    def close(self):
        print("close() - reseting!")
        machine.reset()
        self.close_connection()


        if self.m_Exit == False:
            print("Disconnecting and deactivating wifi")
            self.wlan.disconnect()
            self.wlan.active(True)
        self.m_Exit = True

    def connect_thread(self):
        fixedLed = Pin("LED", Pin.OUT)
        try:
            print("Getting Creds")
            creds = WifiCreds
            print ("Got Creds")
            
            while True:                    
                print("Scanning:")

                self.m_ssid = creds.SSID()
                self.m_password = creds.PWD()
                
                print("SSID : %s" % self.m_ssid)
                
                pokeWatchDog()
                scanlist = self.wlan.scan()

                print("Got Scanlist")
                for result in scanlist:
                    ssid, bssid, channel, RSSI, authmode, hidden = result
                    print("     %s == %s,  authmode=%d" % (ssid, ubinascii.hexlify(bssid), authmode))

                print("Scan CompleteX")

                pokeWatchDog()
                print("Poked")
                print("Connecting to %s" % self.m_ssid)
                self.wlan.connect(self.m_ssid, self.m_password)
                
                while True:
                    print('waiting for connection...  Status=%d, connected=%d' % (self.wlan.status(), self.wlan.isconnected()))
                    if self.wlan.status() < 0 or self.wlan.status() >= 3:
                        break
                    pokeWatchDog()
                    time.sleep(1)

                if self.wlan.isconnected():
                    #fixedLed.value(1)
                    print('connected')
                    status = self.wlan.ifconfig()
                    print( 'ip = ' + status[0] )
                    uart.write('ip' + status[0])
                else:
                    print('network connection failed')
                    continue

                addr = socket.getaddrinfo('0.0.0.0', 25)[0][-1]
                
                # periodic at 1kHz
                timer.init(mode=Timer.PERIODIC, period=2000, callback=pokeWatchDogTimer)
                
                print("Downloading")
                url = "http://raw.githubusercontent.com/chgray/OpenCoffee/user/chgray/se2/WiFiUpdate.py"
                response = urequests.get(url)
                
               

                hash_object = hashlib.sha256(response.text)
                hex_dig = binascii.hexlify(hash_object.digest())     
                
                if(fileExists("main.py")):
                    f = open('main.py')
                    orig_file = f.read()               
                    f.close()
                
                    orig_hash_object = hashlib.sha256(orig_file)
                    orig_hex_dig = binascii.hexlify(orig_hash_object.digest())
                else:
                    orig_hex_dig = "No File"
                
                print("Orig_Hex: %s" % orig_hex_dig)                
                print(" New_Hex: %s" % hex_dig)
                
                print(response)
                with open("main.py2", "w") as file:
                    file.write(response.text)
                
                if(fileExists("main_old.py")):
                    os.remove("main_old.py")
                if(fileExists("main.py")):  
                    os.rename("main.py", "main_old.py")
                os.rename("main.py2", "main.py")
                print("Downloaded and saved!")
                
                print("BYE")
                
                for y in range(0, 10):
                    fixedLed.value(1)                    
                    sleep(0.1)
                    fixedLed.value(0)
                    sleep(0.1)
                    
                machine.reset()
            
        except KeyboardInterrupt as e:
            print("KEYBOARD INTERUPT!")
            #self.close()
        except OSError as e:
                         
            print(" Reseting...")
            sys.print_exception(e)
        	#machine.reset()

    def __init__(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)





w = WifiLog()
#w.log("hi")
#w.connect_thread()



def ManageWifi():
    try:
        print("Starting connect thread")
        w.connect_thread()
        print("Wifi Gone")
    except OSError as e:
        w.close()
        print('Closing Up...')
        print("OSERROR() - reseting!")
        #machine.reset()

    except KeyboardInterrupt as e:
        w.close()
        print('Keyboard-Closing Up...')
        print("KEYBOARDINTERRUPT() - reseting!")
        #machine.reset()


try:
    ManageWifi()

except KeyboardInterrupt as e:
    w.close()


print("Bye!! - xyz")

                        



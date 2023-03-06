import network
import socket
import time
import ubinascii
import _thread

from machine import UART
from machine import Pin, Timer
from machine import Pin, Timer
import time

import machine, onewire, ds18x20, select

from time import sleep
from machine import Pin

from machine import WDT
wdt = WDT(timeout=5000) #timeout is in ms
timer = Timer()

uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)

def watchDogDisabled(t):
    #print("Tick");
    wdt.feed() #resets countdown

class WifiLog(object):
    
    m_ssid = 'Hello'
    m_password = 'deadbeef01'
   
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
        try:
            while True:
                    print("Hi wifi")
                    print("Scanning:")                   
                    #machine.reset()
                    
                    scanlist = self.wlan.scan()
                    print("Got Scanlist")
                    for result in scanlist:
                      ssid, bssid, channel, RSSI, authmode, hidden = result
                      print("     %s == %s,  authmode=%d" % (ssid, ubinascii.hexlify(bssid), authmode))

                    print("Scan Complete")
                    
                    self.wlan.connect(self.m_ssid, self.m_password)
                    
                    while True:
                        print('waiting for connection...  Status=%d, connected=%d' % (self.wlan.status(), self.wlan.isconnected()))
                        if self.wlan.status() < 0 or self.wlan.status() >= 3:
                            break
                        wdt.feed() #resets countdown
                        time.sleep(1)

                    if self.wlan.isconnected():
                        print('connected')
                        status = self.wlan.ifconfig()
                        print( 'ip = ' + status[0] )
                        uart.write('ip' + status[0])
                    else:
                        print('network connection failed')
                        continue

                    addr = socket.getaddrinfo('0.0.0.0', 25)[0][-1]

                    s = socket.socket()
                    s.bind(addr)
                    s.listen(1)

                    print('listening on', addr)
                
                   
                    # periodic at 1kHz
                    timer.init(mode=Timer.PERIODIC, freq=2000, callback=watchDogDisabled)

                    # Listen for connections
                    needExit = False
                    while needExit == False:
                            timer.deinit()
                        
                            clientSock, addr = s.accept()
                            poller = select.poll()
                            poller.register(clientSock, select.POLLIN)
                            
                            print('client connected from', addr)
                                                       
                            #clientSock.settimeout(5000)                           
                            clientSock.send("Welcome to OpenCoffee\r\n") 
                            
                            while True:    
                                res = poller.poll(100)  # time in milliseconds
                                wdt.feed()
                                
                                if res:                                    
                                    inputBuffer = clientSock.recv(1000)
                                    print('socket got data bytes=%d' % len(inputBuffer))
                                    #print("line complete")
                                    
                                    commands = inputBuffer.strip().decode('utf-8').split("\n")
                                    for command in commands:                                        
                                        if command is 'kill':
                                            print("KILLING!")
                                            clientSock.write("KILLING!!!")
                                            os.remove("main.py")
                                            machine.reboot()
                                        elif command is 'reboot':
                                            machine.reset()
                                        elif command.startswith("PASS:"):
                                            clientSock.write("PASSING OFF %s\r\n" % command[5:])
                                            uart.write(command[5:])
                                        else:
                                            clientSock.write("Unknown command %s, all I know is kill and reboot\r\n" % command)
                                            
                        
                                    print(inputBuffer)
                                            
                                if(uart.any()):
                                    data = uart.read()
                                    #uart.write("hello world")
                                    #print("D %d " % len(data))
                                    #data_string = data.decode('utf-8')
                                    print("Serial : %d" % len(data))
                                    clientSock.write(data)
                                #print("DTA %s" % (string)data)                                
                                #sleep(2)
                                #print("Waiting.")
                                uart.write("tick\r\n")
                                #clientSock.send(".")
                     
                    print("Received Exit : Closing Listening Socket")
                    s.close
                    sleep(5)
                
        except KeyboardInterrupt as e:
                print("KEYBOARD INTERUPT!")
                #self.close()
    
    def __init__(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        
   
    
led = Pin(25, Pin.OUT)

w = WifiLog()
#w.log("hi")
#w.connect_thread()



def ManageWifi():
    try:
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
    
    
print("Bye!!")
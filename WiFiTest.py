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
wdt = WDT(timeout=10000) #timeout is in ms

uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)


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

                    addr = socket.getaddrinfo('0.0.0.0', 23)[0][-1]

                    s = socket.socket()
                    s.bind(addr)
                    s.listen(1)

                    print('listening on', addr)
                

                    # Listen for connections
                    needExit = False
                    while needExit == False:
                            clientSock, addr = s.accept()
                            print('client connected from', addr)
                            
                            clientSock.settimeout(5000)                           
                            clientSock.send("Welcome to OpenCoffee\r\n")
                            
                            
                            poller = select.poll()
                            poller.register(clientSock, select.POLLIN)
                            
                                 
                            inputBuffer = bytearray(0)
                            
                            while True:    
                                res = poller.poll(100)  # time in milliseconds
                                
                                if res:                                    
                                    print('socket got data')
                                    
                                    inputBuffer += clientSock.recv(100)
                                    print("line complete")
                                    
                                    #print(inputBuffer)
                                    #s = inputBuffer.decode("utf-8")
                                    #print(s)
                                    #print(input.decode("utf-8").split("\n"))
                                    
                                    #toss = clientSock.recv(100)
                                    #data = toss.decode("utf-8")
                                    #data = clientSock.readline().decode("utf-8").strip()
                                    
                                    #if data is 'kill':
                                    #    print("KILLING!")
                                    #    clientSock.write("KILLING!!!")
                                    #    os.remove("main.py")
                                    #elif data is 'reboot':
                                    #    machine.reset()
                                    #else:
                                    #    clientSock.write("Unknown command %s, all I know is kill and reboot" % data)
                                        
                        
                                    print(inputBuffer)
                                            
                                if(uart.any()):
                                    data = uart.read()
                                    #uart.write("hello world")
                                    #print("D %d " % len(data))
                                    #data_string = data.decode('utf-8')
                                    print(data)
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
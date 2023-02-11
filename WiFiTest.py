import network
import socket
import time
import ubinascii
import _thread
from machine import UART
from machine import Pin, Timer
from machine import Pin, Timer
import time

import machine, onewire, ds18x20


from time import sleep
from machine import Pin


uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)

uart.write("UART: welcome to OpenCoffee")
#flipper = Pin(4, Pin.OUT)
#flipper2 = Pin(5, Pin.OUT)
#flipper.value(1)
#flipper2.value(0)
#while True:
#    uart.write("hi")
    #flipper.toggle()
    #flipper2.toggle()
#    print("hi")
#    sleep(1)
    

class WifiLog(object):
    
    m_ssid = 'Hello'
    m_password = 'deadbeef01'
    m_lock = _thread.allocate_lock()
    m_connected = False
    m_Exit = False
    
    def close_connection(self):
        self.m_lock.acquire()
        if self.m_connected:
            print("Closing Socket:")
            self.m_socket.close()
        
        self.m_connected = False
        self.m_lock.release()
    
    def close(self):
        print("close() - reseting!")
        machine.reset()
        self.close_connection()
        
        self.m_lock.acquire()
        if self.m_Exit == False:
            print("Disconnecting and deactivating wifi")
            self.wlan.disconnect()
            self.wlan.active(True)
        self.m_Exit = True
        self.m_lock.release()
        
        
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
                        time.sleep(1)

                    if self.wlan.isconnected():
                        print('connected')
                        status = self.wlan.ifconfig()
                        print( 'ip = ' + status[0] )
                    else:
                        print('network connection failed')
                        continue

                    addr = socket.getaddrinfo('0.0.0.0', 22)[0][-1]

                    s = socket.socket()
                    s.bind(addr)
                    s.listen(1)

                    print('listening on', addr)

                    # Listen for connections
                    needExit = False
                    while needExit == False:
                            cl, addr = s.accept()
                            print('client connected from', addr)
                            
                            cl.send("Welcome to OpenCoffee")
                            
                            self.m_lock.acquire()
                            self.m_socket = cl
                            self.m_connected = True
                            connected = True
                            self.m_lock.release()
                            
                            # Wait for disconnect
                            while connected:
                                sleep(1)
                                self.m_lock.acquire()
                                connected = self.m_connected
                                self.m_lock.release()
                                
                            print("Going back for a relisten")
                            needExit = False;
                            
                            self.m_lock.acquire()
                            needExit = self.m_Exit
                            self.m_lock.release()

                     
                    print("Received Exit : Closing Listening Socket")
                    s.close
                    sleep(5)
                
        except KeyboardInterrupt as e:
                self.close()
    
    def __init__(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        
    def log(self, data):
        disconnect = False
        self.m_lock.acquire()
        if self.m_connected:
            print(data)
            try:
                self.m_socket.write(data)
            except:
                print("Socket Broken");
                disconnect = True
        else:
            print(data)
            
        self.m_lock.release()
        
        
        if disconnect:
            self.close_connection()
            print("disconnect() - reseting!")
            #machine.reset()
    
    
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
    
def RunCoffee():
    global w
    try:
        i = 0
        while True:                
            if(uart.any()):
                data = uart.read()
                #uart.write("hello world")
                #print("D %d " % len(data))
                #data_string = data.decode('utf-8')
                #print("%s" % data)
                w.log(data)   
            #print("DTA %s" % (string)data)
            i = i + 1
            sleep(2)
            print("Waiting.")
            uart.write("tick\r\n")
    except OSError as e:
        print("RunCoffee() exception")
        print(e)
        #print(e)
        machine.reset()
   
try:
    _thread.start_new_thread(RunCoffee, ())
    ManageWifi()

except KeyboardInterrupt as e:
    w.close()
    
    
print("Bye!!")
import network
import socket
import time
import ubinascii
import _thread
from machine import Pin, Timer
from machine import Pin, Timer
import time

import machine, onewire, ds18x20


from time import sleep
from machine import Pin


class WifiLog(object):
    
    m_ssid = 'Hello'
    m_password = 'deadbeef01'
    m_lock = _thread.allocate_lock()
    m_connected = False
    
    def connect_thread(self):
        while True:
                print("Hi wifi")
                print("Scanning:")
                
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
                    raise RuntimeError('network connection failed')

                addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

                s = socket.socket()
                s.bind(addr)
                s.listen(1)

                print('listening on', addr)

                # Listen for connections
                while True:
                    try:
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

                    except OSError as e:
                        cl.close()
                        print('connection closed')
                
                
                
                sleep(5)
    
    def __init__(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        
    def log(self, data):
        self.m_lock.acquire()
        if self.m_connected:
            print("C: Log %s" % (data))
            try:
                self.m_socket.write("Log %s" % (data))
            except OSError as e:
                self.m_socket.close()
                print('connection closed')
                self.m_connected = False
        else:
            print("D: Log %s" % (data))
            
        self.m_lock.release()
    
    
led = Pin(25, Pin.OUT)

w = WifiLog()
#w.log("hi")
#w.connect_thread()

def xyz():
    w.connect_thread()
    print("Out")
    
def mno():
    global w
    i = 0
    while True:        
        w.log("Boo: %d" % (i))
        i = i + 1
        sleep(2)
        

_thread.start_new_thread(mno, ())

xyz()
#xyz()

i=0

exit(1)

my_ssid = 'Hello'
password = 'deadbeef01'







#https://docs.micropython.org/en/v1.8/esp8266/library/network.html?highlight=wlan#network.WLAN


print("Done Scan, ssid=%s  pwd=%s" % (my_ssid, password))

wlan.connect(my_ssid, password)


max_wait = 10
while max_wait > 0:
    print('waiting for connection...  Status=%d, connected=%d' % (wlan.status(), wlan.isconnected()))
    if wlan.status() < 0 or wlan.status() >= 3:
        break
    max_wait -= 1
    time.sleep(1)

if wlan.isconnected():
    print('connected')
    status = wlan.ifconfig()
    print( 'ip = ' + status[0] )
else:
    raise RuntimeError('network connection failed')

addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1]

s = socket.socket()
s.bind(addr)
s.listen(1)

print('listening on', addr)

# Listen for connections
while True:
    try:
        cl, addr = s.accept()
        print('client connected from', addr)
        #request = cl.recv(1024)
        #print(request)
        #request = str(request)
        
        for i in range(1, 99999999999):
            ret = cl.send("%d, %d\r\n" % (i, i))
            print("RET: %d" % (ret))
            
        cl.close()

    except OSError as e:
        cl.close()
        print('connection closed')
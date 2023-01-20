import network
import socket
import time
import ubinascii

from machine import Pin


class WifiLog():
    def __init__():
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
       
    def log(data):
        print("Log")
    
    
    
led = Pin(25, Pin.OUT)

my_ssid = 'Hello'
password = 'deadbeef01'


wlan = network.WLAN(network.STA_IF)
wlan.active(True)



#https://docs.micropython.org/en/v1.8/esp8266/library/network.html?highlight=wlan#network.WLAN

print("Scanning:")
scanlist = wlan.scan()
print("Got Scanlist")
for result in scanlist:
  ssid, bssid, channel, RSSI, authmode, hidden = result
  print("%s == %s,  authmode=%d" % (ssid, ubinascii.hexlify(bssid), authmode))

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
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


fixedLed = Pin("LED", Pin.OUT)
wdt = WDT(timeout=8000) #timeout is in ms
timer = Timer()

fixedLed.value(1)
fixedLed.toggle()
uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))
uart.init(bits=8, parity=None, stop=2)


startTime = time.ticks_ms() 

def pokeWatchDog():
    global wdt
    global fixedLed
    global startTime
    
    if startTime + (6 * 60 *  1000) < time.ticks_ms():
        print("RESETING")        
        machine.reset()
    
    #led = Pin("LED", Pin.OUT)
    fixedLed.value(0)
    sleep(1)
    #print("WatchDog Poke")
    fixedLed.value(1)
    wdt.feed() #resets countdown
    

def pokeWatchDogTimer(t):
    #print("Watchdog Timer")
    pokeWatchDog()


timer.init(mode=Timer.PERIODIC, period=2000, callback=pokeWatchDogTimer)


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
        try:
            while True:
                    print("Hi wifi")
                    print("Scanning:")


                    pokeWatchDog()
                    scanlist = self.wlan.scan()

                    print("Got Scanlist")
                    for result in scanlist:
                      ssid, bssid, channel, RSSI, authmode, hidden = result
                      print("     %s == %s,  authmode=%d" % (ssid, ubinascii.hexlify(bssid), authmode))

                    print("Scan Complete")

                    pokeWatchDog()
                    print("Connecting to %s" % self.m_ssid)
                    self.wlan.connect(self.m_ssid, self.m_password)

                    while True:
                        print('waiting for connection...  Status=%d, connected=%d' % (self.wlan.status(), self.wlan.isconnected()))
                        if self.wlan.status() < 0 or self.wlan.status() >= 3:
                            break
                        pokeWatchDog()
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

                    pokeWatchDog()
                    s = socket.socket()
                    s.bind(addr)
                    s.listen(1)

                    print('listening on', addr)


                    # periodic at 1kHz
                    timer.init(mode=Timer.PERIODIC, period=2000, callback=pokeWatchDogTimer)
                    clientSock, addr = s.accept()
                    #timer.deinit()
                    print('client connected from', addr)

                    poller = select.poll()
                    poller.register(clientSock, select.POLLIN)

                    #clientSock.settimeout(5000)
                    clientSock.send("Welcome to OpenCoffee\r\n")
                    startTime = time.ticks_ms() 
                    
                    while True:
                        res = poller.poll(100)  # time in milliseconds
                        pokeWatchDog()

                        if res:
                            inputBuffer = clientSock.recv(1000)
                            print('socket got data bytes=%d' % len(inputBuffer))
                            #print("line complete")

                            try:
                                commands = inputBuffer.strip().decode('utf-8').split("\n")
                                for command in commands:
                                    if command is 'kill':
                                        print("KILLING!")
                                        clientSock.write("KILLING!!!")
                                        os.remove("/main.py")
                                        clientSock.write("file deleted")
                                        machine.reset()
                                        clientSock.write("...reboot didnt happen")
                                    elif command is 'reboot':
                                        machine.reset()
                                    elif command.startswith("PASS:"):
                                        clientSock.write("PASSING OFF %s\r\n" % command[5:])
                                        uart.write(command[5:])
                                    else:
                                        clientSock.write("Unknown command %s, all I know is kill and reboot\r\n" % command)

                            except:
                                print("Serial Input Error")

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
                        #uart.write("tick\r\n")
                        #clientSock.send(".")

                    print("Received Exit : Closing Listening Socket")
                    s.close
                    sleep(5)

        except KeyboardInterrupt as e:
                print("KEYBOARD INTERUPT!")
                #self.close()
	except:
		machine.reset()

    def __init__(self):
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)





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

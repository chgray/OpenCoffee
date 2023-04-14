import network, os
import urequests, gc
import socket
import time
import io
import ubinascii
import hashlib
import binascii
import sys
import _thread
from WifiCreds import *

#hi

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
#wdt = WDT(timeout=8000) #timeout is in ms
timer = Timer()


fixedLed.value(1)
sleep(1)
fixedLed.value(0)

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
    fixedLed.toggle()
    #global wdt
    #global fixedLed
    #global startTime

    #if startTime + (6 * 60 *  1000) < time.ticks_ms():
    #    print("RESETING")
    #    machine.reset()

    #led = Pin("LED", Pin.OUT)
    #fixedLed.value(0)
    #sleep(1)
    #print("WatchDog Poke")
    #fixedLed.value(1)
    #wdt.feed() #resets countdown
    #fixedLed.toggle()

def pokeWatchDogTimer(t):
    #print("Watchdog Timer")
    #fixedLed.toggle()
    pokeWatchDog()


#print("Initing Timer")
#timer.init(mode=Timer.PERIODIC, period=200, callback=pokeWatchDogTimer)
#print("...timer inited")




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


#
# Read update file
#
import ujson
class CoffeeUpdater(object):

    def __init__(self, downloader):
        print("OpenCoffee Init")
        self.downloader = downloader

    def Update(self):
        # Bootstrap ourselves with DeviceConfig on disk
        #localConfig = self.downloader.LoadFile("DeviceConfig.json")
        #localConfigJson = ujson.load(io.StringIO(localConfig.Content()))

        creds = WifiCreds()
        print("Got creds : %s" % creds.DeviceFunction())

        # Retrieve our high water config file
        targetConfig = self.downloader.LoadContent(creds.DeviceFunction())
        #targetConfig = self.downloader.LoadContent(localConfigJson["DeviceFunction"])

        print("Downloaded...")

        targetConfigJson = ujson.load(io.StringIO(targetConfig.Content()))

        print(targetConfig.Content())
        print("-----")

        # if targetConfig.Hash() == targetConfigJson["DeviceFunctionHash"]:
        #     print ("Downloaded file looks good")
        # else:
        #     print ("ERROR: Dont update - corrupted download")
        #     print ("Expected Hash: %s" % targetConfigJson["DeviceFunctionHash"])
        #     print ("Actual Hash: %s" % targetConfig.Hash())
        #     return

        print("Device Function : %s" % targetConfigJson["DeviceFunction"])

        for key, value in enumerate(targetConfigJson['Files']):
            print ("Updating: %s" % value["FileName"])
            print ("    URL=" + value["URL"])
            print ("    EXPECTED_HASH=" + value["Hash"])

            try:
                localFile = self.downloader.LoadFile(value["FileName"])
                needUpdate = False

                if localFile == None:
                    print("    LocalFile=FALSE;  forcing update")
                    needUpdate = True
                else:
                    print("    LocalFile=TRUE;  checking hashs to see if update is needed")
                    print("        LocalFileHash=%s" % localFile.Hash())
                    if localFile.Hash() != value["Hash"]:
                        print("    UPDATE REQUIRED;  hashes differ")
                        needUpdate = True
                    else:
                        print("    UPDATE NOT REQUIRED; hashes are the same - %s" % localFile.Hash())

                if needUpdate:
                    print("    DOWNLOADING %s" % value["URL"])
                    content = self.downloader.LoadContent(value["URL"])

                    if content.Hash() == value["Hash"]:
                        print ("    HASH of downloaded file verfied!")
                    else:
                        print ("    HASH of downloaded file corrupted - corruption on the network, or with the configuration JSON")
                        print ("    ACTUAL HASH : %s" % content.Hash())
                        needUpdate = False


                if needUpdate:
                    print("")
                    print("    Performing Update...")
                    with open(value["FileName"] + ".temp", "wb") as file:
                        print("    Writing temp file")
                        file.write(content.Content())

                    if localFile != None:
                        print("   Removing original file")
                        os.remove(value["FileName"])

                    print("    Perform rename")
                    os.rename(value["FileName"] + ".temp", value["FileName"])

                    print("    SUCCESS: %s updated!" % value["FileName"])


            except OSError as e:
                print("OSERROR() - download - reseting!")
                #machine.reset()


class ChecksumContent(object):

    def __init__(self, content):
        #print("ChecksumContent : %d" % len(content))
        self.shaHash = hashlib.sha256(content)
        self.hexHash = binascii.hexlify(self.shaHash.digest())
        self.hexHash = self.hexHash.decode('utf-8')
        self.content = content

        #print(self.Hash())
        #print("HexHash %s" % self.hexHash)

    def Content(self):
        return self.content

    def Hash(self):
        return self.hexHash

class CoffeeFileDownloader(object):
    def __init__(self):
        print("OpenCoffee Downloader (FILE)")

    def LoadFile(self, location):
        #print("Getting %s from FILESYSTEM" % location)

        if fileExists(location) == False:
            return None

        with open(location, 'rb') as f:
            ret = ChecksumContent(f.read())
            return ret

    def LoadContent(self, location):
        #print("Getting %s from FILESYSTEM" % location)

        if fileExists(location) == False:
            return None

        with open(location, 'rb') as f:
            ret = ChecksumContent(f.read())
            return ret



class CoffeeFileDownloaderWifi(CoffeeFileDownloader):
    def __init__(self):
        super().__init__()
        print("OpenCoffee Downloader (Wifi)")

        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(False)

        fixedLed.value(1)
        sleep(1)
        fixedLed.value(0)

    def Connect(self):
        creds = WifiCreds()
        self.m_ssid = creds.SSID()
        self.m_password = creds.PWD()

        # scanlist = self.wlan.scan()
        # for result in scanlist:
        #     ssid, bssid, channel, RSSI, authmode, hidden = result
        #     print("     %s == %s,  authmode=%d" % (ssid, ubinascii.hexlify(bssid), authmode))

        print("Connecting to %s with password %s" % (self.m_ssid, self.m_password))

        self.wlan.active(True)
        fixedLed.value(1)
        sleep(3)
        self.wlan.connect(self.m_ssid, self.m_password)

        while self.wlan.isconnected() == False:
            print('waiting for connection...  Status=%d, connected=%d' % (self.wlan.status(), self.wlan.isconnected()))
            sleep(1)

        if self.wlan.isconnected():
            print('connected')
            status = self.wlan.ifconfig()
            print( 'ip = ' + status[0] )
        else:
            print('network connection failed')
            machine.reset()

    def LoadContent(self, location):
        print("Getting %s from WIFI" % location)
        response = urequests.get(location)
        hash_object = hashlib.sha256(response.text)
        hex_dig = binascii.hexlify(hash_object.digest())
        ret = ChecksumContent(response.text)
        return ret


#
# On PicoW init wifi
#
import network


try:
    if hasattr(network, "WLAN"):
        print("WIFI")
        downloader = CoffeeFileDownloaderWifi()
        downloader.Connect()
    else:
        #downloader = CoffeeFileDownloader()
        print("NOT WIFI DEVICE!!!! - EXITING")
        exit(1)

    cu = CoffeeUpdater(downloader)
    cu.Update()

except OSError as e:
        print('Closing Up...')
        print("OSERROR() - reseting!")
        #machine.reset()

except KeyboardInterrupt as e:
    print('Keyboard-Closing Up...')
    print("KEYBOARDINTERRUPT() - reseting!")
    #machine.reset()

for count in range(0,10000):
    fixedLed.toggle()
    sleep(0.05)
    
print("Bye!! - xyz")
machine.reset()
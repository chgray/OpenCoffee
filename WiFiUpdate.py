import network, os
import urequests
import ustruct
import time
import io
import hashlib
import binascii
import time
import machine
import ujson

from WifiCreds import *
from machine import UART
from machine import Pin, Timer
from machine import Pin, Timer
from time import sleep
from machine import Pin

fixedLed = Pin("LED", Pin.OUT)
#wdt = WDT(timeout=8000) #timeout is in ms
timer = Timer()

fixedLed.value(1)
sleep(1)
fixedLed.value(0)


# class CoffeeSerialServer(object):
#     def __init__(self, uartNum, txPin, rxPin):
#         uart = UART(uartNum, baudrate=115200, tx=Pin(txPin), rx=Pin(rxPin))
#         uart.irq(UART.RX_ANY, priority=1, handler=None, wake=machine.IDLE)

#     def callback

def callback(data):
    print("INTERRUPT")
    print(data)

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


#
# Read update file
#
class CoffeeUpdater(object):

    def __init__(self, downloader):
        print("OpenCoffee Init")
        self.downloader = downloader

    def Update(self):
        creds = WifiCreds()
        print("Got creds : %s" % creds.DeviceFunction())

        # Retrieve our high water config file
        targetConfig = self.downloader.LoadContent(creds.DeviceFunction())
        print("Downloaded...")

        targetConfigJson = ujson.load(io.StringIO(targetConfig.Content()))

        print(targetConfig.Content())

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

class CoffeeCommands():
    RETRANSMIT = 1
    GET_FILE = 2
    
    
class CoffeePacket(object):
    
    def __init__(self, opCode, packetID, len, data):
        self.OpCode = opCode
        self.PacketId = packetID
        self.Length = len
        self.Data = data
    
    def __str__(self):
        oc = self.OpCode
        ret = "CoffeePacket: OpCode={opCode}, PacketId={pID}".format(opCode=oc, pID=self.PacketId)
        return ret
        
    def OpCode(self):
        return self.OpCode
    
    def PacketId(self):
        return self.PacketId
    
class CoffeeSerialServer(object):
    def __init__(self, uartNum, txPin, rxPin):
        self.uart = UART(uartNum, baudrate=115200, tx=Pin(txPin), rx=Pin(rxPin))
        self.uart.write("Hi") # <-- this intentionally puts us 2 bytes offset, so we can test the code to align()
        
        
    def SendPacket(self, opCode, data):
        return SendHeader(opCode, 0)
    
    def Process(self):
        if(self.uart.any() != 0):
            p = self.ReadPacket()
            print("Got Packet : ID=%d" % p.PacketId())
        
            
        
    def SendHeader(self, opCode, len):
        fmt = 'IIII'
        rlen = ustruct.calcsize(fmt)
        buf = bytearray(rlen)
        ustruct.pack_into(fmt, buf, 0, 0xAAAAAAAA, opCode, len, 0xDDDDDDDD)
        self.uart.write(buf)

    def SendRetransmitRequest(self):
        return self.SendHeader(CoffeeCommands.RETRANSMIT, 0)
        
    def ReadPacket(self):
        # Read one byte;  the rest of our header
        fmt = 'IIII'
        rlen = ustruct.calcsize(fmt)        
        rxData = bytes()          
        startRead = time.ticks_ms()          
        while True:
            if len(rxData) == rlen:                
                break
             
            if(self.uart.any() == 0):
                 if(time.ticks_ms() - startRead > 1000):
                    print("TIMING OUT")
                    self.Align()
                    print("...reading again")
                    self.SendRetransmitRequest()
                    return self.ReadPacket()
                 
                 time.sleep_ms(1)
                 continue
            
            else:     
                #print("RXDataLen: %d of %d" % (len(rxData), rlen))           
                rxData += self.uart.read(1)
               

        reframe = False
        if(len(rxData) == rlen):
            unpacked_data = ustruct.unpack(fmt, rxData)

            if(unpacked_data[0] != 0xAAAAAAAA or unpacked_data[3] != 0xDDDDDDDD):               
                print("CORRUPT PACKET - need REFRAME!")
                reframe = True
        else:
            print("ERROR: short read")
            reframe = True


        if(reframe):
            print("ERROR: Reframing Packet!")
            self.SendRetransmitRequest()
            self.Align()
            self.SendRetransmitRequest()
            return self.ReadPacket()

        #print(unpacked_data)
        #print(".....")
        ret = CoffeePacket(unpacked_data[1], 80, unpacked_data[2], 0)
        return ret

    #https://docs.micropython.org/en/latest/library/struct.html
    def Align(self):
        print("Aligning")
        self.SendRetransmitRequest()
        self.SendRetransmitRequest()
        
        while True:
            fmt = 'B'
            rlen = ustruct.calcsize(fmt)

            # Read one byte;  looking for 0xFF
            rxData = bytes()
            hits = 0
            while self.uart.any() != 0:
                rxData += self.uart.read(1)
                #print("RXDataLen: %d of %d" % (len(rxData), rlen))
                time.sleep_ms(1)

                unpacked_data = ustruct.unpack(fmt, rxData)
                print(unpacked_data[0])
                if unpacked_data[0] != 0xAA and hits == 4:
                    print("Found end of marker")
                    break
                elif unpacked_data[0] == 0xAA:
                    print("HIT! %d" % hits)
                    rxData = bytes()
                    hits = hits + 1
                else:
                    print("Not end of marker")
                    hits = 0
                    rxData = bytes()

            print("GOOD")

             # Read one byte;  the rest of our header
            fmt = 'III'
            rlen = ustruct.calcsize(fmt)
            while self.uart.any() != 0 and len(rxData) < rlen:
                rxData += self.uart.read(1)
                #print("RXDataLen: %d of %d" % (len(rxData), rlen))
                time.sleep_ms(10)

            #print("Finished Reading DataLen: %d (left=%d)" % (len(rxData), self.uart.any()))
            unpacked_data = ustruct.unpack(fmt, rxData)
            print(unpacked_data)
            if(unpacked_data[2] != 0xDDDDDDDD):
                print("TAIL OF HEADER BAD - aligning again")                
                return self.Align()

            print(unpacked_data)
            print("**ALIGNED!!")
            return

            # if unpacked_data[0] == 0xFF:
            #    print("MISS!")
            #    continue





print("Trying Serial")
s = CoffeeSerialServer(0, 0, 1)
s.SendRetransmitRequest()
print(str(s.ReadPacket()))
s.SendRetransmitRequest()
print("---***---***---***---***")
print(str(s.ReadPacket()))
print("---***---***---***---***")
s.SendRetransmitRequest()
print(str(s.ReadPacket()))

        # while True:
        #     fmt = 'iii'
        #     rlen = ustruct.calcsize(fmt)
        #     buf = bytearray(rlen)
        #     ustruct.pack_into(fmt, buf, 0, 8080, 0, 1)

        #     sleep(1)
        #     print("Sending %d" % len(buf))
        #     uart.write(buf)
        #     sleep(1)

        #     rxData = bytes()
        #     while uart.any() != 0:
        #         rxData += uart.read(1)
        #         #print("LEFT: %d, total: %d" % (uart.any(),len(rxData)))
        #         time.sleep_ms(1)

        #     unpacked_data = ustruct.unpack('iii', rxData)
        #     print(unpacked_data)
        #     print("Tick")



#
# On PicoW init wifi
#
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

print("Blinking LEDs for a bit")
for count in range(0,100):
    fixedLed.toggle()
    sleep(0.5)
sleep(1)
print("Bye!! - xyz")
sleep(5)
machine.reset()
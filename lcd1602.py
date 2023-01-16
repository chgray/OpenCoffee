import machine
import time


LCD_CLEARDISPLAY = 0x01
LCD_RETURNHOME = 0x02
LCD_ENTRYMODESET = 0x04
LCD_DISPLAYCONTROL = 0x08
LCD_CURSORSHIFT = 0x10
LCD_FUNCTIONSET = 0x20
LCD_SETCGRAMADDR = 0x40
LCD_SETDDRAMADDR = 0x80
LCD_ENTRYRIGHT = 0x00
LCD_ENTRYLEFT = 0x02
LCD_ENTRYSHIFTINCREMENT = 0x01
LCD_ENTRYSHIFTDECREMENT = 0x00
LCD_DISPLAYON = 0x04
LCD_DISPLAYOFF = 0x00
LCD_CURSORON = 0x02
LCD_CURSOROFF = 0x00
LCD_BLINKON = 0x01
LCD_BLINKOFF = 0x00
LCD_DISPLAYMOVE = 0x08
LCD_CURSORMOVE = 0x00
LCD_MOVERIGHT = 0x04
LCD_MOVELEFT = 0x00
LCD_8BITMODE = 0x10
LCD_4BITMODE = 0x00
LCD_2LINE = 0x08
LCD_1LINE = 0x00
LCD_5x8DOTS = 0x00

class LCD():
    def __init__(self, addr=0x27, blen=1):
        sda = machine.Pin(6)
        scl = machine.Pin(7)
        self.bus = machine.I2C(1,sda=sda, scl=scl, freq=400000)
        #print(self.bus.scan())
        self.addr = addr
        self.blen = blen
        self.send_command(LCD_FUNCTIONSET | LCD_8BITMODE | 0x3) # |  0x33) # Must initialize to 8-line mode at first
        time.sleep(0.005)
        self.send_command(LCD_FUNCTIONSET | LCD_8BITMODE | 0x2) #0x32) # Then initialize to 4-line mode
        #time.sleep(0.005)
        self.send_command(LCD_FUNCTIONSET | LCD_2LINE) #0x28 2 Lines & 5*7 dots
        time.sleep(0.005)
        self.send_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON ) # 0xC Enable display without cursor
        time.sleep(0.005)
        self.clear()
        
    def write_word(self, data):
        temp = data
        if self.blen == 1:
            #print("[BLEN_ON]")
            temp |= 0x08
        else:
            #print("[BLEN_OFF]")
            temp &= 0xF7
        #print("   Writing 0x%x to addr:0x%x" % (temp, self.addr))
        self.bus.writeto(self.addr, bytearray([temp]))
    
    def send_command(self, cmd):
        #print("send_command 0x%x" % (cmd))
        
        # Send bit7-4 firstly
        buf = cmd & 0xF0
        buf |= 0x04               # RS = 0, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB               # Make EN = 0
        self.write_word(buf)

        # Send bit3-0 secondly
        buf = (cmd & 0x0F) << 4
        buf |= 0x04               # RS = 0, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB               # Make EN = 0
        self.write_word(buf)
    
    def send_data(self, data):
        # Send bit7-4 firstly
        buf = data & 0xF0
        buf |= 0x05               # RS = 1, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB               # Make EN = 0
        self.write_word(buf)

        # Send bit3-0 secondly
        buf = (data & 0x0F) << 4
        buf |= 0x05               # RS = 1, RW = 0, EN = 1
        self.write_word(buf)
        time.sleep(0.002)
        buf &= 0xFB               # Make EN = 0
        self.write_word(buf)
    
    def clear(self):
        self.send_command(LCD_CLEARDISPLAY) #0x1 Clear Screen
            
    def openlight(self):  # Enable the backlight
        self.bus.writeto(self.addr,bytearray([0x08]))
        #self.bus.close()
    
    def write(self, x, y, str):
        if x < 0:
            x = 0
        if x > 15:
            x = 15
        if y < 0:
            y = 0
        if y > 1:
            y = 1

        # Move cursor
        addr = 0x80 + 0x40 * y + x
        self.send_command(addr)

        for chr in str:
            self.send_data(ord(chr))
    
    def message(self, text):
        print("message: %s"%text)
        for char in text:
            if char == '\n':
                self.send_command(0xC0) # next line
            else:
                self.send_data(ord(char))


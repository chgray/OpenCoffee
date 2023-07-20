import machine
import time
from time import sleep


 # Display Image & text on I2C driven ssd1306 OLED display 
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C
import framebuf
import math
import utime
WIDTH  = 128                                            # oled display width
HEIGHT = 64                                             # oled display height

# Explicit Method
sda=machine.Pin(4)
scl=machine.Pin(5)
i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)
#  print(i2c.scan())
from ssd1306 import SSD1306_I2C
oled = SSD1306_I2C(128, 64, i2c)

# Raspberry Pi logo as 32x32 bytearray
buffer = bytearray(b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00|?\x00\x01\x86@\x80\x01\x01\x80\x80\x01\x11\x88\x80\x01\x05\xa0\x80\x00\x83\xc1\x00\x00C\xe3\x00\x00~\xfc\x00\x00L'\x00\x00\x9c\x11\x00\x00\xbf\xfd\x00\x00\xe1\x87\x00\x01\xc1\x83\x80\x02A\x82@\x02A\x82@\x02\xc1\xc2@\x02\xf6>\xc0\x01\xfc=\x80\x01\x18\x18\x80\x01\x88\x10\x80\x00\x8c!\x00\x00\x87\xf1\x00\x00\x7f\xf6\x00\x008\x1c\x00\x00\x0c \x00\x00\x03\xc0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")

# Load the raspberry pi logo into the framebuffer (the image is 32x32)
fb = framebuf.FrameBuffer(buffer, 32, 32, framebuf.MONO_HLSB)

def blk():
    oled.fill(0)
    oled.show()
    
def horiz(l,t,r,c):  # left, right , top
    n = r-l+1        # Horizontal line
    for i in range(n):
        oled.pixel(l + i, t, c)

def vert(l,t,b,c):   # left, top, bottom
    n = b-t+1        # Vertical line
    for i in range(n):
        oled.pixel(l, t+i,c)

def box(l,t,r,b,c):  # left, top, right, bottom
    horiz(l,t,r,c)   # Hollow rectangle
    horiz(l,b,r,c)
    vert(l,t,b,c)
    vert(r,t,b,c)
    
def ring2(cx,cy,r,c):   # Centre (x,y), radius, colour
    for angle in range(0, 90, 2):  # 0 to 90 degrees in 2s
        y3=int(r*math.sin(math.radians(angle)))
        x3=int(r*math.cos(math.radians(angle)))
        oled.pixel(cx-x3,cy+y3,c)  # 4 quadrants
        oled.pixel(cx-x3,cy-y3,c)
        oled.pixel(cx+x3,cy+y3,c)
        oled.pixel(cx+x3,cy-y3,c)
        
# Clear the oled display in case it has junk on it.
oled.fill(0) # Black

# Blit the image from the framebuffer to the oled display
oled.blit(fb, 96, 0)

# Basic stuff
oled.text("Raspberry Pi",5,5)
oled.text("Pico",5,15)
oled.pixel(10,60,1)
oled.rect(5,32,20,10,1)
oled.fill_rect(40,40,20,10,1)
oled.line(77,45,120,60,1)
oled.rect(75,32,40,10,1)

ring2(50,43,20,1)  # Empty circle             
# Finally update the oled display so the image & text is displayed
oled.show()
utime.sleep(3)

# Bouncing Raspberry
oled.fill(0) # Black
dx = 1
dy = 1
x = 13
y = 10
c = 1  # Colour White
for loop in range(250):
    oled.blit(fb, x, y)
    x = x + dx
    y = y + dy
    if (x == 95) or (x == 1):
        dx = dx * -1
    if (y == 31) or (y == 1):
        dy = dy * -1
    box(4,0,123,63,c)
    oled.show()
oled.fill(0) # Black# MicroPython SSD1306 OLED driver, I2C and SPI interfaces

from micropython import const
import framebuf


# register definitions
SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_DISP = const(0xAE)
SET_MEM_ADDR = const(0x20)
SET_COL_ADDR = const(0x21)
SET_PAGE_ADDR = const(0x22)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)

# Subclassing FrameBuffer provides support for graphics primitives
# http://docs.micropython.org/en/latest/pyboard/library/framebuf.html
class SSD1306(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        for cmd in (
            SET_DISP | 0x00,  # off
            # address setting
            SET_MEM_ADDR,
            0x00,  # horizontal
            # resolution and layout
            SET_DISP_START_LINE | 0x00,
            SET_SEG_REMAP | 0x01,  # column addr 127 mapped to SEG0
            SET_MUX_RATIO,
            self.height - 1,
            SET_COM_OUT_DIR | 0x08,  # scan from COM[N] to COM0
            SET_DISP_OFFSET,
            0x00,
            SET_COM_PIN_CFG,
            0x02 if self.width > 2 * self.height else 0x12,
            # timing and driving scheme
            SET_DISP_CLK_DIV,
            0x80,
            SET_PRECHARGE,
            0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL,
            0x30,  # 0.83*Vcc
            # display
            SET_CONTRAST,
            0xFF,  # maximum
            SET_ENTIRE_ON,  # output follows RAM contents
            SET_NORM_INV,  # not inverted
            # charge pump
            SET_CHARGE_PUMP,
            0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01,
        ):  # on
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP | 0x00)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        x0 = 0
        x1 = self.width - 1
        if self.width == 64:
            # displays with width of 64 pixels are shifted by 32
            x0 += 32
            x1 += 32
        self.write_cmd(SET_COL_ADDR)
        self.write_cmd(x0)
        self.write_cmd(x1)
        self.write_cmd(SET_PAGE_ADDR)
        self.write_cmd(0)
        self.write_cmd(self.pages - 1)
        self.write_data(self.buffer)


class SSD1306_I2C(SSD1306):
    def __init__(self, width, height, i2c, addr=0x3C, external_vcc=False):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b"\x40", None]  # Co=0, D/C#=1
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80  # Co=1, D/C#=0
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)


class SSD1306_SPI(SSD1306):
    def __init__(self, width, height, spi, dc, res, cs, external_vcc=False):
        self.rate = 10 * 1024 * 1024
        dc.init(dc.OUT, value=0)
        res.init(res.OUT, value=0)
        cs.init(cs.OUT, value=1)
        self.spi = spi
        self.dc = dc
        self.res = res
        self.cs = cs
        import time

        self.res(1)
        time.sleep_ms(1)
        self.res(0)
        time.sleep_ms(10)
        self.res(1)
        super().__init__(width, height, external_vcc)

    def write_cmd(self, cmd):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.spi.init(baudrate=self.rate, polarity=0, phase=0)
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(buf)
        self.cs(1)
oled.show()

# Scrolling text on Sine curve
# Modified from a method by Tony DiCola
msg = 'Pico + SSD1306 is Magic!'
f_width  = 8   # Font width in pixels
f_height = 8   # Font Height in pixels
amp = 50   # Amplitude of sin wave
freq = 1    # Screen cycles (360 degrees)  
pos = WIDTH  # X position of the first character in the msg.
msg_len_px = len(msg) * f_width  # Pixel width of the msg.
# Extra wide lookup table - calculate once to speed things up
y_table = [0] * (WIDTH+f_width) # 1 character extra
for i in range(len(y_table)):
    p = i / (WIDTH-1)  # Compute current position
    # Create lookup table of  y co-ordinates 
    y_table[i] = int(((amp/2.0) * math.sin(2.0*math.pi*freq*p)) + (amp/2.0))
    
# Main loop:
finish = utime.ticks_ms() + 40000 # 40 seconds
while utime.ticks_ms() < finish:
    # Start again if msg finished
    pos -= 1
    if pos <= -msg_len_px:
        pos = WIDTH
    # Go through each character in the msg.
    blk()
    for i in range(len(msg)):
        char = msg[i]
        char_x = pos + (i * f_width)  # Character's X position on the screen.
        if -f_width <= char_x < WIDTH:
            # If character is visible, draw it.
            oled.text(char, char_x + 5, y_table[char_x + f_width], 1)
    oled.show()
    utime.sleep(0.08)
    
# Tidy up
blk() 












led = machine.Pin("LED", machine.Pin.OUT)
led.off()
led.on()

while True:
    led.toggle()
    sleep(1)
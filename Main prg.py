import RPi.GPIO as GPIO
import time
import random
import smbus
from PCF8574 import PCF8574_GPIO
from Adafruit_LCD1602 import Adafruit_CharLCD
from time import sleep, strftime
from datetime import datetime

PCF8574_address = 0x27 #I2C address of PCF8574 chip
PCF8574A_address = 0x3F #I2C address of the PCF8574A chip
# Create PCF8574 GPIO adapter to the LCD screen from the PCF mocdule
try:
    mcp = PCF8574_GPIO(PCF8574_address)
except:
    try:
        mcp = PCF8574_GPIO(PCF8574A_address)
    except:
        print('I2C Address Error !')
        long curTime = time.time()
        while (time.time()-curTime) < 0.75:
            timeDiff = time() - curTime
            if (timeDiff<0.15 && timeDiff>0) || (timeDiff>0.3 && timeDiff<0.45) ||
                (timeDiff>0.6 && timeDiff<0.75):
                LEDEvent(PowerLedPin, GPIO.HIGH)
            else:
                LEDEvent(PowerLedPin, GPIO.LOW)
        LEDEvent(PowerLedPin, GPIO.LOW)
        exit(1)
# Create the LCD object, using the Adafruit_CharLCD class by passing in MCP GPIO adapter
lcd = Adafruit_CharLCD(pin_rs = 0, pin_e = 2, pins_db = {4,5,6,7}, GPIO = mcp)
# note: above line of code might fail due to having "{}" in place of "[]" in the list                

# IO pin addresses
PowerLedPin = 11 #define the ledPin
redButtonPin = 13 #define the buttonPin
PlayPauseButtonPin = 15 #defines the Play/Pause button pin
ServoSig = 16 #Servo Motor Signal line

LedBarSH = 12 #Serial shift clock
LedBarST = 40 #Parallel update output
LedBarDS = 38 #Serial data input
address = 0x48 # default address of PCF8591
bus = smbus.SMBus(1) # creates an SMBus instance (object)
command = 0x40 # command

# Defines the data bit that gets transmitted first
LSBFIRST = 1
MSBFIRST = 2

const digitalValPerLEDbar = 256/8  # digital value per increment of LED bar light
trigPin = 29
echoPin = 31
MAX_DISTANCE = 220 # defines the maximum measured distance (cm)
timeOut = MAX_DISTANCE*60 # calculates the time out

discoLightsPin = 33

OFFSET_DUTY = 0.5 # Defines the pulse offset of servo
SERVO_MIN_DUTY = 2.5 + OFFSET_DUTY # Defines the pulse duty cycle for minumum angle of servo
SERVO_MAX_DUTY = 12.5 + OFFSET_DUTY # Defines the pulse duty cycle for maximum angle of servo


# Note: 1st pin is the most significant bit I think


pins = {'RGBLedB':18, 'RGBLedG': 25, 'RGBLedR' : 32} # defines the PWM pin for Blue, red and green light


# Necessary global variables
RedLEDstate = False
PlayPauseState = False 

def setup():
    GPIO.setmode(GPIO.BOARD)  # Numbers the GPIO Pins by physical location on the board
    GPIO.setup(PowerLedPin, GPIO.OUT)
    GPIO.output(PowerLedPin, LOW)
    GPIO.setup(redButtonPin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

    global pinR, PinG, pinB
    for i in pins:
        GPIO.setup(pins[i], GPIO.OUT)
        GPIO.setmode(pins[i], GPIO.HIGH)
    # sets the pins to PWM output at freq. of 2000 Hz
    pinR = GPIO.PWM(pins['RGBLedR'], 2000)
    pinG = GPIO.PWM(pins['RGBLedG'], 2000)
    pinB = GPIO.PWM(pins['RGBLedB'], 2000)
    # sets the LEDs off initially
    pinR.start(100)
    pinG.start(100)
    pinB.start(100)
    GPIO.setup(LedBarDS, GPIO.OUT)
    GPIO.setup(LedBarST, GPIO.OUT)
    GPIO.setup(LedBarSH, GPIO.OUT)
    GPIO.setup(trigPin, GPIO.OUT)
    GPIO.setup(echoPin, GPIO.IN)
    GPIO.setup(discoLightsPin, GPIO.OUT)
    GPIO.setup(ServoSig, GPIO.OUT)
    GPIO.output(ServoSig, GPIO.LOW)
    p = GPIO.PWM(ServoSig,50) # Set frequency of PWM to 50Hz
    p.start(0) # Duty Cycle = 0
    mcp.output(3, GPIO.HIGH) # Turns on the LCD backlight. LCD backlight pin is number 3
    lcd.begin(16,2) # sets the number of columns and lines to display

def LCDWriteText(text, cursorPosX,cursorPosY):
    lcd.setCursor(cursorPosX, cursorPosY) # sets cursor position
    lcd.message(text) # writes text onto the LCD


def servoWrite(angle): # make the servo rotate the specific angle(0-180 degrees)
    if angle<0:
        angle = 0
    elif angle>180:
        angle = 180
    # Gives the angle in terms of the duty cycle between 2.5 ms and 12.5ms
    angleInDutyCycle = (SERVO_MAX_DUTY-SERVO_MIN_DUTY)*(angle-0)/(180-0) + SERVO_MIN_DUTY
    p.ChangeDutyCycle(angleInDutyCycle)


# Note: The code for determining the distance can be run in a new seperate thread    
# Obtains the pulse time of the pin
def PulseIn():
    t0 = time.time()
    while(GPIO.input(echoPin) != GPIO.HIGH):
        if((time.time()-t0) > timeOut*0.000001):
            return 0
    t0 = time.time()
    while(GPIO.input(echoPin) == GPIO.HIGH):
        if((time.time()-t0) > timeOut*0.000001):
            return 0
    pulseTime = (time.time()-t0)*0.000001
    return pulseTime

# Get the measurement result of ultrasonic module.
# returns a value between 0 and 255
def getSonar():
    GPIO.output(trigPin, GPIO.HIGH) # make trigPin send 10 us high level
    time.sleep(0.00001)
    GPIO.output(trigPin, GPIO.LOW)
    pingTime = PulseIn() # read pulse time of echo Pin
    distance = pingTime*340.0/2.0/10000.0 # speed of sound regarded as 340 m/s and distance in cm
    return distance

#Later come up with a function(algorithm) that tracks the hand movement and gives the correct output within (0-255)
# to LEDbar function    
    

# turn on the disco lights. NOTE: timing will be added in the main routine
def DiscoLightsOnOff(channel, TurnOnOrOff):
    GPIO.output(channel, TurnOnOrOff)
        

# shiftOut function, shifts the serial byte into shift register
def shiftOut(dPin, cPin, order, val):
    for i in range(0,8):
        GPIO.output(cPin, LOW)
        if(order == LSBFIRST):
            GPIO.output(dPin, (0x01&(val>>i)== 0x01) and GPIO.HIGH or GPIO.LOW)
        elif order == MSBFIRST:
            GPIO.output(dPin, (0x80&(val<<i) == 0x80) and GPIO.HIGH or GPIO.LOW)
        GPIO.output(cPin, GPIO.HIGH)

# turns on the required number of LEDs based on digital input parameter    
def NumLEDbars(digitVal):
    numLEDs = int((digitVal+1+16)/digitValPerLEDbar)
    binNum
    for i in range(0,8):
        if(i < numLEDs):
            binNum += '1'
        else:
            binNum+= '0'
    binNum += '0b'
    byteInHex = hex(int(binNum, 2)) # may cause problems due to '0b' in binary number
    GPIO.output(LedBarST, GPIO.LOW)
    shiftOut(LedBarDS, LedBarSH, LSBFIRST, byteInHex)
    GPIO.output(LedBarST, GPIO.HIGH)
    GPIO.setup(trigPin, GPIO.OUT)
    GPIO.setup(echoPin, GPIO.IN)
    GPIO.setup(discoLightsPin, GPIO.OUT)
    

# Event function that turns ON the Red LED
def LEDEvent(channel, activate): # receives arg on whether to turn ON or OFF LED
    GPIO.output(channel,activate)
    RedLEDstate = not RedLEDstate

# Event function that changes color at play/pause
def LEDPlayPauseEvent():
    PlayPauseState = not PlayPauseState
    if not PlayPauseState:
        setColor(0,100,100)
    else:
        setColor(100,0,100)

# reads the Analogue values from channels 0 and 1, "value" is the digital quantitiy between 0-255
def analogueRead(channel):
    value = bus.read_byte_data(address, command + channel)
    return value


# Event function that changes colour of RGB light
def setColor(r_val, g_val, b_val):
    pinR.ChangeDutyCycle(r_val)
    pinG.ChangeDutyCycle(g_val)
    pinB.ChangeDutyCycle(b_val)

# Function that randomly chooses color of RGB LED during a song
def RandColorChanger():
    r = random.randint(0,100)
    g = random.randint(0,100)
    b = random.randint(0,100)

# Button event function that returns value based on state of play/pause button
def playPauseButtonEvent(ButtonChannel):
    if GPIO.add_event_detect(ButtonChannel, GPIO.FALLING, callback = LEDPlayPauseEvent, bouncetime = 300): 
        if PlayPauseState :
            return 1  #button pressed, LED OGREEN
        return 0 # button pressed. LED RED
    return -1   # button not pressed


# Button event function, returns value based on state of power button
def ButtonEventDetect(ButtonChannel):
    if GPIO.add_event_detect(ButtonChannel, GPIO.FALLING, callback = LEDEvent(PowerLedPin, not RedLEDstate), bouncetime = 300): 
        if((not RedLEDstate) == True):
            return 1  #button pressed, LED ON
        return 0 # button pressed. LED OFF
    return -1   # button not pressed 

def destroy():
    GPIO.output(PowerLedPin, LOW)
    pinR.stop()
    pinG.stop()
    pinB.stop()
    for i in pins:
        GPIO.output(pin[i], HIGH)
    bus.close()
    p.stop()
    
    GPIO.cleanup()



if __name__ == '__main__':
    setup()

    # rest of the code
    # sequence of code

    destroy()

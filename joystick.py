import math
import time
import array as arr
import serial
from pyglet.gl import *


################################# Define functions #################################
# Initialize joystick
def Joystick_Init():
    global joystick
    # Get joysticks and open
    joysticks = pyglet.input.get_joysticks()
    # Ensure device(s) found
    assert joysticks, "No joystick device is connected"
    # Select joystick
    joystick = joysticks[4]
    count = 0
    for i in joysticks:
        print(count, i.device)
        count += 1

    print(joystick.device)
    for x in joystick.device.get_controls():
        print(x)
    # Open selected joystick
    joystick.open()


# XBee TX/RX
def processInputs():
    # Get joystick inputs

    x = joystick.x  # L/R
    y = joystick.y  # U/D
    z = joystick.rz  # Throttle?
    rz = joystick.z  # ?
    print('x: {}\ny: {}\nz: {}'.format(x, y, z))
    print()

    # Convert joystick values to int
    xConv = convertToInt(x)
    yConv = convertToInt(y)
    zConv = convertToInt(z)
    print('xConv: {}\nyConv: {}\nzConv: {}'.format(xConv, yConv, zConv))
    print()

    elements = [xConv, yConv, zConv, 0, 0, 0, 0, 0]
    data = bytearray(elements)
    vect = list(data)
    print(vect)

    # Write inputs to Remote Xbee
    arduinoData.write(vect)

    # # Read back data from Remote XBee
    # arduinoString = arduinoData.readline()                # (causes delay)
    # arduinoString = str(arduinoString, encoding='utf-8')
    # print("from Remote: ", arduinoString)
    # time.sleep(1)


# Convert controller data to int for processing
def convertToInt(value):
    value = int(math.ceil(value * 10) + 10)
    return value

def periodicFunc(dt):       # can control write data w/ specified periodicity
    print("triggered")

def checkSerial(dt):       # serial test
    n = arduinoData.in_waiting
    if n > 0:
        databytes = arduinoData.read(n)
        arduinoString = str(databytes, encoding='utf-8')
        print("n: ", n)
        print("checkSerial: ", arduinoString)

################################# End define functions #################################


# Initialize joystick and print controls
Joystick_Init()

# Connect to Xbee
arduinoData = serial.Serial('COM7', 9600, serial.EIGHTBITS)


@joystick.event
def on_joyaxis_motion(stick, axis, value):
    print("event")
    processInputs()


pyglet.clock.schedule_interval(checkSerial, 0.1)

# Run app (must be last func)
pyglet.app.run()

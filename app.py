#!/usr/bin/env python

DEBUG = 0
async_mode = None

if async_mode is None:
    try:
        import eventlet
        async_mode = 'eventlet'
    except ImportError:
        pass

    if async_mode is None:
        try:
            from gevent import monkey
            async_mode = 'gevent'
        except ImportError:
            pass

    if async_mode is None:
        async_mode = 'threading'

    print('async_mode is ' + async_mode)

# monkey patching is necessary because this application uses a background
# thread
if async_mode == 'eventlet':
    import eventlet
    eventlet.monkey_patch()
elif async_mode == 'gevent':
    from gevent import monkey
    monkey.patch_all()

import time
import spidev
from threading import Thread
from flask import Flask, render_template, session, request
from flask_socketio import SocketIO, emit, send, disconnect
from flask.ext.assets import Environment, Bundle

from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor, Adafruit_StepperMotor
import atexit
import RPi.GPIO as GPIO

import Adafruit_MCP9808.MCP9808 as MCP9808

### GYRO SENSOR SETUP
from Adafruit_BNO055 import BNO055

# Create and configure the BNO sensor connection.  Make sure only ONE of the
# below 'bno = ...' lines is uncommented:
# Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

# How often to update the BNO sensor data (in hertz).
BNO_UPDATE_FREQUENCY_HZ = 10

# Name of the file to store calibration data when the save/load calibration
# button is pressed.  Calibration data is stored in JSON format.
CALIBRATION_FILE = 'calibration.json'

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')
                
### PUSHOVER SETUP
import httplib, urllib
def sendMessage():
    conn = httplib.HTTPSConnection("api.pushover.net:443")
    conn.request("POST", "/1/messages.json",
      urllib.urlencode({
        "token": "aqXxR8tiaKy8Q9hLQKCbwwSysnrAhU",
        "user": "ufqBkUqEwtq8XVbx8q2cjKqnADvNwh",
        "message": "hello world",
      }), { "Content-type": "application/x-www-form-urlencoded" })
    print("sent message")
    conn.getresponse()

import json

### TEMPERATURE SENSOR

# Define a function to convert celsius to fahrenheit.
def c_to_f(c):
    return c * 9.0 / 5.0 + 32.0

sensor = MCP9808.MCP9808()
# Optionally you can override the address and/or bus number:
#sensor = MCP9808.MCP9808(address=0x20, busnum=2)

# Initialize communication with the sensor.
sensor.begin()

### MCP3008 A/D CHIP
spi = spidev.SpiDev()
spi.open(0,0)

# read SPI data from MCP3008 chip, 8 possible adc's (0 thru 7)
def readadc(adcnum):
    if ((adcnum > 7) or (adcnum < 0)):
            return -1
    r = spi.xfer2([1,(8+adcnum)<<4,0])
    adcout = ((r[1]&3) << 8) + r[2]
    return adcout

# adc pins
adc_0 = 0;
adc_1 = 1;
adc_2 = 2;

last_read = 0       # this keeps track of the last analog value
tolerance = 5       # to keep from being jittery we may only change value when the pot has moved more than 5 'counts'


app = Flask(__name__)
assets = Environment(app)
css = Bundle('styles.scss',
            filters='scss', output='styles.css')
assets.register('css_all', css)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)
thread = None
touchSensorThread = None
tempSensorThread = None
gyroSensorThread = None
analogSensorThread = None
buttonsThread = None

# create a default object, no changes to I2C address or frequency
mh = Adafruit_MotorHAT(addr = 0x60)

# recommended for auto-disabling motors on shutdown!
def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
 
atexit.register(turnOffMotors)

motorSpeed = 30
myStepper = mh.getStepper(200, 1)       # 200 steps/rev, motor port #1



def moveMotorBACK(steps):
    myStepper.step(steps, Adafruit_MotorHAT.BACKWARD, Adafruit_MotorHAT.DOUBLE)

def moveMotorFWD(steps):
    myStepper.step(steps, Adafruit_MotorHAT.FORWARD,  Adafruit_MotorHAT.DOUBLE)


def background_thread():
    """Example of how to send server generated events to clients."""
    count = 0
    
    while True:
        time.sleep(3)
        count += 1
        print("background awake")
        socketio.emit('my response',
                      {'data': 'Server generated event', 'count': count},
                      namespace='/test')
        socketio.emit('my response',
                      {'data': 'Server generated event', 'count': count},
                      namespace='/remote')

#def buttonsThreadWorker():
#    GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#
#    while True:
#        input_state = GPIO.input(12)
#        if input_state == False:
#            print('Button Pressed')
#            time.sleep(0.3)


def gyroSensorThreadWorker():
    while True:
        # Read the Euler angles for heading, roll, pitch (all in degrees).
        heading, roll, pitch = bno.read_euler()
        # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
        sys, gyro, accel, mag = bno.get_calibration_status()
        # Print everything out.
        #print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
        #      heading, roll, pitch, sys, gyro, accel, mag))
        # Other values you can optionally read:
        # Orientation as a quaternion:
        #x,y,z,w = bno.read_quaterion()
        # Sensor temperature in degrees Celsius:
        #temp_c = bno.read_temp()
        # Magnetometer data (in micro-Teslas):
        #x,y,z = bno.read_magnetometer()
        # Gyroscope data (in degrees per second):
        #x,y,z = bno.read_gyroscope()
        # Accelerometer data (in meters per second squared):
        #x,y,z = bno.read_accelerometer()
        # Linear acceleration data (i.e. acceleration from movement, not gravity--
        # returned in meters per second squared):
        #x,y,z = bno.read_linear_acceleration()
        # Gravity acceleration data (i.e. acceleration just from gravity--returned
        # in meters per second squared):
        #x,y,z = bno.read_gravity()
        # Sleep for a second until the next reading.
        time.sleep(0.1)   
        socketio.emit('gyro',
                      {'heading': heading, 'roll': roll, 'pitch': pitch},
                      namespace='/test')

def analogSensorThreadWorker():
    while True:
        adc0_value = readadc(adc_0)
        adc1_value = readadc(adc_1)
        adc2_value = readadc(adc_2)

        adc0ranged = translate(adc0_value, 0, 1024, 0, 175)
        adc1ranged = translate(adc1_value, 0, 1024, 0, 175)
        adc2ranged = adc2_value * 5

        socketio.emit('joystick',
                      {'Xaxis': adc0ranged, 'Yaxis': adc1ranged},
                      namespace='/test')

        socketio.emit('sonarrange',
                      {'distance': adc2ranged},
                      namespace='/test')

        time.sleep(0.1)

        if DEBUG:
            print "adc0_value", adc0_value
            print "adc1_value", adc1_value
            print "adc2_value", adc2_value
            time.sleep(0.3)

def touchSensorThreadWorker():
    """Whatever"""
    
    while True:
        GPIO.setmode(GPIO.BCM)
         
        #set the GPIO input pins
        pad3 = 24
        pad4 = 23

        GPIO.setup(pad3, GPIO.IN)
        GPIO.setup(pad4, GPIO.IN)

        pad3alreadyPressed = False
        pad4alreadyPressed = False

        pad3pressed = not GPIO.input(pad3)
        pad4pressed = not GPIO.input(pad4)

        if pad3pressed and not pad3alreadyPressed:
            socketio.emit('pads',
                      {'data': 'pad0'},
                      namespace='/test')
        pad3alreadyPressed = pad3pressed
        

        if pad4pressed and not pad4alreadyPressed:
            socketio.emit('pads',
                      {'data': 'pad1'},
                      namespace='/test')
        pad4alreadyPressed = pad4pressed
        time.sleep(0.2)

def tempSensorThreadWorker():
    """Whatever 2"""
    
    while True:
        temp = sensor.readTempC()
        socketio.emit('temp',
                      {'data': '{0:0.1F} C / {1:0.1F} F'.format(temp, c_to_f(temp))},
                      namespace='/test')
        time.sleep(0.6)

def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

@app.route('/')
def index():
    global thread
    if thread is None:
        thread = Thread(target=background_thread)
        thread.daemon = True
        thread.start()

    global touchSensorThread
    if touchSensorThread is None:
        touchSensorThread = Thread(target=touchSensorThreadWorker)
        touchSensorThread.daemon = True
        touchSensorThread.start()

    global tempSensorThread
    if tempSensorThread is None:
        tempSensorThread = Thread(target=tempSensorThreadWorker)
        tempSensorThread.daemon = True
        tempSensorThread.start()

    global gyroSensorThread
    if gyroSensorThread is None:
        gyroSensorThread = Thread(target=gyroSensorThreadWorker)
        gyroSensorThread.daemon = True
        gyroSensorThread.start()

    global analogSensorThread
    if analogSensorThread is None:
        analogSensorThread = Thread(target=analogSensorThreadWorker)
        analogSensorThread.daemon = True
        analogSensorThread.start()

#    global buttonsThread
#    if buttonsThread is None:
#        buttonsThread = Thread(target=buttonsThreadWorker)
#        buttonsThread.daemon = True
#        buttonsThread.start()

    return render_template('index.html')

@socketio.on('my event', namespace='/test')
def test_message(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my response',
         {'data': message['data'], 'count': session['receive_count']})
    speed = message['data']
    print speed

@socketio.on('aaa', namespace='/remote')
def response(speed):
    print('hi')

@socketio.on('speed', namespace='/test')
def response(speed):
    motorSpeed = speed['data']
    myStepper.setSpeed(motorSpeed)                  # 30 RPM
    print motorSpeed

@socketio.on('motorMove', namespace='/test')
def response(motorMove):
    motorSpeed = 30
    myStepper.setSpeed(motorSpeed)                  # 30 RPM
    direction = motorMove['data']
    if direction == 'advance':
        moveMotorFWD(350)
        print('fwd')
        turnOffMotors()
    else:
        moveMotorBACK(350)
        print('back')
        turnOffMotors()

@socketio.on('connect', namespace='/test')
def test_connect():
    emit('my response', {'data': 'Connected', 'count': 0})


@socketio.on('disconnect', namespace='/test')
def test_disconnect():
    print('Client disconnected', request.sid)

#@app.before_first_request

if __name__ == '__main__':
    socketio.run(app, debug=True, host='0.0.0.0')


"""
Simple script for take off and control with arrow keys
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import os
from threading import Thread, Event, Timer
import time
import signal
import sys

def TimerReset(*args, **kwargs):
    """ Global function for Timer """
    return _TimerReset(*args, **kwargs)

class _TimerReset(Thread):
    """Call a function after a specified number of seconds:

    t = TimerReset(30.0, f, args=[], kwargs={})
    t.start()
    t.cancel() # stop the timer's action if it's still waiting
    """

    def __init__(self, interval, function, args=[], kwargs={}):
        Thread.__init__(self)
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.finished = Event()
        self.resetted = True

    def cancel(self):
        """Stop the timer if it hasn't finished yet"""
        self.finished.set()

    def run(self):
      while not self.finished.isSet():
          self.resetted = True
          while self.resetted:
              self.resetted = False
              self.finished.wait(self.interval)

          if not self.finished.isSet():
              self.function(*self.args, **self.kwargs)
      print "Time: %s - timer finished!" % time.asctime()

    def reset(self, interval=None):
        """ Reset the timer """

        if interval:
            self.interval = interval

        self.resetted = True
        self.finished.set()
        self.finished.clear()

global vehicle

#-- Setup the commanded flying speed
global gnd_speed  # [m/s]

global commandTimer

def commandTimerTimeout():
    print("Not receive any command...")
    if vehicle.armed:
        vehicle.mode = VehicleMode("LAND")
        print("Is Armed:% s" % vehicle.armed)

commandTimer = TimerReset(10, commandTimerTimeout)

def signal_handler(signal, frame):
        print('You pressed Ctrl+C!')
        commandTimer.cancel()
        sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#-- Define arm and takeoff
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    vehicle.flush()

    print(vehicle.armed)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("Waiting for arming... Current Mode:% s" % vehicle.mode.name)
        print("Is Armed:% s" % vehicle.armed)
        vehicle.armed = True
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    # while True:
    #     print "Altitude: ", vehicle.location.global_relative_frame.alt
    #     #Break and return from function just below target altitude.
    #     if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
    #         print "Reached target altitude"
    #         break
    #     time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

#-- Key event function
def controlDrone(vehicle, event):
    gnd_speed = 0.5

    if event == 'TAKEOFF':
        arm_and_takeoff(gnd_speed)
    elif event == 'FORWARD':
        if vehicle.armed:
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        else:
            print("Vehicle is not armed.")
    elif event == 'BACKWARD':
        if vehicle.armed:
            set_velocity_body(vehicle, -gnd_speed, 0, 0)
    elif event == 'LEFT':
        if vehicle.armed:
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif event == 'RIGHT':
        if vehicle.armed:
            set_velocity_body(vehicle, 0, gnd_speed, 0)
    elif event == 'LAND':
        if vehicle.armed:
            vehicle.mode = VehicleMode("LAND")
            print("Is Armed:% s" % vehicle.armed)
            commandTimer.cancel()
    elif event == 'LANDANDSHUTDOWN':
        if not vehicle.armed:
            print("Is Armed:% s" % vehicle.armed)
            os.system("shutdown -h now")
        else:
            print("Land it first!")
    else:
        print("Command not found.")

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc==0:
        print("Connected with result code " + str(rc))
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("/controlling-drone")

        # #-- Connect to the vehicle
        print('Connecting...')
        global vehicle
        # vehicle = connect('tcp:192.168.2.1:5760', wait_ready=False)
        vehicle = connect('/dev/ttyACM0', wait_ready=False)

        commandTimer.start()    

        # Callback definition for mode observer
        def mode_callback(self, attr_name, observer):
            print self.mode
            print("Is Armed:% s" % vehicle.armed)

        # Add observer callback for attribute `mode`
        vehicle.add_attribute_listener('mode', mode_callback)
    else:
        print("Bad connection Returned code=",rc)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    commandTimer.reset(10)    
    controlDrone(vehicle, str(msg.payload))

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("bbff39d0d3066758ffe55666762b3c8b150295b848cb6c871b79f2fff36c79fb",
                       "50acea3098359517297e08040dc6bfc371d044190be6527c1ac29e078cbe8313")

client.connect("localhost", 1883, 60)
# client.connect("192.168.2.148", 1883, 60)

client.loop_forever()

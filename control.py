"""
Simple script for take off and control with arrow keys
"""
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import paho.mqtt.client as mqtt
import os
from threading import Thread, Event, Timer
import time
import signal
import sys
import math
import redis

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

global redisClient

#-- Setup the commanded flying speed
global gnd_speed  # [m/s]

global commandTimer

def commandTimerTimeout():
    print("Not receive any command...")
    if vehicle.armed:
        vehicle.mode = VehicleMode("LAND")
        print("Is Armed:% s" % vehicle.armed)
        commandTimer.cancel()

commandTimer = TimerReset(60, commandTimerTimeout)

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

"""
Convenience functions for sending immediate/guided mode commands to control the Copter.
The set of commands demonstrated here include:
* MAV_CMD_CONDITION_YAW - set direction of the front of the Copter (latitude, longitude)
* MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed (latitude, longitude, altitude)
* MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.
The full set of available commands are listed here:
http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/
"""

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    
    if vehicle.mode == "BRAKE":
        vehicle.mode = VehicleMode("GUIDED")
        
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

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

    if vehicle.mode == "BRAKE":
        vehicle.mode = VehicleMode("GUIDED")

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

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.

    For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame      
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def get_distance_metres(aLocation1, aLocation2):
    dlat        = aLocation2.lat - aLocation1.lat
    dlong       = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def gotoGPS(vehicle, location):
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = get_distance_metres(currentLocation, location)
    goto_position_target_global_int(location)
    vehicle.flush()
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, location)
        if remainingDistance<=targetDistance*0.05: #Just below target, in case of undershoot.
            print "Reached target"
            break
        time.sleep(2)

#-- Key event function
def controlDrone(vehicle, event):
    if redisClient.get("GSPEED") == None:
        gnd_speed = 0.5
    else:
        gnd_speed = float(redisClient.get("GSPEED"))

    vehicle.airspeed = gnd_speed

    vehicle.groundspeed = gnd_speed

    if event == 'TAKEOFF':
        arm_and_takeoff(3)
    elif event == 'FORWARD':
        if vehicle.armed:
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        else:
            print("Vehicle is not armed.")
    elif event == 'BACKWARD':
        if vehicle.armed:
            set_velocity_body(vehicle, -gnd_speed, 0, 0)
        else:
            print("Vehicle is not armed.")
    elif event == 'LEFT':
        if vehicle.armed:
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        else:
            print("Vehicle is not armed.")
    elif event == 'RIGHT':
        if vehicle.armed:
            set_velocity_body(vehicle, 0, gnd_speed, 0)
        else:
            print("Vehicle is not armed.")
    elif event == 'LAND':
        if vehicle.armed:
            vehicle.mode = VehicleMode("LAND")
            print("Is Armed:% s" % vehicle.armed)
            commandTimer.cancel()
            redisClient.set("GSPEED", 0.5)
        else:
            print("Vehicle is not armed.")
    elif event.startswith("YAW"):
        if vehicle.armed:
            yaw = event.split(":")
            condition_yaw(int(yaw[1]), relative=True)
        else:
            print("Vehicle is not armed.")
    elif event == 'BRAKE':
        if vehicle.armed:
            vehicle.mode = VehicleMode("BRAKE")
        else:
            print("Vehicle is not armed.")
    elif event.startswith("ALT"):
        if vehicle.armed:
            altMeter = event.split(":")
            newLoc = LocationGlobalRelative (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, int(altMeter[1]))
            gotoGPS(vehicle, newLoc)
        else:
            print("Vehicle is not armed.")
    elif event == "INCR":
        if vehicle.armed:
            if redisClient.get("GSPEED") != None:
                if float(redisClient.get("GSPEED")) < 5.0:
                    redisClient.incrbyfloat("GSPEED", 0.25)
            else:
                redisClient.incrbyfloat("GSPEED", 0.25)
        else:
            print("Vehicle is not armed.")
    elif event == "DECR":
        if vehicle.armed:
            if redisClient.get("GSPEED") != None:
                if float(redisClient.get("GSPEED")) > 0.5:
                    redisClient.incrbyfloat("GSPEED", -0.25)
        else:
            print("Vehicle is not armed.")
    elif event == 'LANDANDSHUTDOWN':
        if not vehicle.armed:
            print("Is Armed:% s" % vehicle.armed)
            redisClient.set("GSPEED", 0.5)
            commandTimer.cancel()
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
        # vehicle = connect('localhost:14551', wait_ready=False)
        vehicle = connect('/dev/ttyACM0', wait_ready=False)

        global redisClient
        redisClient = redis.StrictRedis(host='localhost', port=6379, db=0)

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
    commandTimer.reset()    
    controlDrone(vehicle, str(msg.payload))

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("bbff39d0d3066758ffe55666762b3c8b150295b848cb6c871b79f2fff36c79fb",
                       "50acea3098359517297e08040dc6bfc371d044190be6527c1ac29e078cbe8313")

client.connect("localhost", 1883, 60)
# client.connect("192.168.2.148", 1883, 60)

client.loop_forever()

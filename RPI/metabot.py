# -*- coding: utf-8 -*-
"""Module metabot.py

Description :
    This module controls the Raspberry Pi at the centre of the Metabot.
    It connects to:
        -  a controller attached to a remote PC and connected over a TCP
            connection using JSON
        -  an Arduino connected over a serial connection
        -  a local user interface, normally exposed over SSH to a remote
            terminal.

Design :
    metabot.py uses a combination of Pykka Actors and separate threads to
    service its different work sources.
        -  main() creates the different actors and threads and connects them
            together before going into a loop waiting
            for user input at the terminal.
        -  The Controller class runs as a separate thread.  It waits for IP
            messages from the remote PC / Joystick
            controller.  It then parses these messages into a common format
            and sends the information to the ControlLoop Actor.
        -  The ControlLoop Actor processes work from incoming work sources.
            At the moment that is just the Controller class, but in the future
            could be other sensors attached the RPi, timers running in other
            threads, or other work sources.  Based on the inputs it decides
            what actions to take and then transmits them to the Arduino.
        -  The Arduino Actor communicates over a serial connection to the
            Arduino.  It can be invoked using either a Pykka tell method,
            which is a one way communication, or an ask method that will wait
            for a reply from the Arduino and pass that back to the caller.

    The role of Pykka in the code below is not at all obvious.  It only
    shows itself in the inherited classes of the two actors and in the
    initialization and termination calls in main().  However, under the
    covers it is doing quite a bit.
        -  What looks like a simple call to a method in a Pykka actor is
            actually a call to a Pykka proxy instead.
        -  This serializes the parameters to the call, turns them into a
            message and posts them to the inbox of the actor thread.
        -  At some point later, that message is retrieved on the actor thread,
            the parameters extracted from the message and the target method is
            called.
        -  Likewise, the return value from the target method is serialized,
            turned into a message and sent back to the thread of the calling
            code.
        -  If the original call looks at the return parameters, then these are
            turned into a Pykka future, which is picked-up from the value in
            the returned message.
    The net is that we get a nice message passing co-processing scheme all
    done under the covers.

"""

import sys
import threading
import socket
import struct
import json
import getopt
import logging
import os
import serial
import shlex
import subprocess
import pykka
import pygame
import time
import RPi.GPIO as GPIO

from pygame.locals import *

# Set the display to fb1 - i.e. the TFT
os.environ["SDL_FBDEV"] = "/dev/fb1"
# Remove mouse
os.environ["SDL_NOMOUSE"] = "1"
# initialise pygame module
pygame.init()
# set up the window
screen = pygame.display.set_mode((240, 320), 0, 32)

DEFAULT_HOST = "10.5.5.1"
logger = logging.getLogger()
MODES = {"j": "Joystick",
         "r": "Reverse Joystick",
         "s": "Speed test",
         "l": "Line follower",
         "3": "Three point turn",
         "b": "Bowling",
         "p": "Proximity Alert",
         "c": "Config",
         "t": "Test",
         "q": "Quit",
         "x": "Stop",
         ":": "Arduino Cmd",
         "?": "Help"}

# dictionary of images to display when entering modes
IMAGES = {"j": "joystick.png",
          "r": "reverse.png",
          "s": "speed.png",
          "l": "line_follow.png",
          "3": "3_pt_turn.png",
          "b": "bowling.png",
          "p": "proximity.png",
          "c": "config.png",
          "t": "test.png",
          }


class TraceConsumer(threading.Thread):
    def __init__(self):
        super(TraceConsumer, self).__init__()
        self.ser = serial.Serial(port="/dev/ttyACM0",
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
        self.ser.open()
        self.serial_log_file = open("seriallog.txt", 'w', 0)

    def run(self):
        line = None
        while True:
            try:
                line = self.ser.readline()
            except serial.SerialException:
                print "Serial exception hit, ignoring"
            self.serial_log_file.write(line)


class Controller(threading.Thread):
    def __init__(self, mode):
        super(Controller, self).__init__()
        self._mode = mode

    def set_mode(self, mode):
        self._mode = mode

    @staticmethod
    def deadzone(val, cutoff):
        if abs(val) < cutoff:
            return 0
        elif val > 0:
            return (val - cutoff) / (1 - cutoff)
        else:
            return (val + cutoff) / (1 - cutoff)


class RemoteController(Controller):
    """Interface to a remote controller, using JSON over an IP connection"""

    def __init__(self, ipaddress, mode):
        super(RemoteController, self).__init__(mode)
        self.running = True
        self._ipaddress = ipaddress
        self.conn = None
        self.lastpos = [0, 0]
        self.lastdmh = False  # Dead Man's Handle

    def update(self):
        """
        Blocking function which receives a joystick update from the network.
        Joystick updates are framed JSON - the first byte gives the length
        of the incoming packet.
        The JSON encodes all the joystick inputs.  Note that these are
        different for different types of joystick. This routine must
        understand the differences and pick out the correct values.
        """
        try:
            header = self.conn.recv(4)
            if len(header) < 4:
                self.conn.close()
                self.conn = None
                return {}
            length = struct.unpack("<L", header)[0]
            payload = self.conn.recv(length)
        except socket.error as msg:
            logger.error("Unexpected socket error: %s", sys.exc_info()[0])
            logger.error(msg)
            return {}
        try:
            control_msg = json.loads(payload)
        except ValueError:
            logger.error("Error in json - skipping this update")
            return {}
        logger.info(control_msg)

        pos = self.lastpos
        dmh = self.lastdmh
        retval = {}

        if control_msg["controller"] == "keypad":
            # Extract keypad data
            pos = [float(control_msg["K_RIGHT"] - control_msg["K_LEFT"]),
                   float(control_msg["K_UP"] - control_msg["K_DOWN"])]
            dmh = control_msg["K_SPACE"]

        elif control_msg["controller"] == "Wireless Controller":
            # Extract Playstation Controller data
            pos = [self.deadzone(float(control_msg['sticks'][2]), 0.1),
                   self.deadzone(float(control_msg['sticks'][3]), 0.1) * -1]
            dmh = (int(control_msg['buttons'][6]) + int(
                control_msg['buttons'][7])) > 0

        elif control_msg["controller"] == "Controller (XBOX 360 For Windows)":
            # Extract XBOX 360 controller data
            pos = [self.deadzone(float(control_msg['sticks'][4]), 0.2),
                   self.deadzone(float(control_msg['sticks'][3]), 0.2) * -1]
            dmh = abs(float(control_msg['sticks'][2])) > 0.1

        if pos != self.lastpos:
            self.lastpos = pos
            retval["pos"] = pos

        if dmh != self.lastdmh:
            self.lastdmh = dmh
            retval["dmh"] = dmh

        return retval

    def run(self):
        logger.info("Remote Controller Thread Running")
        port = 10000
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.bind((self._ipaddress, port))
        addr = None

        logger.info("Waiting for Controller Connection")
        while self.running:
            sock.listen(1)
            try:
                self.conn, addr = sock.accept()
                self.conn.setblocking(1)
            except socket.timeout:
                pass
            if self.conn is not None:
                logger.info("Controller Connected to {0}".format(addr))
            while self.running and self.conn is not None:
                controller_msg = self.update()
                self._mode.controller_update(controller_msg)

        logger.info("Remote Controller Thread Stopped")


class LocalController(Controller):
    """ Interface to a locally attached PS controller """

    def __init__(self, mode):
        super(LocalController, self).__init__(mode)
        self.running = True
        self.jsdev = None
        self.lastpos = [0, 0]
        self.lastdmh = False  # Dead Man's Handle

    def update(self):
        retval = {}
        # we want a copy of the values in self.lastpos, so we use list()
        pos = list(self.lastpos)
        dmh = self.lastdmh

        try:
            evbuf = self.jsdev.read(8)
        except IOError:
            self.jsdev = None
            return retval

        if evbuf:
            _time, _value, _type, _number = struct.unpack('IhBB', evbuf)

            if _type & 0x02:
                if _number == 0x02:  # x axis
                    pos[0] = self.deadzone(float(_value) / 32767.0, 0.1)
                elif _number == 0x05:  # y axis
                    pos[1] = self.deadzone(float(_value) / -32767.0, 0.1)
                elif _number == 0x03 or _number == 0x04:  # dmh
                    dmh = _value > 0
                elif _number == 0x07:
                    if _value > 0:
                        # down
                        retval["down"] = True
                    elif _value < 0:
                        # up
                        retval["up"] = True

        if pos != self.lastpos:
            self.lastpos = pos
            retval["pos"] = pos

        if dmh != self.lastdmh:
            self.lastdmh = dmh
            retval["dmh"] = dmh

        return retval

    def run(self):
        logger.info("Local Controller Thread Running")

        logger.info("Waiting for Controller Connection")
        while self.running:
            try:
                self.jsdev = open('/dev/input/js0', 'rb')
            except IOError:
                self.jsdev = None

            while self.running and self.jsdev is not None:
                controller_msg = self.update()
                self._mode.controller_update(controller_msg)

            time.sleep(1)  # try to connect again in a second's time

        logger.info("Local Controller Thread Stopped")


class Arduino(pykka.ThreadingActor):
    """ Interface to the Arduino over a serial connection """

    def __init__(self):
        logger.info("Arduino Actor Running")
        super(Arduino, self).__init__()
        self.ser = None
        self.open_serial()

    def open_serial(self):
        try:
            self.ser = serial.Serial(port="/dev/ttyAMA0",
                                     baudrate=115200,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE,
                                     bytesize=serial.EIGHTBITS)
            self.ser.close()
            self.ser.open()
        except serial.SerialException:
            logger.warning("Failed to Open Serial Connection")
            self.ser = None

    @staticmethod
    def on_stop():
        logger.info("Arduino Actor Stopped")

    def send_cmd(self, cmd):
        logger.info("sending: {}".format(cmd))
        if self.ser is not None:
            try:
                self.ser.write(cmd + "\r")
            except serial.SerialException:
                logger.warning("Serial Write Failed")
                self.ser.close()
                self.ser = None
        else:
            self.open_serial()


class Mode(pykka.ThreadingActor):
    def __init__(self, arduino):
        super(Mode, self).__init__()
        self._arduino = arduino

    @staticmethod
    def _xytospeeddirection(x, y):
        def sign(num):
            if num < 0:
                return -1
            else:
                return 1
        # Use the squares of x and y to make it non-linear
        speed = y * y * sign(y) * 1000.0
        direction = x * x * sign(x) * 200.0
        return int(speed), int(direction)


class JoystickMode(Mode):
    """ Joystick Mode Controller that reads inputs and drives outputs """

    def __init__(self, arduino):
        logger.info("Start Joystick Mode")
        super(JoystickMode, self).__init__(arduino)
        self._arduino_mode = 'j'
        self.modelist = MODES.keys()

        # Move J and RJ modes to the front to allow us to flip the robot
        # direction quickly
        self.modelist.insert(0, self.modelist.pop(self.modelist.index('j')))
        self.modelist.insert(0, self.modelist.pop(self.modelist.index('r')))

    def controller_update(self, update):
        if "pos" in update:
            speed, direction = self._xytospeeddirection(update["pos"][0],
                                                        update["pos"][1])
            self._arduino.send_cmd("F {0:d} {1:d}".format(speed, direction))
        if "dmh" in update:
            self._arduino.send_cmd("D {0:d}".format(update["dmh"]))
        if "down" in update:
            self._cycle_mode(True)
        if "up" in update:
            self._cycle_mode(False)

    def mode_update(self, newmode):
        logger.info("New Mode : {}".format(MODES[newmode]))
        self._arduino.send_cmd("M {}".format(newmode.upper()))
        self._arduino_mode = newmode
        if newmode in IMAGES:
            display_image(IMAGES[newmode])

    def _cycle_mode(self, direction):
        currentindex = self.modelist.index(self._arduino_mode)
        if direction:
            nextindex = (currentindex + 1) % len(self.modelist)
            self.mode_update(self.modelist[nextindex])
        else:
            nextindex = (currentindex - 1) % len(self.modelist)
            self.mode_update(self.modelist[nextindex])

    def send_cmd(self, cmd):
        logger.info("Command : {}".format(cmd))
        self._arduino.send_cmd("{}".format(cmd))

    @staticmethod
    def on_stop():
        logger.info("Stop Joystick Mode")


def main(argv):
    # Reset Arduino
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.HIGH)
    time.sleep(0.1)
    GPIO.output(17, GPIO.LOW)
    GPIO.cleanup()
    time.sleep(1)

    print "Starting main"
    logging.basicConfig(level=logging.WARNING)

    host = DEFAULT_HOST
    try:
        opts, args = getopt.getopt(argv, "i:d")
    except getopt.GetoptError:
        print 'metabot.py -d -i <ip address>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-i':
            host = arg
        elif opt == '-d':
            logger.setLevel(level=logging.DEBUG)

    print "starting serial logger"
    command = "cat /dev/ttyACM0 > /home/pi/metabot2/seriallog.txt"
    serial_logger = subprocess.Popen(command, shell=True)
    print "starting arduino"
    arduino = Arduino.start().proxy()
    mode = JoystickMode.start(arduino).proxy()
    remote_controller = RemoteController(host, mode)
    remote_controller.daemon = True
    remote_controller.start()
    local_controller = LocalController(mode)
    local_controller.daemon = True
    local_controller.start()

    print("Enter Mode Values :")
    for mode in MODES:
        print mode, MODES[mode]
    modecmd = "j"
    mode.mode_update('j')
    try:
        while modecmd != "q":
            modecmd = raw_input("New Mode :")
            if modecmd == "?":
                for mode in MODES:
                    print mode, MODES[mode]
            elif modecmd == "x":
                mode.send_cmd("X")
            elif len(modecmd) > 0 and modecmd[0] == ":":
                mode.send_cmd(modecmd[1:])
            elif modecmd in MODES and modecmd != "q":
                print("  Entering Mode : {}".format(MODES[modecmd]))
                mode.mode_update(modecmd)
    except KeyboardInterrupt:
        pass

    mode.send_cmd("X")  # Send a final Stop command
    # May need a delay or better still some sync in here to stop
    # the controller threads stopping before the Stop command gets out
    remote_controller.running = False
    local_controller.running = False
    pykka.ActorRegistry.stop_all()
    sys.exit()


def display_image(filename):
    """
    Loads a file and displays it on the screen
    :param filename: String containing the name of the file to load.
    """
    image = pygame.image.load(filename)
    screen.blit(image, (0, 0))
    pygame.display.flip()
    pygame.display.update()


if __name__ == "__main__":
    print "Looks like I've been run - kicking off main"
    main(sys.argv[1:])

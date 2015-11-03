"""
Module metabot.py

Description :
    This module controls the Raspberry Pi at the centre of the Metabot.  It connects to
    -  a controller attached to a remote PC and connected over an IP connection using JSON
    -  an Arduino connected over a serial connection
    -  a local user interface, normally exposed over SSH to a remote terminal.

Design :
    metabot.py uses a combination of Pykka Actors and separate threads to service its different work sources.
    -  main() creates the different actors and threads and connects them together before going into a loop waiting
       for user input at the terminal.
    -  The Controller class runs as a separate thread.  It waits for IP messages from the remote PC / Joystick
       controller.  It then parses these messages into a common format and sends the information to the
       ControlLoop Actor.
    -  The ControlLoop Actor processes work from incoming work sources.  At the moment that is just the Controller
       class, but in the future could be other sensors attached the RPi, timers running in other threads, or other
       work sources.  Based on the inputs it decides what actions to take and then transmits them to the Arduino.
    -  The Arduino Actor communicates over a serial connection to the Arduino.  It can be invoked using either a
       Pykka tell method, which is a one way communication, or an ask method that will wait for a reply from the
       Arduino and pass that back to the caller.

    The role of Pykka in the code below is not at all obvious.  It only shows itself in the inherited classes of
    the two actors and in the initialization and termination calls in main().  However, under the covers it is doing
    quite a bit.
    -  What looks like a simple call to a method in a Pykka actor is actually a call to a Pykka proxy instead.
    -  This serializes the parameters to the call, turns them into a message and posts them to the inbox of the
       actor thread.
    -  At some point later, that message is retrieved on the actor thread, the parameters extracted from
       the message and the target method is called.
    -  Likewise, the return value from the target method is serialized, turned into a message and sent back to the
       thread of the calling code.
    -  If the original call looks at the return parameters, then these are turned into a Pykka future, which is picked-
       up from the value in the returned message.
    The net is that we get a nice message passing co-processing scheme all done under the covers.

"""

import sys
import threading
import socket
import struct
import json
import getopt
import logging
import serial
import pykka

DEFAULT_HOST = "10.5.5.1"
logger = logging.getLogger()
MODES = {"j": "Joystick",
         "s": "Speed test",
         "l": "Line follower",
         "3": "Three point turn",
         "b": "Bowling",
         "p": "Proximity Alert",
         "q": "quit",
         "?": "help"}


class Controller(threading.Thread):
    """ Represents the connection to the controller, using JSON over an IP connection  """

    def __init__(self, ipaddress, controlloop):
        super(Controller, self).__init__()
        self.running = True
        self._controlLoop = controlloop
        self._ipaddress = ipaddress
        self.conn = None
        self.lastpos = [0, 0]
        self.lastdmh = False     # Dead Man's Handle

    @property
    def joystick_update(self):
        """
        Blocking function which receives a joystick update from the network. Joystick updates are framed
        JSON - the first byte gives the length of the incoming packet.
        The JSON encodes all the joystick inputs.  Note that these are different for different types of joystick.
        This routine must understand the differences and pick out the correct values.
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
            dmh = (int(control_msg['buttons'][6]) + int(control_msg['buttons'][7])) > 0

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

    @staticmethod
    def deadzone(val, cutoff):
        if abs(val) < cutoff:
            return 0
        elif val > 0:
            return val - cutoff
        else:
            return val + cutoff

    def run(self):
        logger.info("Controller Thread Running")
        port = 10000
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        s.bind((self._ipaddress, port))
        addr = None

        logger.info("Waiting for Controller Connection")
        while self.running:
            s.listen(1)
            try:
                self.conn, addr = s.accept()
                self.conn.setblocking(1)
            except socket.timeout:
                pass
            if self.conn is not None:
                logger.info("Controller Connected to {0}".format(addr))
            while self.running and self.conn is not None:
                controller_msg = self.joystick_update
                self._controlLoop.controller_update(controller_msg)

        logger.info("Controller Thread Stopped")


class Arduino(pykka.ThreadingActor):
    """ Represents the connection to the Arduino over a serial connection """

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

    def on_stop(self):
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


class ControlLoop(pykka.ThreadingActor):
    """ Control loop that reads inputs and drives outputs """

    def __init__(self, arduino):
        logger.info("Control Loop Actor Running")
        super(ControlLoop, self).__init__()
        self._arduino = arduino

    @staticmethod
    def _xytolr(x, y):
        speed = y * 1000.0
        direction = x * 200.0
        l = int(speed + direction)
        r = int(speed - direction)
        return l, r

    def controller_update(self, update):
        if "pos" in update:
            l, r = self._xytolr(update["pos"][0], update["pos"][1])
            self._arduino.send_cmd("F {0:d} {1:d}".format(l, r))
        if "dmh" in update:
            self._arduino.send_cmd("D {0:d}".format(update["dmh"]))

    def mode_update(self, newmode):
        logger.info("New Mode : {}".format(MODES[newmode]))
        self._arduino.send_cmd("M {}".format(newmode.upper()))

    def on_stop(self):
        logger.info("Control Loop Actor Stopped")


def main(argv):
    logging.basicConfig(level=logging.WARNING)

    host = DEFAULT_HOST
    try:
        opts, args = getopt.getopt(argv, "i:d")
    except getopt.GetoptError:
        print 'metabot.py -i <ip address>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-i':
            host = arg
        elif opt == '-d':
            logger.setLevel(level=logging.DEBUG)

    arduino = Arduino.start().proxy()
    control_loop = ControlLoop.start(arduino).proxy()
    controller = Controller(host, control_loop)
    controller.start()

    print("Enter Mode Values :")
    for s in MODES:
        print s, MODES[s]
    mode = "j"
    while mode != "q":
        mode = raw_input("New Mode :")
        if mode == "?":
            for s in MODES:
                print s, MODES[s]
        elif mode in MODES and mode != "q":
            print("  Entering Mode : {}".format(MODES[mode]))
            control_loop.mode_update(mode)

    controller.running = False
    pykka.ActorRegistry.stop_all()
    sys.exit()

if __name__ == "__main__":
    main(sys.argv[1:])

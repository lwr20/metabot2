import sys
import threading
import socket
import struct
import json
import getopt
import logging
import serial
import pykka

HOST = "10.5.5.1"
logger = logging.getLogger()

class Controller(threading.Thread):
    """ Represents the connection to the controller, using JSON over an IP connection
    """

    def __init__(self, ipaddress, controlloop):
        super(Controller, self).__init__()
        self.running = True
        self._controlLoop = controlloop
        self._ipaddress = ipaddress
        self.conn = None

    def joystickUpdate(self):
        """
        Blocking function which receives a joystick update from the
        network. Joystick updates are framed JSON - the first byte gives
        the length of the incoming packet.
        The JSON encodes all the joystick inputs:
        {
            'sticks': ["0", "0", "0", "0", "0"]
            'buttons': ["0", "0", "0", "0", "0", "0", "0", "0", "0"]
            'hats': "(0,0)"
        }
        controlData['sticks'][x] is:
          x=0: left stick horizontal - right is +ve
          x=1: left stick vertical - down is +ve
          x=2: sum of triggers - left trigger is +ve
          x=3: right stick vertical - down is +ve
          x=4: right stick horizontal - right is +ve

        controlData['buttons'][x] is:
          x=0: A
          x=1: B
          x=2: X
          x=3: Y
          x=4: LB
          x=5: RB
          x=6: back
          x=7: start
          x=8: left_joy
          x=9: right_joy

        """
        try:
            header = self.conn.recv(4)
            if len(header)< 4:
                self.conn.close()
                self.conn = None
                return {}
            length = struct.unpack("<L", header)[0]
            payload = self.conn.recv(length)
        except socket.error as msg:
            logger.error("Unexpected socket error: %s", sys.exc_info()[0])
            logger.error(msg)
            return{}
        try:
            controlData = json.loads(payload)
        except:
            logger.error("Error in json - skipping this update")
            return {}

        logger.info(controlData)
        if controlData["controller"] == "keypad":
            x = float(controlData["K_RIGHT"] - controlData["K_LEFT"])
            y = float(controlData["K_UP"] - controlData["K_DOWN"])
            return {"x": x, "y": y}
        elif controlData["controller"] == "Wireless Controller":
            x = self.deadzone(float(controlData['sticks'][2]), 0.05)
            y = self.deadzone(float(controlData['sticks'][3]), 0.05) * -1
            return {"x": x, "y": y}
        elif controlData["controller"] == "Controller (XBOX 360 For Windows)":
            x = self.deadzone(float(controlData['sticks'][4]), 0.2)
            y = self.deadzone(float(controlData['sticks'][3]), 0.2) * -1
            return {"x": x, "y": y}

        return {}

    def deadzone(self, val, cutoff):
        if abs(val) < cutoff:
            return 0
        elif (val > 0):
            return val-cutoff
        else:
            return val+cutoff

    def run(self):
        logger.info("Controller Thread Running")
        port = 10000
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1.0)
        s.bind((self._ipaddress, port))
        addr = None

        logger.info("Waiting for Controller Connection")
        while self.running and self.conn is None:
            s.listen(1)
            try:
                self.conn, addr = s.accept()
                self.conn.setblocking(1)
            except socket.timeout:
                pass
            if self.conn is not None:
                logger.info("Controller Connected to {0}".format(addr))
            while self.running and self.conn is not None:
                joystickData = self.joystickUpdate()
                self._controlLoop.controllerUpdate(joystickData)

        logger.info("Controller Thread Stopped")


class Arduino(pykka.ThreadingActor):
    """ Represents the connection to the Arduino over a serial connection
    """

    def __init__(self):
        logger.info("Arduino Actor Running")
        super(Arduino, self).__init__()
        try:
            self.ser = serial.Serial(port="/dev/ttyACM0",
                                 baudrate=115200,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS)
            self.ser.close()
            self.ser.open()
        except serial.SerialException:
            self.ser = None

    def on_stop(self):
        logger.info("Arduino Actor Stopped")

    def sendCmd(self, cmd):
        logger.info("sending: {}".format(cmd))
        if self.ser != None:
            self.ser.write(cmd)


class ControlLoop(pykka.ThreadingActor):
    """ Control loop that reads inputs and drives outputs
    """

    def __init__(self, arduino):
        logger.info("Control Loop Actor Running")
        super(ControlLoop, self).__init__()
        self._arduino = arduino

    def _xytolr(self, x, y):
        speed = y * 1000.0
        dir = x * 200.0
        l = int(speed + dir)
        r = int(speed - dir)
        return l, r

    def controllerUpdate(self, update):
        if "x" in update:
            l, r = self._xytolr(update["x"], update["y"])
            self._arduino.sendCmd("F {0:d} {1:d}".format(l, r))

    def on_stop(self):
        logger.info("Control Loop Actor Stopped")


def main(argv):
    logging.basicConfig(level=logging.WARNING)
    host = HOST
    try:
        opts, args = getopt.getopt(argv, "i:")
    except getopt.GetoptError:
        print 'metabot.py -i <ip address>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-i':
            host = arg

    arduino = Arduino.start().proxy()
    controlLoop = ControlLoop.start(arduino).proxy()
    controller = Controller(host, controlLoop)
    controller.start()
    mode = "j"
    while mode != "q":
        mode = raw_input ("Enter Mode :")

    controller.running = False
    pykka.ActorRegistry.stop_all()


if __name__ == "__main__":
    main(sys.argv[1:])

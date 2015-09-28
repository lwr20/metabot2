# import smbus
import time, sys, subprocess, math, threading
import socket, pprint, struct, json
import getopt
import serial
from ast import literal_eval as make_tuple


class serial_port():
    def __init__(self):
        self.ser = serial.Serial(port="/dev/ttyACM0",
                            baudrate=115200,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS)
        self.ser.close()
        self.ser.open()

    def send(self, command, params=[]):
        cmd = "%s %s\r" % (command, " ".join(map(str, params)))
        print "sending: %s" % cmd
        self.ser.write(cmd)

    def read_data(self):
        pass


class I2C():
    """Deals with the i2c channel to the arduino"""
    def __init__(self, address=0x04, bus=1):
        # Note that you would set bus=0 for the version 1 RPi
        self.channel = smbus.SMBus(bus)
        self.address = address

    def send(self, command, params=[]):
        """Command is a string character, params is a list"""
        #print "{}, {}".format(command, params)
        try:
            self.channel.write_i2c_block_data(self.address, ord(command), params)
            #pass
        except IOError as e:
            print e
            print "Attempting to carry on"
            subprocess.call(['i2cdetect', '-y', '1'])

    def _read_byte(self):
        try:
            return self.channel.read_byte(self.address)
        except IOError as e:
            print e
            print "Attempting to carry on"
            subprocess.call(['i2cdetect', '-y', '1'])
        return None

    def read_data(self):
        """Reads sensor data from the Arduino and does format conversion"""
        # Data format is:
        # S1, S2, A, B, B, C, C, D, D, D, D, E, E, E, E, T1, T2

        s = struct.Struct('=BBBHHLLBB')
        S1 = 0xAA      # Framing bytes
        S2 = 0x55
        T1 = 0x0F
        T2 = 0xF0
        BYTES_PER_FRAME = 17
        RETRIES = BYTES_PER_FRAME - 1  # Worst case is we're off by 1
        MAX_BYTE_RETRY = 3


        input_string = ""
        retry = 0
        byte = 0
        for i in range(BYTES_PER_FRAME):
            while byte is None or retry >= MAX_BYTE_RETRY:
                byte = self._read_byte()
                if byte is None:
                    retry += 1
            if byte is not None:
                input_string += chr(self._read_byte())
            else:
                return None

        retry = 0
        while retry < RETRIES:
            #print "input_string length is %s" % len(input_string)
            #print "input_string is %s" % input_string
            input = list(s.unpack(input_string))
            #print "input = %s" % input
            # We'll use k to step through input[]
            # We're doing it like this to allow easy swapping
            # back to just doing read_byte() directly
            k = 0
            if input[k] == S1:
                k += 1
                if input[k] == S2:
                    k += 1
                    data = {}
                    # read 8 digital sensors: A
                    assert input[k] >= 0
                    data["digital"] = input[k]
                    k += 1
                    # read 2 counts: B, C
                    for i in range(2):
                        assert input[k] >= 0
                        data["count%d" % i] = input[k]
                        k += 1
                    # read Motor period values DDDD and EEEE (4 bytes each)
                    # (these give a long containing microseconds for 1/10 revolution)
                    for i in range(2):
                        assert input[k] >= 0
                        data["motor%d" % i] = input[k]
                        k += 1
                    if input[k] == T1:
                        k += 1
                        if input[k] == T2:
                            return data
            # If we get here, then the frame didn't look right.
            # So dump the first byte, and read a new last one.
            # Then re-run it through the code above to see if it looks OK now
            input_string = input_string[1:]
            input_string += chr(self._read_byte())
            #print "Retrying. Input = %s" % input
            retry += 1
        return None

class Motor:
    """Represents a motor.  Stores whether the motor rotation is reversed,
    current speed, desired speed, etc.
    """

    def __init__(self, reversed=False, id=0):
        self.reversed = reversed
        self.measured_speed = 0
        self.measurement_update_time = 0
        self.desired_speed = 0
        self.id = id
        self.count = 0
        self.offset = 0

    def change_speed(self, value):
        """Changes desired speed of motor.  Value can be negative."""
        self.desired_speed = self.desired_speed + value
        # add code to send update to arduino!
        return self.desired_speed

    def set_measured_period(self, period):
        """Sets the current measured speed from the period reported by Arduino.
        Expected to be called by the routine which reads the sensors.
        """
        self.measurement_update_time = time.time()
        # So the period reported by arduino is time in microseconds for
        # 1/10 of a revolution.
        # the plus 1 is to ensure we don't try to divide by zero!
        # convert to speed in revs per sec
        speed = (1000000 / 3) / float(period)
        self.measured_speed = speed

    def countup(self, value):
        self.count += value

    def reset_count(self):
        self.count = 0

    def get_dir(self):
        """Returns the motor direction as a boolean"""
        return self.desired_speed > 0

    def get_current_speed(self):
        """Returns the current motor speed and direction (the sign of the
        returned value)
        """
        #valid = True
        #if (time.time() > self.measurement_update_time + 1):
        #    print "Speed has not been updated for a while!"
        #    valid = False

        # There is an assumption in the following that the motor always turns
        # in the direction we told it to. Should be fine due to our motors.
        if self.get_dir():
            return self.measured_speed
        else:
            return -self.measured_speed

class Scheduler(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._exitflag = False

    def measure_speed(self, interval=0.1):
        self.robot.get_sensors()
        lstartcount = self.robot.left_motor.count
        rstartcount = self.robot.right_motor.count
        starttime = time.time()
        time.sleep(interval)
        self.robot.get_sensors()
        lendcount = self.robot.left_motor.count
        rendcount = self.robot.right_motor.count
        endtime = time.time()
        lspeed = (lendcount - lstartcount) / (endtime - starttime)
        rspeed = (rendcount - rstartcount) / (endtime - starttime)
        return lspeed, rspeed

    def finish(self):
        self._exitflag = True

class SensorLoop(Scheduler):
    def __init__(self, robot):
        super(SensorLoop, self).__init__()
        self.robot = robot

    def run(self):
        now = time.time()
        lcount = self.robot.left_motor.count
        rcount = self.robot.right_motor.count
        self.lasttime = now
        self.last_left_count = lcount
        self.last_right_count = rcount
        while self._exitflag == False:
            self.robot.get_sensors()
            now = time.time()
            lcount = self.robot.left_motor.count
            rcount = self.robot.right_motor.count

            left_speed = (lcount - self.last_left_count) / (now - self.lasttime)
            right_speed = (rcount - self.last_right_count) / (now - self.lasttime)

            self.lasttime = now
            self.last_left_count = lcount
            self.last_right_count = rcount

            #print "{}: Speeds.  L: {:.2},  R: {:.2}   Counts: L: {}    R: {}".format(now, left_speed, right_speed, lcount, rcount)
            time.sleep(0.1)
        print "SensorLoop exit"
        sys.exit(0)

class Robot:
    def __init__(self):
        self.left_motor = Motor(reversed=False, id=0)
        self.right_motor = Motor(reversed=False, id=1)
        self.i2c = serial_port()
        self.direction = 0   # direction is the intended direction. -1 to +1
        self.speed = 0   # speed is the forward speed of the robot.
        #self.sensorloop = SensorLoop(self)
        #self.sensorloop.daemon=True
        #self.sensorloop.start()
        self.line_sensor = 0
        self.lastspeed = 0
        self.lastdir = 0

        # This factor determined by graphing speed/setting for the motors.
        self.left_motor_corr = 1.0

    def _send_speed_update(self):
        """Sends a speed update to the Arduino after applying transforms to the
        values to take account of reversal, etc.
        """
        if self.left_motor.reversed:
            left_motor_dir = not self.left_motor.get_dir()
        else:
            left_motor_dir = self.left_motor.get_dir()
        left_motor_spd = abs(self.left_motor.desired_speed)*self.left_motor_corr

        if self.right_motor.reversed:
            right_motor_dir = not self.right_motor.get_dir()
        else:
            right_motor_dir = self.right_motor.get_dir()
        right_motor_spd = abs(self.right_motor.desired_speed)

        if left_motor_spd > 254:
            left_motor_spd = 254

        if right_motor_spd > 254:
            right_motor_spd = 254

        self.i2c.send('M', params=[
                        int(left_motor_dir),
                        int(left_motor_spd),
                        int(right_motor_dir),
                        int(right_motor_spd),
                      ])

    def _update_motor_speeds(self):
        # MAX_SPEED is a number between 0 and 254 which controls the max
        # speed of the motors.  254 = flat out!
        MAX_SPEED = 254
        #MAX_SPEED = 127
        x = self.direction
        y = self.speed
        v = (1 - abs(x)) * y + y
        w = (1 - abs(y)) * x + x

        self.left_motor.desired_speed  = MAX_SPEED * ((v + w) / 2)
        self.right_motor.desired_speed = MAX_SPEED * ((v - w) / 2)

        self._send_speed_update()

    def set_direction(self, direction):
        """Sets the 'direction' of the robot to a value.  Can be -ve
        Expected to be a value between -1 and +1.
        """
        print "set_direction: %s" % direction
        assert direction <= 1
        assert direction >= -1
        self.direction = direction
        # Don't bother updating dir if it is the same as last time.
        if self.direction != self.lastdir:
            self._update_motor_speeds()
            self.lastdir = self.direction
        self._update_motor_speeds()

    def set_speed(self, speed):
        """Sets the forward speed of the robot to a value.  Can be -ve
        Expected to be a value between -1 and +1.
        """
        print "set_speed: %s" % speed
        assert speed <= 1
        assert speed >= -1
        self.speed = speed
        # Don't bother updating speed if it is the same as last time.
        if self.speed != self.lastspeed:
            self._update_motor_speeds()
            self.lastspeed = self.speed

    def reset_counters(self):
        self.left_motor.reset_count()
        self.right_motor.reset_count()

    def get_sensors(self):
        return

        self.i2c.send('R')
        data = self.i2c.read_data()
        if data is not None:
            #pp.pprint(data)

            # In the line sensor byte, only the bits marked B are connected
            # to a sensor: XXBBBBBX
            # So zero any elements we aren't using
            sensor = data['digital'] & 0b00111110
            # And right shift it so that the interesting bits are in the
            # lowest positions.
            sensor = sensor / 2
            self.line_sensor = sensor

            # print the digital byte in binary form, chopping off the 0b
            # from the front.
            # print bin((data['digital'] & 0b00111110) | 0b11000001)[2:],
            # print bin(sensor)

            self.left_motor.set_measured_period(data['motor0'])
            self.right_motor.set_measured_period(data['motor1'])
            # print "Left Speed: {}, Right Speed: {}".format(
            #     self.left_motor.get_current_speed(),
            #     self.right_motor.get_current_speed())
            self.left_motor.countup(data['count0'])
            self.right_motor.countup(data['count1'])

    def set_servo(self, angle):
        return
        self.i2c.send('V', params=[angle])

    def stop(self):
        self.set_speed(0)
        self.set_direction(0)

    def finish(self):
        self.stop()
        # self.sensorloop.finish()

class Controller(object):
    def __init__(self):
        self.exitflag = False
        self.max_speed = 0.9
        self.right_counts_per_degree = 1
        self.left_counts_per_degree = 1

    def sign(self, value):
        """Returns the sign of value."""
        if value > 0:
            return 1
        return -1

    def measure_speed(self, interval=0.1):
        self.robot.get_sensors()
        lstartcount = self.robot.left_motor.count
        rstartcount = self.robot.right_motor.count
        starttime = time.time()
        time.sleep(interval)
        self.robot.get_sensors()
        lendcount = self.robot.left_motor.count
        rendcount = self.robot.right_motor.count
        endtime = time.time()
        lspeed = (lendcount - lstartcount) / (endtime - starttime)
        rspeed = (rendcount - rstartcount) / (endtime - starttime)
        return lspeed, rspeed

    def move(self, count, direction, speed):
        """ Performs a single movement before returning. Carefully
        monitors the right wheel encoder to decide when the move is
        complete.
        """
        print "Entered Move.  Cnt: {}, Dir: {}, Spd: {}".format(count, direction, speed)
        self.robot.set_speed(speed)
        if speed == 0:
            self.robot.set_direction(direction)
        else:
            self.robot.set_direction(direction+self.offset)
        self.robot.get_sensors()
        first_count = self.robot.right_motor.count
        #first_count = self.robot.left_motor.count
        cur_count = self.robot.right_motor.count
        #cur_count = self.robot.left_motor.count
        while (cur_count - first_count) < count:
            self.robot.get_sensors()
            cur_count = self.robot.right_motor.count
            #cur_count = self.robot.left_motor.count
            print "    {}/{}".format((cur_count - first_count), count)
            time.sleep(0.01)
        self.robot.stop()
        print "Exiting Move."

    def forward(self, count):

        self.move(count, 0, self.max_speed)

    def right(self, angle):
        count = angle * self.right_counts_per_degree
        self.move(count, -self.max_speed, 0)

    def left(self, angle):
        count = angle * self.left_counts_per_degree
        self.move(count, self.max_speed, 0)

    def back(self, count):
        self.move(count, 0, -self.max_speed)

    def servo(self, pos):
        """ pos is a boolean and indicates if we want the servo gate open
        or closed.  True = open.
        """
        if pos:
            self.robot.set_servo(90)
        else:
            self.robot.set_servo(0)

    def finish(self):
        self.robot.stop()
        self.robot.finish()
        self.exitflag = True


class JoystickController(Controller):
    def __init__(self):
        print "creating JoystickController"
        super(JoystickController, self).__init__()
        HOST = ipaddress    # The IP address for the robot
        PORT = 10000
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((HOST, PORT))
        s.listen(1)
        self.conn, addr = s.accept()
        self.reverse_throttle = False
        self.reverse_direction = True

        self.robot = Robot()

    def scale(self, x, k=0.5):
        """
        Scales the signal (x) by factor k.
        """
        return x * k

    def expo(self, x, k=2):
        """ Takes an input (x) from -1 to +1.
        Returns y = (x*x*x*(k-1)+x)/k
        This approximates an RC 'expo' curve
        which makes the stick less sensitive near the centre
        and more sensitive towards the edges while still allowing full
        deflection at the extremes.
        K is the exponential 'factor' - the higher, the greater
        the effect.
        """
        return (x * x * x * (k - 1) + x) / k

    def dead_band(self, x):
        """Apply a 'dead-band' - an area near the centre which is all zero
        to ensure that when you let go of the stick, robot always stops.
        """
        # Value chosen by experiment - move and let go of the sticks a few
        # times - this value needs to be (just) higher than any value you
        # observe having let go of the sticks.
        if abs(x) < 0.15:
            x = 0
        return x

    def invert(self, x):
        return -x

    def joystick_update(self):
        """ Blocking function which receives a joystick update from the
        network. Joystick updates are framed JSON - the first byte gives
        the length of the incoming packet.
        The JSON encodes all the joystick inputs:
        {
            'sticks': ["0", "0", "0", "0", "0"]
            'buttons': ["0", "0", "0", "0", "0", "0", "0", "0", "0"]
            'hats': "(0,0)"
        }
        simple_joy['sticks'][x] is:
          x=0: left stick horizontal - right is +ve
          x=1: left stick vertical - down is +ve
          x=2: sum of triggers - left trigger is +ve
          x=3: right stick vertical - down is +ve
          x=4: right stick horizontal - right is +ve

        simple_joy['buttons'][x] is:
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
            length = struct.unpack("<L", header)[0]
            payload = self.conn.recv(length)
        except:
            print "Unexpected error:", sys.exc_info()[0]
            print "Shutting down"
            self.finish()
            sys.exit(1)
        try:
            simple_joy = json.loads(payload)
        except:
            print "Error in json - skipping this update"
            return None
        return simple_joy

    def run(self):
        print "Running"
        while not self.exitflag:
            self.loop()

    def loop(self):
        """ This is the joystick receiver loop.  It is event driven (i.e.
        when a joystick update comes in, we run through the loop.
        The sender generally ensures that we only get an update when
        something changes, but we should not assume that here.
        """
        simple_joy = self.joystick_update()
        if simple_joy is not None:
            print "simplejoy = %s" % simple_joy
            throttle = float(simple_joy['sticks'][3])
            direction = float(simple_joy['sticks'][4])
            print "Throttle: %s" % throttle
            print "Direction: %s" % direction

            throttle = self.dead_band(throttle)
            direction = self.dead_band(direction)

            throttle = self.expo(throttle)
            direction = self.expo(direction)

            throttle = self.scale(throttle, 0.7)
            direction = self.scale(direction, 0.5)

            if self.reverse_throttle:
                throttle = self.invert(throttle)
            self.robot.set_speed(throttle)

            if self.reverse_direction:
                direction = self.invert(direction)
            self.robot.set_direction(direction)

            hatx, haty = make_tuple(simple_joy['hats'][0])
            #print hatx, haty
            if haty == -1:
                self.servo(False)
            if haty == 1:
                self.servo(True)

            # @@LR2: Insert code calling macros in here.


class SpeedScheduler(Scheduler):
    def __init__(self, robot):
        super(SpeedScheduler, self).__init__()
        self.robot = robot
        self.button = 0
        self.speed = 0
        self.direction = -0.02
        self.robot.set_direction(0)

    def run(self):
        """ When the button is pressed, ramp the speed of the motors up to
        max and hold it there.  When button is released, stop immediately.
        """
        MAX_SPEED = -0.2
        RAMPUP = 0.1
        RAMPDOWN = 0.2
        DIR_INC = 0.01
        DIR_NUDGE = 0.2

        while not self._exitflag:
            lspd, rspd = self.measure_speed()
            print "Speeds: {} : {} : {} : {}".format(lspd, rspd, self.direction, self.speed)

            # # decide if we're going straight - if not, adjust!
            # if lspd < rspd:
                # self.direction -= 0.02
            # elif lspd > rspd:
                # self.direction += 0.02

            # # Keep direction in range
            # if self.direction > 1:
                # self.direction = 1
            # elif self.direction < -1:
                # self.direction = -1

            # if the button is pressed, speed up to max
            if self.button == 1:
                self.speed -= RAMPUP
                if self.speed < MAX_SPEED:
                    self.speed = MAX_SPEED
            else:
                #print "SpeedScheduler loop: Button up"
                # We have to stop gradually using the routine below:
                # It turns out that if you just stop dead, the robot flips
                # over (guess how we found that out...)!
                 if self.speed < 0:
                    self.speed += RAMPDOWN
                    if self.speed > 0:
                        self.speed = 0

            self.robot.get_sensors()
            nudge = 0
            if not (self.robot.line_sensor & 0b00100):
                nudge += DIR_NUDGE
            if not (self.robot.line_sensor & 0b01000):
                nudge -= DIR_NUDGE

            self.robot.set_speed(self.speed)
            if self.speed != 0:
                self.robot.set_direction(self.direction + nudge)
            else:
                self.robot.set_direction(0)
            time.sleep(0.1)

        print "SpeedScheduler exit"
        self.robot.stop()

class SpeedController(JoystickController):
    def __init__(self):
        super(SpeedController, self).__init__()
        self.robot = Robot()
        self.scheduler = SpeedScheduler(self.robot)
        self.scheduler.daemon=True
        self.scheduler.start()

    def run(self):
        print "Running"
        while not self.exitflag:
            self.loop()

    def loop(self):
        """ A very simple controller which only looks for 2 buttons:
        One is a 'dead mans handle': the robot only runs when this is pressed.
        The other is a 'quit' button for disarming the robot at the end of
        the run.
        """
        simple_joy = self.joystick_update()
        #pp.pprint(simple_joy['buttons'])

        #print "setting button to %s" % simple_joy['buttons'][1]
        self.scheduler.button = int(simple_joy['buttons'][1])
        #print "Quit button = %s" % simple_joy['buttons'][0]
        if int(simple_joy['buttons'][0]) == 1:
            self.robot.finish()
            self.scheduler.finish()
            self.finish()
            sys.exit(0)


class CalibController(JoystickController):
    def __init__(self):
        super(CalibController, self).__init__()
        self.left_dir = 0
        self.right_dir = 1
        self.i2c = I2C()

    def run(self):
        """ A very simple controller which only looks for 2 buttons:
        One is a 'dead mans handle': the robot only runs when this is pressed.
        The other is a 'quit' button for disarming the robot at the end of
        the run.
        """
        lpwm = 0
        rpwm = 0

        simple_joy = self.joystick_update()
        while int(simple_joy['buttons'][1]) == 0:
            simple_joy = self.joystick_update()
            time.sleep(0.1)
        while int(simple_joy['buttons'][1]) == 1 and lpwm < 255 and rpwm < 255:
            self.i2c.send('M', params=[
                            int(self.left_dir),
                            int(lpwm),
                            int(self.right_dir),
                            int(rpwm),
                          ])
            lspd, rspd = self.measure_speed()
            print "{}:{}, {}:{}".format(lpwm, lspd, rpwm, rspd)
            if lspd < rspd:
                lpwm += 1
            else:
                rpwm += 1
            simple_joy = self.joystick_update()

        self.i2c.send('M', params=[
                        int(self.left_dir),
                        int(0),
                        int(self.right_dir),
                        int(0),
                      ])


class ThreePointTurnController(Controller):
    def __init__(self):
        super(ThreePointTurnController, self).__init__()
        self.robot = Robot()
        self.max_speed = 0.12
        self.offset = -0.006

    def run(self):
        print "Running"
        self.forward(100)
        time.sleep(4)
        self.right(90)
        time.sleep(4)
        self.back(100)
        time.sleep(4)
        self.left(180)
        time.sleep(4)
        self.forward(100)
        time.sleep(4)
        self.right(360)
        time.sleep(4)
        self.left(360)
        time.sleep(4)
        # for angle in range(180):
        #     self.robot.set_servo(angle)
        #     time.sleep(0.1)
        self.robot.stop()
        self.robot.finish()

class LineFollowerScheduler(Scheduler):
    def __init__(self, robot):
        super(LineFollowerScheduler, self).__init__()
        self.robot = robot
        self.button = 0
        self.robot.stop()
        self.last_pos = 0.0
        self.delta_t = 0.1
        # PID values
        self.kp = 0.075
        self.ki = 0.0        # Tune this last.  We may do without it.
        self.kd = 0.0 #0.02        # Maybe try with this ~20 times bigger than kp?
        self.speed = -0.1
        self.pos_dict = {
            0b00001 : 2,
            0b00011 : 2,
            0b00111 : 2,
            0b00010 : 1,
            0b01111 : 1,
            0b00110 : 1,
            0b11111 : 0,
            0b01110 : 0,
            0b00100 : 0,
            0b01100 : -1,
            0b11110 : -1,
            0b01000 : -1,
            0b11100 : -2,
            0b11000 : -2,
            0b10000 : -2,
            }
        # Tuning PID

        # Once you have PID running in your robot, you will probably notice
        # that it still doesn't follow the line properly. It may even perform
        # worse than it did with just proportional!  The reason behind this is
        # you haven't tuned the PID routine yet.  PID requires the Kp, Ki and
        # Kd factors to be set to match your robot's characteristics and these
        # values will vary considerably from robot to robot.  Unfortunately,
        # there is no easy way to tune PID. It requires manual trial and error
        # until you get the desired behaviour.  There are some basic guidelines
        # that will help reduce the tuning effort.

        # Start with Kp, Ki and Kd equalling 0 and work with Kp first. Try
        # setting Kp to a value of 1 and observe the robot. The goal is to get
        # the robot to follow the line even if it is very wobbly. If the robot
        # overshoots and loses the line, reduce the Kp value. If the robot
        # cannot navigate a turn or seems sluggish, increase the Kp value.

        # Once the robot is able to somewhat follow the line, assign a value of
        # 1 to Kd (skip Ki for the moment). Try increasing this value until you
        # see lesser amount of wobbling.

        # Once the robot is fairly stable at following the line, assign a value
        # of 0.5 to 1.0 to Ki. If the Ki value is too high, the robot will jerk
        # left and right quickly. If it is too low, you won't see any
        # perceivable difference.  Since Integral is cumulative, the Ki value
        # has a significant impact. You may end up adjusting it by .01
        # increments.

        # Once the robot is following the line with good accuracy, you can
        # increase the speed and see if it still is able to follow the line.
        # Speed affects the PID controller and will require retuning as the
        # speed changes.

        # Lastly, please keep in mind that you do not need to implement all
        # three controllers (proportional, derivative, and integral) into a
        # single system, if not necessary. For example, if a PI controller
        # gives a good enough response, then you don't need to implement
        # derivative controller to the system. Keep the controller as simple
        # as possible.


    def get_position(self, sensor):
        """
        Takes a list of sensor booleans and returns a value which describes
        where the line is.
        """
        output = None

        if sensor in self.pos_dict:
            output = self.pos_dict[sensor]
            self.last_pos = output
        # elif sensor == 0:
            # #0b00000 : 5 or -5 (depending on previous values)
            # if self.last_pos > 0:
                # #output = 3
                # output = 0
            # elif self.last_pos < 0:
                # #output = -3
                # output = 0

        # if output is None:
            # print "ERROR!  Could not figure out where line is!"
            # print "Sensor = %s" % bin(sensor)
            # #output = self.last_pos
            # output = 0



        return output

    def run(self):
        """
        When the button is pressed, start moving and looking for the line.
        When button is released, stop immediately.
        """
        curr_pos = None
        while curr_pos is None:
            self.robot.get_sensors()
            curr_pos = self.get_position(self.robot.line_sensor)
            print curr_pos
            time.sleep(self.delta_t)
        previous_error = curr_pos
        integral = 0
        while not self._exitflag:
            if self.button == 1:
                self.robot.get_sensors()
                curr_pos = self.get_position(self.robot.line_sensor)
                if curr_pos is not None:
                    error = curr_pos
                    integral = integral + (error * self.delta_t)
                    derivative = (error - previous_error) / self.delta_t
                    output = (self.kp * error) \
                        + (self.ki * integral) \
                        + (self.kd * derivative)
                    previous_error = error

                    # Cap the absolute value of the output to the permitted range
                    if output > 1:
                        output = 1
                    elif output < -1:
                        output = -1

                    print output, self.kp*error, self.ki*integral, self.kd*derivative, curr_pos

                    self.robot.set_direction(output)
                    self.robot.set_speed(self.speed)
                else:
                    # We've lost the line, so stop and spin on the spot
                    # to hunt for it
                    self.robot.set_speed(0)
                    if previous_error > 0:
                        self.robot.set_direction(0.2)
                    else:
                        self.robot.set_direction(-0.2)

            else:
                self.robot.stop()
            time.sleep(self.delta_t)

        print "LineFollowerScheduler exit"
        self.robot.stop()


class LineFollowerController(JoystickController):
    def __init__(self):
        super(LineFollowerController, self).__init__()
        self.robot = Robot()
        self.scheduler = LineFollowerScheduler(self.robot)
        self.scheduler.daemon=True
        self.scheduler.start()

    def run(self):
        print "Running"
        while not self.exitflag:
            self.loop()


    def loop(self):
        """ A very simple controller which only looks for 2 buttons:
        One is a 'dead mans handle': the robot only runs when this is pressed.
        The other is a 'quit' button for disarming the robot at the end of
        the run.
        """
        simple_joy = self.joystick_update()
        #pp.pprint(simple_joy['buttons'])

        #print "setting button to %s" % simple_joy['buttons'][1]
        self.scheduler.button = int(simple_joy['buttons'][1])
        #print "Quit button = %s" % simple_joy['buttons'][0]
        if int(simple_joy['buttons'][0]) == 1:
            #self.robot.finish()
            print "Controller exiting"
            self.scheduler.finish()
            self.finish()
            sys.exit(0)


class TestController(Controller):
    def __init__(self):
        super(TestController, self).__init__()
        self.robot = Robot()
        self.max_speed = 0.0

    def run(self):
        print "Running"

        setspeed = 0.14
        while setspeed <= 0.65:
            self.robot.set_speed(setspeed)
            time.sleep(1)
            self.robot.get_sensors()
            lstartcount = self.robot.left_motor.count
            rstartcount = self.robot.right_motor.count
            starttime = time.time()
            time.sleep(1)
            self.robot.get_sensors()
            lendcount = self.robot.left_motor.count
            rendcount = self.robot.right_motor.count
            endtime = time.time()
            lspeed = (lendcount - lstartcount) / (endtime - starttime)
            rspeed = (rendcount - rstartcount) / (endtime - starttime)
            print lendcount, lstartcount, rendcount, rstartcount, endtime, starttime
            print "Set Speed = {}, Left speed = {}, Right speed = {}".format(setspeed, lspeed, rspeed)
            setspeed += 0.05

        self.robot.stop()
        self.robot.finish()

class WallStopScheduler(Scheduler):
    """ Takes care of driving straight.
    """
    def __init__(self, robot):
        super(WallStopScheduler, self).__init__()
        self.robot = robot
        self.stopped = False

    def run(self):
        direction = -0.006
        integral = 0
        KP = 0.0 # 1.5
        KI = 0.0
        KD = 0.0 # 0.5
        last_error = 0
        print "Starting loop"
        while not self.stopped:
            self.robot.set_direction(direction)
            lspd, rspd = self.measure_speed(interval=0.2)
            # decide if we're going straight - if not, adjust!
            if lspd < rspd:
                integral -= 0.02
            elif lspd > rspd:
                integral += 0.02

            error = (lspd - rspd) / 900
            delta_error = last_error - error

            #direction = (KI * integral) + (KP * error) + (KD * delta_error)

            # Keep direction in range
            if direction > 1:
                direction = 1
            elif direction < -1:
                direction = -1
            print "Speeds: {} : {} : {} : {} : {} : {}".format(lspd,
                                                               rspd,
                                                               direction,
                                                               error,
                                                               integral,
                                                               delta_error)
        self.robot.stop()


class WallStop(Controller):
    """ Looks out for walls.
    """
    def __init__(self):
        super(WallStop, self).__init__()
        self.robot = Robot()
        self.max_speed = -0.12
        self.scheduler = WallStopScheduler(self.robot)
        self.scheduler.daemon=True

    def run(self):
        print "Running"
        self.robot.stop()
        self.robot.reset_counters()
        # Read sensors until we're sure we getting clear readings
        while True:
            self.robot.get_sensors()
            print bin(self.robot.line_sensor)
            if self.robot.line_sensor == 0b11111:
                break
            time.sleep(0.01)

        # OK - lets go!
        self.scheduler.start()
        self.robot.set_speed(self.max_speed)

        while not self.scheduler.stopped:
            # Are we nearly there yet?
            self.robot.get_sensors()
            print bin(self.robot.line_sensor)
            if self.robot.line_sensor != 0b11111:
                print "STOP STOP STOP"
                self.scheduler.stopped = True
                self.robot.stop()
            time.sleep(0.01)

        self.robot.stop()
        self.robot.finish()


ipaddress = "10.5.5.1"
def main(argv):
    controller = None

    try:
        opts, args = getopt.getopt(argv,"i:ht3sjlwc")
    except getopt.GetoptError:
        print 'metabot.py -i <ip address> -[ht3sjlwc]'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'metabot.py -i <ip address> -[ht3sjlwc]'
            print '-h	help'
            print '-t	test'
            print '-3	three point turn'
            print '-s	speed control'
            print '-j	joystick control [default]'
            print '-l	line follower'
            print '-w	wall stop'
            print '-c	Calibration'
            sys.exit()
        elif opt in ("-i"):
            global ipaddress
            ipaddress = arg
        elif opt == '-t':
            controller = TestController()
        elif opt == '-3':
            controller = ThreePointTurnController()
        elif opt == '-s':
            controller = SpeedController()
        elif opt == '-j':
            controller = JoystickController()
        elif opt == '-l':
            controller = LineFollowerController()
        elif opt == '-w':
            controller = WallStop()
        elif opt == '-c':
            controller = CalibController()

    if controller is None:
        controller = JoystickController()

    controller.run()

if __name__ == "__main__":
   main(sys.argv[1:])

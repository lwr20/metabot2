import pygame
import socket
import json
import struct
import sys

HOST = '10.5.5.1'  # The IP address for the robot
PORT = 10000

# Define some colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)


# Class to write Joystick info to the screen.
class Screen:
    def __init__(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

        self.screen = pygame.display.set_mode((500, 700))
        pygame.display.set_caption("Metabot2 Controller")
        self.font = pygame.font.Font(None, 20)

    def myprint(self, textstring):
        textbitmap = self.font.render(textstring, True, BLACK)
        self.screen.blit(textbitmap, [self.x, self.y])
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10

    def update(self):
        self.screen.fill(WHITE)
        self.reset()

        # Get count of joysticks
        joystick_count = pygame.joystick.get_count()

        if sender.socket is None:
            self.myprint("Metabot not connected")

        self.myprint("Number of joysticks: {}".format(joystick_count))
        self.indent()

        # For each joystick:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()

            self.myprint("Joystick {}".format(i))
            self.indent()

            # Get the name from the OS for the controller/joystick
            name = joystick.get_name()
            self.myprint("Joystick name: {}".format(name))

            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            self.myprint("Number of axes: {}".format(axes))
            self.indent()
            for j in range(axes):
                axis = joystick.get_axis(j)
                self.myprint("Axis {} value: {:>6.3f}".format(j, axis))
            self.unindent()

            buttons = joystick.get_numbuttons()
            self.myprint("Number of buttons: {}".format(buttons))
            self.indent()

            for j in range(buttons):
                button = joystick.get_button(j)
                self.myprint("Button {:>2} value: {}".format(j, button))
            self.unindent()

            # Hat switch. All or nothing for direction, not like joysticks.
            # Value comes back in an array.
            hats = joystick.get_numhats()
            self.myprint("Number of hats: {}".format(hats))
            self.indent()

            for j in range(hats):
                hat = joystick.get_hat(j)
                self.myprint("Hat {} value: {}".format(j, str(hat)))

            self.unindent()
            self.unindent()

        if joystick_count == 0:
            # No joysticks, so use arrow keys on keyboard instead
            keys = pygame.key.get_pressed()
            self.myprint("Use Keyboard Arrows instead")
            self.indent()
            self.myprint("Up arrow : {}".format(keys[pygame.K_UP]))
            self.myprint("Down arrow : {}".format(keys[pygame.K_DOWN]))
            self.myprint("Left arrow : {}".format(keys[pygame.K_LEFT]))
            self.myprint("Right arrow : {}".format(keys[pygame.K_RIGHT]))
            self.myprint("Space : {}".format(keys[pygame.K_SPACE]))

        # Go ahead and update the screen with what we've drawn.
        pygame.display.flip()


class Sender:
    def __init__(self, bot_host, port):
        self.host = bot_host
        self.port = port
        self.connect()
        self.socket = None

    def connect(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(1.0)
        try:
            self.socket.connect((self.host, self.port))
        except socket.error:
            self.socket.close()
            self.socket = None

    def send(self, data):
        if self.socket is None:
            self.connect()
        if self.socket is None:
            return
        payload = json.dumps(data)
        length = len(payload)
        header = struct.pack("<L", length)
        try:
            self.socket.sendall("%s%s" % (header, payload))
        except socket.error:
            self.socket.close()
            self.socket = None

    def send_update(self, joystick):
        # create a dictionary and add the controller name
        simple_joy = {"controller": joystick.get_name()}

        # get all the stick data
        axes = joystick.get_numaxes()
        axis_list = []
        for i in range(axes):
            axis = joystick.get_axis(i)
            axis_list.append("{:>6.3f}".format(axis))
        simple_joy["sticks"] = axis_list

        # get all the button data
        buttons = joystick.get_numbuttons()
        button_list = []
        for i in range(buttons):
            button = joystick.get_button(i)
            button_list.append("{}".format(button))
        simple_joy["buttons"] = button_list

        # get all the hat data
        hat_list = []
        hats = joystick.get_numhats()
        for i in range(hats):
            hat = joystick.get_hat(i)
            hat_list.append("{}".format(str(hat)))
        simple_joy["hats"] = hat_list

        # send it off over the network
        self.send(simple_joy)

    def send_keys(self):
        # create a dictionary and add the controller name
        keys = {"controller": "keypad"}
        keystate = pygame.key.get_pressed()
        keys["K_UP"] = keystate[pygame.K_UP]
        keys["K_DOWN"] = keystate[pygame.K_DOWN]
        keys["K_LEFT"] = keystate[pygame.K_LEFT]
        keys["K_RIGHT"] = keystate[pygame.K_RIGHT]
        keys["K_SPACE"] = keystate[pygame.K_SPACE]
        self.send(keys)

# ==============================
# Main

# Check for a command line argument
if len(sys.argv) > 1:
    host = sys.argv[1]
else:
    host = HOST

# Set-up pygame
pygame.init()
clock = pygame.time.Clock()
pygame.joystick.init()

# Set-up pretty printer
screen = Screen()

# Create our network handler
sender = Sender(host, PORT)

controllerEvents = (pygame.JOYAXISMOTION,
                    pygame.JOYBALLMOTION,
                    pygame.JOYBUTTONDOWN,
                    pygame.JOYBUTTONUP,
                    pygame.JOYHATMOTION)

actionkeys = (pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_SPACE)

# -------- Main Program Loop -----------
# Loop until the user clicks the close button.
done = False
while not done:

    # EVENT PROCESSING STEP
    for event in pygame.event.get():  # Poll for an event
        if event.type == pygame.QUIT:  # If user clicked close
            done = True  # Flag that we are done so we exit this loop
        elif event.type in controllerEvents:
            sender.send_update(joystick=pygame.joystick.Joystick(event.dict['joy']))
        elif event.type in (pygame.KEYDOWN, pygame.KEYUP) and event.dict['key'] in actionkeys:
            sender.send_keys()

    # Write out info about the currently attached joysticks
    screen.update()

    # Limit to 20 frames per second
    clock.tick(20)

# Close the window and quit.
pygame.quit()

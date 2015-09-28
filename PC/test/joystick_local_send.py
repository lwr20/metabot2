import pygame, socket, json, struct

HOST = '127.0.0.1'    # The IP address for the robot
PORT = 10000

CONTROLLER = "XBOX 360"

# Define some colors
BLACK    = (   0,   0,   0)
WHITE    = ( 255, 255, 255)

# This is a simple class that will help us print to the screen
# It has nothing to do with the joysticks, just outputing the
# information.
class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def myprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
    
class Sender:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((host, port))
    def send(self, data):
        payload = json.dumps(data)
        length = len(payload)
        header = struct.pack("<L", length)
        self.socket.sendall("%s%s" % (header,payload))
        

pygame.init()
 
# Set the width and height of the screen [width,height]
size = [500, 700]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("My Game")

#Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates
clock = pygame.time.Clock()

# Initialize the joysticks
pygame.joystick.init()
    
# Get ready to print
textPrint = TextPrint()

# Create our network handler
sender = Sender(HOST, PORT)

def send_update(joystick):
    # create a dictionary to save our joystick data in
    simple_joy = {}     
    
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
        button = joystick.get_button( i )
        button_list.append("{}".format(button))
    simple_joy["buttons"]=button_list
    
    # get all the hat data
    hat_list = []
    hats = joystick.get_numhats()
    for i in range(hats):
        hat = joystick.get_hat( i )
        hat_list.append("{}".format(str(hat)))
    simple_joy["hats"]=hat_list
    
    # send it off over the network
    sender.send(simple_joy)

def get_controller(controller_name):
    """Returns the index of the desired controller"""
    joystick_count = pygame.joystick.get_count()
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        name = joystick.get_name()
        if controller_name in name:
            return i
    # We didn't find the desired controller
    return -1



controller_index = get_controller(CONTROLLER)
if controller_index == -1:
    print "Desired controller not found!"
    sys.exit(1)
    
# -------- Main Program Loop -----------
while done==False:
    # EVENT PROCESSING STEP
    for event in pygame.event.get(): # User did something
        if event.type == pygame.QUIT: # If user clicked close
            done=True # Flag that we are done so we exit this loop
        else:
            send_update(joystick = pygame.joystick.Joystick(controller_index))
        # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
        # if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
            # print("Joystick button changed.")
        # elif event.type == pygame.JOYAXISMOTION
            # print("Joystick button released.")
        
            
 
    # DRAWING STEP
    # First, clear the screen to white. Don't put other drawing commands
    # above this, or they will be erased with this command.
    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()

    textPrint.myprint(screen, "Number of joysticks: {}".format(joystick_count) )
    textPrint.indent()
    
    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
    
        textPrint.myprint(screen, "Joystick {}".format(i) )
        textPrint.indent()
    
        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        textPrint.myprint(screen, "Joystick name: {}".format(name) )
        
        # Don't bother with anything which isn't an XBOX controller
        if "XBOX 360" in name:
            # Usually axis run in pairs, up/down for one, and left/right for
            # the other.
            axes = joystick.get_numaxes()
            textPrint.myprint(screen, "Number of axes: {}".format(axes) )
            textPrint.indent()
            for i in range(axes):
                axis = joystick.get_axis(i)
                textPrint.myprint(screen, "Axis {} value: {:>6.3f}".format(i, axis) )
            textPrint.unindent()
            
            buttons = joystick.get_numbuttons()
            textPrint.myprint(screen, "Number of buttons: {}".format(buttons) )
            textPrint.indent()

            for i in range(buttons):
                button = joystick.get_button( i )
                textPrint.myprint(screen, "Button {:>2} value: {}".format(i,button) )
            textPrint.unindent()
            
            # Hat switch. All or nothing for direction, not like joysticks.
            # Value comes back in an array.
            hats = joystick.get_numhats()
            textPrint.myprint(screen, "Number of hats: {}".format(hats) )
            textPrint.indent()

            for i in range( hats ):
                hat = joystick.get_hat( i )
                textPrint.myprint(screen, "Hat {} value: {}".format(i, str(hat)) )
                
            textPrint.unindent()
            textPrint.unindent()
    
    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second
    clock.tick(20)
    
    
# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit ()
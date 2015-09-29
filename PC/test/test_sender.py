import socket, json, struct, time

HOST = '172.18.55.18'    # The IP address for the robot
PORT = 10000

    
class Sender:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((host, port))
    def send(self, data):
        payload = json.dumps(data)
        length = len(payload)
        header = struct.pack("<L", length)
        self.socket.sendall("%s%s" % (header,payload))
        

def set_speed(speed):
    simple_joy = {}
    speed_text = '%.3f' % speed
    sticks = ['0.000', '0.000', '0.000', speed_text,  '0.000']
    hats = ['(0, 0)']
    buttons = ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0']
    simple_joy['sticks'] = sticks
    simple_joy['hats'] = hats
    simple_joy['buttons'] = buttons
    print simple_joy
    sender.send(simple_joy)
 

# Create our network handler
sender = Sender(HOST, PORT)


#for i in range(60):
while True:
    try:
        print time.time()
        #set_speed(0.15+(i/240.0))
        set_speed(0)
        time.sleep(1)
    except:
        set_speed(0)
        print time.time()
        time.sleep(1)
        
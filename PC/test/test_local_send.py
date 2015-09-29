import socket, json, struct, time

HOST = '127.0.0.1'    # The IP address for the robot
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
        


# Create our network handler
sender = Sender(HOST, PORT)

simple_joy = {}

sticks = ['0.000', '0.000', '0.000', '0.000', '0.000']
hats = ['(0, 0)']
buttons = ['0', '0', '0', '0', '0', '0', '0', '0', '0', '0']

simple_joy['sticks'] = sticks
simple_joy['hats'] = hats
simple_joy['buttons'] = buttons

sticks = ['0.000', '0.000', '0.200', '0.000', '0.000']
simple_joy['sticks'] = sticks
sender.send(simple_joy)
time.sleep(10)
sticks = ['0.000', '0.000', '0.000', '0.000', '0.000']
simple_joy['sticks'] = sticks
sender.send(simple_joy)
time.sleep(1)
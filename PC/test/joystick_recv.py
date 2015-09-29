import socket, json, struct, pprint

HOST = '127.0.0.1'    # The IP address for the robot
PORT = 10000
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
pp = pprint.PrettyPrinter(indent=4)

while True:
    header = conn.recv(4)
    length = struct.unpack("<L", header)[0]
    payload = conn.recv(length)
    #print "Length:%s  Payload:%s" % (length, payload)
    simple_joy = json.loads(payload)
    pp.pprint(simple_joy)
    

import serial
import select
ser = serial.Serial('/dev/ttyACM0', 9600)
# port pdb;pdb.set_trace()
while 1:
    line = ser.readline()
    values = line.split()
    if len(values) == 5:





    select.select([ser],[],[])

def position(values):
    i = get_top_index(values)
    if i in [0, 4]:
        return i
    left = values[i+1]
    right = values[i-1]
    minval = min(values)
    if left > right:
        return i - ((values[i] - minval)/(left - minval))




def get_top_index(values):
    return values.index(max(values))

def print_graph(values):
        print(chr(27) + "[2J")
        print '|' * (int(values[0])/12)
        print '|' * (int(values[1])/12)
        print '|' * (int(values[2])/12)
        print '|' * (int(values[3])/12)
        print '|' * (int(values[4])/12)

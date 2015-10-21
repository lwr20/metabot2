import serial
import select
ser = serial.Serial('/dev/ttyACM0', 9600)
# port pdb;pdb.set_trace()
while 1:
    line = ser.readline()
    values = line.split()
    if len(values) == 5:
        print(chr(27) + "[2J")
        print '|' * (int(values[0])/12)
        print '|' * (int(values[1])/12)
        print '|' * (int(values[2])/12)
        print '|' * (int(values[3])/12)
        print '|' * (int(values[4])/12)
    select.select([ser],[],[])

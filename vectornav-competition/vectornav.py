import serial

def read_buffer(ser):
    string = ""
    while len(string) == 0:
        if(ser.read() == b'$'):
            string += '$'
    for i in range(0, 195):
        value = (ser.read()).decode('ascii')
        string += (value)
        if(value == '*'):
            value = (ser.read()).decode('ascii')
            string += (value)
            value = (ser.read()).decode('ascii')
            string += (value)
            break

    return string

def read_vectornav(ser):
    msg = "$VNINS"
    ser.write(msg.encode())
    buffer = read_buffer(ser)
    return buffer

if __name__ == '__main__':
    ser = serial.Serial(port, baudrate)
    ser.flushInput()
    print(read_vectornav(ser))

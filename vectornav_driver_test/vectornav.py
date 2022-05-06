import serial

def read_buffer(ser):
	string = ""
	while len(string) == 0:
		readString = ser.read().decode('ascii')
		if(readString == '$'):
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
	msg = "$VNGPS"
	ser.write(msg.encode())
	buffer = read_buffer(ser)
	return buffer

if __name__ == '__main__':
	port = input("Enter the port name: ")
	ser = serial.Serial(port, 115200)
	ser.flushInput()
	print(read_vectornav(ser))

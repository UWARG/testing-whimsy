import serial
from vectornav import read_vectornav
from xbeetest import send_xbee

baudrate_vnav = 115200
baudrate_xbee = 115200
port_vnav = r"/dev/ttyUSB1"

def main():
	ser_vnav = serial.Serial(port_vnav, baudrate_vnav)
	print(read_vectornav(ser_vnav))

if __name__ == '__main__':
	main()

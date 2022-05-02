import serial
from vectornav import read_vectornav
from xbeetest import send_xbee

baudrate_vnav = 115200
baudrate_xbee = 115200
#port_vnav = r"/dev/ttyUSB1"
port_vnav = r"COM13"

class vnav_data_parsed:
	def __init__(self) -> None:
		#define the data structure
		#mode, gnssfix, lat, lon, alt, speed, heading, roll, pitch, yaw
		self.mode = 0
		self.gnssfix = 0
		self.lat = 0
		self.lon = 0
		self.alt = 0
		self.speed = 0
		self.heading = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.byteObject = bytearray()

		self.CONST_STATUS_REG_STR = 4
		self.CONST_STATUS_REG_MODE_BYTE = 0
		self.CONST_STATUS_REG_GNSSFIX_BYTE = 1
		#self.CONST_STATUS_REG_GNSFIX_BIT_OFFSET = 
		self.CONST_YAW_STATUS_REG_STR = 5
		self.CONST_PITCH_STATUS_REG_STR = 6
		self.CONST_ROLL_STATUS_REG_STR = 7

	def parse_str(string:str):
		#parse the string for vnav 300:
		
		#generate list based off string split by commas
		string_list = string.split(",")

		#self.mode = 

	
def main():
	ser_vnav = serial.Serial(port_vnav, baudrate_vnav)
	print(read_vectornav(ser_vnav))

if __name__ == '__main__':
	main()

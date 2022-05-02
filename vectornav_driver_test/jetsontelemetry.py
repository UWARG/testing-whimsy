import serial
import struct
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
		self.status = 0
		self.lat = 0
		self.lon = 0
		self.alt = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.byteObject = bytearray()

		self.CONST_STATUS_REG_STR = 3
		self.CONST_YAW_STATUS_REG_STR = 4
		self.CONST_PITCH_STATUS_REG_STR = 5
		self.CONST_ROLL_STATUS_REG_STR = 6
		self.CONST_LAT_REG_STR = 7
		self.CONST_LONG_REG_STR = 8
		self.CONST_ALT_REG_STR = 9

	def parse_str(self, string:str):
		#parse the string for vnav 300:
		
		#generate list based off string split by commas
		string_list = string.split(",")
		print(string_list)

		self.status = int(string_list[self.CONST_STATUS_REG_STR].strip('+'))
		self.yaw = float(string_list[self.CONST_YAW_STATUS_REG_STR].strip('+'))
		self.pitch = float(string_list[self.CONST_PITCH_STATUS_REG_STR].strip('+'))
		self.roll = float(string_list[self.CONST_ROLL_STATUS_REG_STR].strip('+'))
		self.lat = float(string_list[self.CONST_LAT_REG_STR].strip('+'))
		self.long = float(string_list[self.CONST_LONG_REG_STR].strip('+'))
		self.alt = float(string_list[self.CONST_ALT_REG_STR].strip('+'))

		self.byteObject = struct.pack('>Iffffff', self.status, self.lat, self.lon, self.alt, self.roll, self.pitch, self.yaw)
		return self.byteObject
	
def main():
	ser_vnav = serial.Serial(port_vnav, baudrate_vnav)
	data = vnav_data_parsed()
	bytes = data.parse_str(read_vectornav(ser_vnav))

if __name__ == '__main__':
	main()

import serial
from vectornav import read_vectornav
from xbeetest import send_xbee
from devicePorts import getDevicePorts

baudrate_vnav = 115200
baudrate_xbee = 115200
port_vnav = r"/dev/ttyUSB1"

def main():

    vectorPort, xbeePort = getDevicePorts()
    print(vectorPort + " " + xbeePort)
    ser_vnav = serial.Serial(vectorPort, baudrate_vnav)
    ser_xbee = serial.Serial(xbeePort, baudrate_xbee)

    while True:

        vectorNavData = read_vectornav(ser_vnav)

        # convert the returned data to a bytes payload
        payload = list(bytearray(vectorNavData))

        send_xbee(ser_xbee, payload)

if __name__ == '__main__':
    main()

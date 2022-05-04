import time
import serial
from vectornav import read_vectornav
from xbeetest import send_xbee
from devicePorts import getDevicePorts

baudrate_vnav = 115200
baudrate_xbee = 115200
port_vnav = r"/dev/ttyUSB1"

def main():

    vectorPort, xbeePort = getDevicePorts()
    vectorPort = vectorPort.strip()
    xbeePort = xbeePort.strip()
    print(vectorPort + " " + xbeePort)
    ser_vnav = serial.Serial(vectorPort, baudrate_vnav)
    ser_xbee = serial.Serial(xbeePort, baudrate=115200, bytesize=8, timeout=20, stopbits=serial.STOPBITS_ONE)

    while True:

        vectorNavData = read_vectornav(ser_vnav)
        #print(vectorNavData)

        data = vectorNavData.split(",")
        #print(data)
        lat = data[5]
        lon = data[6]
        alt = data[7]

        goodData = lat + "," + lon + "," + alt
        print(goodData)

        payload = bytes(goodData,'utf-8')
        # payload = bytes("testfdsfdsafdsafdsfsdfsdfsdfsdfsdfsdfsdfdsfsdfdsfdsdddddddddddddddddddfff",'utf-8')
        #payload = vectorNavData.encode('utf-8')    
        #payload = [b for b in payload]
        #print(payload)

        # convert the returned data to a bytes payload
        # payload = list(bytearray(vectorNavData))

        send_xbee(ser_xbee,payload)
        time.sleep(0.1)

if __name__ == '__main__':
    main()

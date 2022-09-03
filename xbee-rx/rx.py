import numbers
import serial
import struct

HEADER_SIZE = 17

port = "COM7"#getXbeePort()
xbee = serial.Serial(port=port, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

payloadTypes = {
    1:36
}

index = 0

while(True):
    startingByte = xbee.read()
    if startingByte != b'\x00':
        print("byte read",startingByte.hex(),index)
    index += 1
    if startingByte == b'\x7e':
        print("<------------------- got starting byte ------------------->")
        lengthBytes = xbee.read(2)
        lengthMessage = int.from_bytes(lengthBytes, "big")

        print("the length is",lengthMessage)

        if lengthMessage < HEADER_SIZE:
            print("length too small exiting")
            xbee.read(lengthMessage)
            xbee.read(1) # checksum

        tfFrame = xbee.read(12)
        
        timeBytes = xbee.read(4)
        time = int.from_bytes(timeBytes, "little")
        print("time:",time)

        payloadType = xbee.read(1)
        print("payload type:", payloadType.hex())
        lengthPayload = payloadTypes[1]
        if lengthMessage - HEADER_SIZE != lengthPayload:
            print("length incorrect")
            print("payload length ",lengthPayload)
            print("message length left ", lengthMessage-HEADER_SIZE)
            continue
        
        if payloadType == b'\x01':
            # basic drone payload, GPS, IMU, motor outputs
            latBytes = xbee.read(4)
            lat = struct.unpack('<f',latBytes)
            longBytes = xbee.read(4)
            long = struct.unpack('<f',longBytes)
            altBytes = xbee.read(4)
            alt = struct.unpack('<f',altBytes)

            yawBytes = xbee.read(4)
            yaw = struct.unpack('<f',yawBytes)
            pitchBytes = xbee.read(4)
            pitch = struct.unpack('<f',pitchBytes)
            rollBytes = xbee.read(4)
            roll = struct.unpack('<f',rollBytes)

            print("lat:",lat[0])
            print("long:",long[0])
            print("alt:",alt[0])

            print("yaw:",yaw[0])
            print("pitch:",pitch[0])
            print("roll:",roll[0])

            motorOutputsBytes = xbee.read(12)
            motorOutputs = list(motorOutputsBytes)
            print(motorOutputs)

        checksum = xbee.read(1)
        print("checksum", checksum.hex())
        
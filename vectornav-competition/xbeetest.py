import serial

def generate_checksum(data) -> int:
    # checksum and length
    sumBytes = 0
    for i in range(3,len(data)):
        sumBytes += data[i]
    sumBytes = sumBytes & 0xFF #mask out upper 8 bytes
    checksum = 0xff - sumBytes
    return checksum #$VNINS,000910.627507,0000,0080,+020.984,+009.298,+086.211,+00.00000000,+000.00000000,+00000.000,+000.000,+000.000,+000.000,90.0,00.0,0.00*6C


def send_xbee(ser, payload):
    data = [0x7e, 0x00, 0x00, 0x10, 0x01, 0x00,0x13,0xa2,0x00,0x41,0xb1,0x6d,0x1c, 0xff, 0xfe, 0x00, 0x00]
    #payload = [b for b in payload]
    #print(payload)
    #print(len(payload))

    data += payload

    # checksum and length
    sumBytes = 0
    for i in range(3,len(data)):
        sumBytes += data[i]
    length = len(data) - 3
    length_arr = length.to_bytes(2, 'big')

    data[1] = length_arr[0]
    data[2] = length_arr[1]

    data.append(generate_checksum(data))

    #or b in data:
    #    print(hex(b))

    ser.write(bytearray(data))

#def test(ser):
 #   test_array = [0x7E, 0x00, 0x12, 0x10, 0x01, 0x00, 0x13, 0xA2, 0x00, 0x41, 0xB1, 0x6D, 0x1C, 0xFF, 0xFE, 00, 00, 0x74, 0x65, 0x73, 0x74, 0x01]
  #  packet = bytearray(test_array)
   # print(packet)
    #ser.write(packet)

if __name__ == '__main__':
    serialPort = serial.Serial(port="/dev/ttyUSB1", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    send_xbee(serialPort, bytes([0x10, 0x11]))

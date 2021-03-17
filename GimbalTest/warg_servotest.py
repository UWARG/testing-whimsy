import serial
import time
import keyboard

baudRate = 9600
COMPORT = "COM6"
serialCon = serial.Serial(COMPORT, baudRate) #Create connection on serial port

def writeIntToSerial(int):
    serialCon.write(str(int).encode('Latin-1'))

def checkIfPressed(key):
    if keyboard.is_pressed(key):
        return True
    else:
        return False


while True:
    if(checkIfPressed('W')):
        writeIntToSerial(1)
        print('W')

    elif(checkIfPressed('S')):
        writeIntToSerial(2)
        print('S')

    
    if(checkIfPressed('A')):
        writeIntToSerial(3)
        print('A')
    elif(checkIfPressed('D')):
        writeIntToSerial(4)
        print('D')

    print(serialCon.readline())
    print(serialCon.readline())
    
    #time.sleep(0.1)
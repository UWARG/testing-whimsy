import os 

def getDevicePorts():
    vectorPipe = os.popen('./device.sh 0403:6001')
    vectorPort = vectorPipe.read()
    vectorPipe.close()
    
    xbeePipe = os.popen('./device.sh 0403:6015')
    xbeePort = xbeePipe.read()
    xbeePipe.close()
    
    # print(vectorPort)
    # print(xbeePort)
    return [vectorPort, xbeePort]

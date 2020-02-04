from networktables import NetworkTables
connected = False

def connectionListener(connected, info):
        print(info, '; Connected=%s' % connected)

def sendTheta(x):
    sd.putNumber('targetAngle', x)

NetworkTables.initialize(server='10.33.24.49')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
sd = NetworkTables.getTable("ShuffleBoard")

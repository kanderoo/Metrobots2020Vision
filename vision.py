import cv2
import numpy as np
import os
import json
from networktables import NetworkTables
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

connected = False

def connectionListener(connected, info):
    print(info, '; Connection=%s ' % connected)

def sendTheta(x):
    sd.putNumber('targetAngle', x)
    print(x)

NetworkTables.initialize(server='10.33.24.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
sd = NetworkTables.getTable("SmartDashboard")

def onChange(x):
    pass

# define consts
WIDTH = 640
HEIGHT = 360

# init camera
port = 0
os.system('v4l2-ctl -d /dev/video' + str(port) + ' -c exposure_auto=1')
cap = cv2.VideoCapture(port)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)

# get camera config
with open("/boot/frc.json", "rt", encoding="utf-8") as f:
    j = json.load(f)

print(j["cameras"][0])

# init camera server
inst = CameraServer.getInstance()
camera = UsbCamera("Camera", "/dev/video"+str(port))
server = inst.startAutomaticCapture(camera=camera, return_server=True)
camera.setConfigJson(json.dumps(j["cameras"][0]))
camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

while True:
    rem, frame = cap.read()
    if frame is None:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_hsv = np.array([60, 150, 80])
    upper_hsv = np.array([115, 255, 255])

    # image manipulation
    os.system('v4l2-ctl -d /dev/video'+str(port)+' -c exposure_absolute=0')
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    erosion = cv2.erode(mask, kernel, iterations=2)
    dilation = cv2.dilate(erosion, kernel, iterations=2)

    # contour detection and display
    im2, contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, 0, (0,255,0), 3)
    try:
        x, y, w, h = cv2.boundingRect(contours[0])
        centerX = x + (w / 2)
        centerY = y + (h / 2)
        targetAngle = math.degrees(math.atan((centerX - WIDTH/2)/792)) # Function gotten from r=f*tan(theta)
        sendTheta(targetAngle)
    except:
        sendTheta(-1000);

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

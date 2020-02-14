import cv2
import numpy as np
import os
import json
import math
from networktables import NetworkTables
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer

connected = False

def connectionListener(connected, info):
    print(info, '; Connection=%s ' % connected)

def sendData(name, x):
    sd.putNumber(name, x)
    print(name, ": ", x)

NetworkTables.initialize(server='10.33.24.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
sd = NetworkTables.getTable("SmartDashboard")

def onChange(x):
    pass

# define consts
WIDTH = 320
HEIGHT = 180

TARGET_HEIGHT = 7.5625 # target height in feet (height of target center)
CAMERA_ANGLE = math.radians(60) # camera angle in degrees, converted to radians
CAMERA_HEIGHT = 8.5/12 # camera height in feet (5.5 in)
VERTICAL_FOV = 80.7
HORIZONTAL_FOV = 120
FOCAL_LENGTH = WIDTH/(2*math.tan(math.radians(HORIZONTAL_FOV/2)))

# init camera
port = 0
#os.system('v4l2-ctl -d /dev/video' + str(port) + ' -c exposure_auto=1')
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

    lower_hsv = np.array([68, 98, 129])
    upper_hsv = np.array([89, 255, 255])

    # image manipulation
    #os.system('v4l2-ctl -d /dev/video'+str(port)+' -c exposure_absolute=0')
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    erosion = cv2.erode(mask, kernel, iterations=2)
    dilation = cv2.dilate(erosion, kernel, iterations=2)

    # contour detection and display
    im2, contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, 0, (0,255,0), 3)
    try:
        # horiz angle
        x, y, w, h = cv2.boundingRect(contours[0])
        centerX = x + (w / 2)
        centerY = y + (h / 2)
        targetAngle = math.degrees(math.atan((centerX - WIDTH/2)/792)) # Function gotten from r=f*tan(theta)
        sendData("Horizontal Target Angle", targetAngle)

        # distance calculation
        referencePixel = (HEIGHT/2)-centerY
        a2 = math.radians(referencePixel*(VERTICAL_FOV/HEIGHT))
        totalAngle = CAMERA_ANGLE + a2
        distance = (TARGET_HEIGHT-CAMERA_HEIGHT)/math.tan(totalAngle)
        a1 = math.atan((TARGET_HEIGHT-CAMERA_HEIGHT) / 10) - a2 # calculates a1 from initiation line

        sendData("Distance to Target", distance)
    except:
        sendData("Horizontal Target Angle", 0)
        sendData("Distance to Target", 0)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

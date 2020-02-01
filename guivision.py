import cv2
import numpy as np
import os
import VisionTables
import math

def onChange(x):
    pass

WIDTH = 1280
HEIGHT = 720

port = int(input("Camera port: "))
os.system('v4l2-ctl -d /dev/video' + str(port) + ' -c exposure_auto=1')
cap = cv2.VideoCapture(port)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)

cv2.namedWindow('sliders')
cv2.createTrackbar('hL', 'sliders', 0, 255, onChange)
cv2.createTrackbar('hU', 'sliders', 0, 255, onChange)
cv2.createTrackbar('sL', 'sliders', 0, 255, onChange)
cv2.createTrackbar('sU', 'sliders', 0, 255, onChange)
cv2.createTrackbar('vL', 'sliders', 0, 255, onChange)
cv2.createTrackbar('vU', 'sliders', 0, 255, onChange)
cv2.createTrackbar('erosion', 'sliders', 0, 50, onChange)
cv2.createTrackbar('dilation', 'sliders', 0, 50, onChange)
cv2.createTrackbar('exposure', 'sliders', -10, 10, onChange)

while True:
    rem, frame = cap.read()
    if frame is None:
        break
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_hsv = np.array([0, 0, 0])
    upper_hsv = np.array([255, 255, 255])

    # get slider values
    lower_hsv[(0)] = cv2.getTrackbarPos('hL', 'sliders')
    lower_hsv[(1)] = cv2.getTrackbarPos('sL', 'sliders')
    lower_hsv[(2)] = cv2.getTrackbarPos('vL', 'sliders')
    upper_hsv[(0)] = cv2.getTrackbarPos('hU', 'sliders')
    upper_hsv[(1)] = cv2.getTrackbarPos('sU', 'sliders')
    upper_hsv[(2)] = cv2.getTrackbarPos('vU', 'sliders')
    erodeAmount = cv2.getTrackbarPos('erosion', 'sliders')
    dilateAmount = cv2.getTrackbarPos('dilation', 'sliders')
    exposureAmount = cv2.getTrackbarPos('exposure', 'sliders')

    # image manipulation
    os.system('v4l2-ctl -d /dev/video'+str(port)+' -c exposure_absolute='+str(exposureAmount))


    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    erosion = cv2.erode(mask, kernel, iterations=erodeAmount)
    dilation = cv2.dilate(erosion, kernel, iterations=dilateAmount)

    # contour detection and display
    contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, 0, (0,255,0), 3)
    try:
        x, y, w, h = cv2.boundingRect(contours[0])
        centerX = x + (w / 2)
        centerY = y + (h / 2)
        targetAngle = math.degrees(math.atan((centerX - WIDTH/2)/792))
        print(targetAngle)
        VisionTables.sendTheta(targetAngle)
    except:
        print("No contours")
 

    cv2.imshow('sliders', frame)
    cv2.imshow('mask', dilation)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

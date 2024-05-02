import cv2
import numpy as np

def empty(a):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow("Trackbar")
cv2.resizeWindow("Trackbar",600,300)
cv2.createTrackbar("hue_min","Trackbar",54,255,empty)   
cv2.createTrackbar("hue_max","Trackbar",143,255,empty)
cv2.createTrackbar("sat_min","Trackbar",74,255,empty)
cv2.createTrackbar("sat_max","Trackbar",255,255,empty)
cv2.createTrackbar("val_min","Trackbar",154,255,empty)
cv2.createTrackbar("val_max","Trackbar",217,255,empty)

# 75, 255
# 150, 255
# 62, 255
# 233, 255
# 165, 255
# 255, 255

while True:
    ret, frame = cap.read()
    image_contour = frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("hue_min", "Trackbar")
    h_max = cv2.getTrackbarPos("hue_max", "Trackbar")
    s_min = cv2.getTrackbarPos("sat_min", "Trackbar")
    s_max = cv2.getTrackbarPos("sat_max", "Trackbar")
    v_min = cv2.getTrackbarPos("val_min", "Trackbar")
    v_max = cv2.getTrackbarPos("val_max", "Trackbar")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)

    cnts, hie = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for c in cnts:
        area = cv2.contourArea(c)
        if area > 300:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0),2)
            cv2.drawContours(frame, c, -1, (0,255,0), 2)
            cv2.putText(frame, "credencial", (x + w + 10, y + h + 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)

    cv2.imshow("Frame", frame)

    k = cv2.waitKey(1)
    if(k==ord('q')):
        break

cv2.destroyAllWindows()
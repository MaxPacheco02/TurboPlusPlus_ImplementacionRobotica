import numpy as np
import cv2 as cv

#1280 × 960

def select_point(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        global selected_points
        global imagepoints
        global imagepoints2D
        global c
        selected_points.append((x, y))
        arr = np.array([[x, y]], np.float32)
        if(c == 1):
            imagepoints2D = np.append(imagepoints2D, arr, axis=0)
            cv.circle(image2D, (x, y), 5, (0, 255, 255), -1) 
            cv.imshow("Image", image2D)
        else:
            imagepoints = np.append(imagepoints, arr, axis=0)
            cv.circle(image, (x, y), 5, (0, 255, 255), -1) 
            cv.imshow("Image3D", image)
        #print("Selected points:", selected_points)

image2D = cv.imread("./CRED_2D_2.jpeg")
image = cv.imread("./CRED_RIG.jpeg")

c = 1

objpoints2D=np.array([[0,0,0],[0,5.4,0],[8.5,5.4,0],
                    [8.5,0,0],[0.6,2.55,0],[0.6,3.55,0],
                    [7.85,3.55,0],[7.85,2.55,0]], np.float32) 

objpoints=np.array([[8.5,0,5.4],[0,0,5.4],
                    [0,8.5,5.4],[0,8.5,0],
                    [0,5.4,0],[8.5,5.4,0],
                    [8.5,0,0],[0,0,0]], np.float32)

imagepoints2D = np.zeros((0,2), np.float32)
imagepoints = np.zeros((0,2), np.float32)

img_size2D = image2D.shape[:2]
img_size = image.shape[:2]

objp = []
imgp = []

# mtx = np.array([[1,0,1],
#                 [0,1,1],
#                 [0,0,1]], np.float64)

# Initialize list to store selected points
selected_points = []
cv.namedWindow("Image")

while True:
    cv.imshow("Image", image2D)
    # Set mouse callback function to handle clicks
    cv.setMouseCallback("Image", select_point)
    k = cv.waitKey(1)
    if(k==ord('q')):
        break

cv.destroyAllWindows()

#objpoints = objpoints.T.reshape(-1,3)
#imagepoints = imagepoints.T.reshape(-1,2)

objp.append(objpoints2D)
imgp.append(imagepoints2D)

print(objp)
print("\n")
print(imgp)
print(img_size2D[::-1])


ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objp, imgp, img_size2D[::-1], None, None, None)

print("mtx 2D: \n",mtx)
print("dist 2D: \n", dist)
print("rvecs 2D: ", rvecs)
print("tvecs 2D: ", tvecs)

#-----------------------------------------------------------------------------------------

c = 0

cv.namedWindow("Image3D")

selected_points.clear()
objp.clear()
imgp.clear()

while True:
    cv.imshow("Image3D", image)
    # Set mouse callback function to handle clicks
    cv.setMouseCallback("Image3D", select_point)
    k = cv.waitKey(1)
    if(k==ord('q')):
        break

objp.append(objpoints)
imgp.append(imagepoints)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objp, imgp, img_size[::-1], mtx, None, None, None, flags=cv.CALIB_USE_INTRINSIC_GUESS)

print("\nCamera matrix : \n", mtx)
print("dist: \n", dist)
print("rvecs: ", rvecs)
print("tvecs: ", tvecs)

cv.destroyAllWindows()

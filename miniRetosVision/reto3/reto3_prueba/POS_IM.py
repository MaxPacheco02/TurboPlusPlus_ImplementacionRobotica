
#CAPTURAR IMAGENES DE CALIBRACION

import cv2

IMG = cv2.VideoCapture(0)

num = 0

while IMG.isOpened():
    
    ret, frame = IMG.read()
    
    k = cv2.waitKey(1)
    
    if k == ord('p'): 
        cv2.imwrite('./chessboard/chess' + str(num) + '.png', frame)
        print("images saved!")
        num += 1
    elif k == ord('q'):
        break
    
    cv2.imshow('Image',frame)

IMG.release()
cv2.destroyAllWindows()
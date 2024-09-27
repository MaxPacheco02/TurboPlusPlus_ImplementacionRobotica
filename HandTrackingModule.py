import cv2
import mediapipe as mp
import time


class handDetector():
    def __init__(self,mode =False,maxHands = 2, modelComplex = 1,detectionCon = 0.5, trackCon = 0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.modelComplex = modelComplex
        self.detectionCon = detectionCon
        self.trackCon = trackCon
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode,self.maxHands,self.modelComplex,self.detectionCon,self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils
    def findHands (self,img,draw = True) :
        imgRGB = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        #print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:    
                    self.mpDraw.draw_landmarks(img, handLms,self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self,img,handNo=0,draw =True):
        lnList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]
            for id,lm in enumerate(myHand.landmark):
                    #print(id,lm)
                    h,w,c = img.shape
                    cx,cy = int(lm.x*w),int(lm.y*h)
                    #print(id , cx , cy)
                    lnList.append([id,cx,cy])
                    #if id == 4:
                    cv2.circle(img,(cx,cy),5,(255,0,0),cv2.FILLED)
        return lnList
    def fingersUp(self):
        fingers = []
        #thumb detec
        if self.lnList[self.tipIds[0]][1] < self.lnList[self.tipIds[0]-1][1]:
            fingers.append(1)
        else:
            fingers.append(0)
        # Rest hand fingers
        for id in range(1,5):
            if self.lnList[self.tipIds[id]][2] < self.lnList[self.tipIds[id]-2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        return fingers

def main():
    pTime = 0
    cTime = 0
    cap = cv2.VideoCapture(0)
    detector = handDetector()
    while True:
        success,img = cap.read()
        img = detector.findHands(img)

        lnList = detector.findPosition(img)
        if len(lnList) !=0:
            print(lnList[4])

        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime

        cv2.putText(img,str(int(fps)),(10,70),cv2.FONT_HERSHEY_PLAIN,3,(255,0,255),3)
        
        cv2.imshow("Image",img)
        cv2.waitKey(1)  




if __name__ == "__main__":
    main()
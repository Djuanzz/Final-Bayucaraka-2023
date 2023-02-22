import rospy
import cv2 as cv
import cv_bridge
import math
import mediapipe as mp
from std_msgs.msg import String
from bayucaraka.msg import poss   

cap = cv.VideoCapture(0)
mpHands = mp.solutions.hands
hands = mpHands.Hands()
mpDraw = mp.solutions.drawing_utils
fingerCode = [4, 8, 12, 16, 20]
velDrone = 0

def pitagoras(a, b):
    return math.sqrt(a*a + b*b)

def findHands(img):
    imgRgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    res = hands.process(imgRgb)
    # menggambar landmark tangan
    if res.multi_hand_landmarks:
        for lms in res.multi_hand_landmarks:
            mpDraw.draw_landmarks(img, lms, mpHands.HAND_CONNECTIONS)
    return img

def findPos(img, handNo=0):
    lmList = []
    imgRgb = cv.cvtColor(img, cv.COLOR_BGR2RGB)
    res = hands.process(imgRgb)   
    if res.multi_hand_landmarks:
        # mencari koordinat dan id dari landmarks tangan
        myHand = res.multi_hand_landmarks[handNo]
        for id, lm in enumerate(myHand.landmark):
            h, w, c = img.shape
            cx, cy = int(lm.x * w), int(lm.y * h)
            lmList.append([id, cx, cy])
    return lmList

def findDis (img, tip1, tip2, lmList):
    x1, y1 = lmList[tip1][1], lmList[tip1][2]
    x2, y2 = lmList[tip2][1], lmList[tip2][2]

    cv.circle(img,(x1,y1), 5, (0,0,0), cv.FILLED)
    cv.circle(img,(x2,y2), 5, (0,0,0), cv.FILLED)
    cv.line(img, (x1,y1), (x2,y2), (0,0,0), 3)
    
    # mencari jarak antara 2 buah jari
    length = pitagoras((x2-x1), (y2-y1))
    smooth = 10
    velocity = smooth * round(length/smooth)
    velocity = int(velocity/40)
    return velocity, img

def talking():
    pub = rospy.Publisher("finalBayucaraka", poss, queue_size=10)
    rospy.init_node("mengFinger", anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        success, img = cap.read()
        # flip image
        img = cv.flip(img, 1)
        img = cv.resize(img,(800,600))
        img = findHands(img)
        lmList = findPos(img)
        msg = poss()
        fingers = []
        if len(lmList) != 0:
            # thumb
            if lmList[4][1] < lmList[3][1]:
                # opened
                fingers.append(1)
            else:
                # closed
                fingers.append(0)

            # fingers
            for id in range(1,5):
                if lmList[fingerCode[id]][2] < lmList[fingerCode[id]-2][2]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            # publish x y middle finger
            msg.pos_x = (lmList[12][1] - 400)/20
            msg.pos_y = (lmList[12][2] - 300)/-20

        if(fingers == [1,1,0,0,0]):
            global velDrone
            # find distace between thump and index finger
            velDrone, img = findDis(img, 4, 8, lmList)
            # publish status drone
            msg.status = 6
        else:     
            totalFinger = fingers.count(1)
            # publish status drone
            msg.status = totalFinger

        statusDrone = msg.status
        # publish velocity for drone
        msg.vel_res = velDrone

        # make a circle in center of image
        cv.circle(img,(400,300),5,[150,0,0],cv.FILLED)
        # show velocity and status
        if statusDrone == 6:
            cv.putText(img,f"status : change velocity",(20,30), cv.FONT_HERSHEY_DUPLEX,1,[0,0,0],2)
        else:
            cv.putText(img,f"status : {statusDrone}",(20,30), cv.FONT_HERSHEY_DUPLEX,1,[0,0,0],2)
        cv.putText(img,f"velocity : {velDrone}",(20,60), cv.FONT_HERSHEY_DUPLEX,1,[0,0,0],2)
        
        cv.imshow("image", img)
        pub.publish(msg)
        rate.sleep()
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    try:
        talking()
    except rospy.ROSInterruptException:
        pass
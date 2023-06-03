from pyfirmata import Arduino,SERVO,util
from time import sleep
import numpy as np
import cv2
board = Arduino('/dev/ttyACM0')
webcam = cv2.VideoCapture(2)
def goToContainer():
    pin_elbow = 11
    board.digital[pin_elbow].mode = SERVO
    for i in range(10,180):
        rotateServo(pin_elbow, i)
def rotateServo(pin,angle):
    board.digital[pin].write(angle)
    sleep(0.005)
def closeGripper():
    pin_gripper = 9
    board.digital[pin_gripper].mode = SERVO
    for i in reversed(range(140 + 1)):
        rotateServo(pin_gripper,i)
def openGripper():
    pin_gripper = 9
    board.digital[pin_gripper].mode = SERVO
    for i in range (30,150):
        rotateServo(pin_gripper,i)

def gohome1():
    print("Green container")
    pin_elbow = 11
    board.digital[pin_elbow].mode = SERVO
    for i in range(140, 105, -1):
        rotateServo(pin_elbow,i)
    pin_base = 10
    board.digital[pin_base].mode = SERVO
    sleep(1.2)
    for i in range(180,120,-1):
        rotateServo(pin_elbow,i)
    openGripper()
    sleep(0.8)
    for i in range(0, 180):
        rotateServo(pin_base, i)

def gohome2():
    print("Red container")
    pin_elbow = 11
    board.digital[pin_elbow].mode = SERVO
    for i in range(150, 100, -1):
        rotateServo(pin_elbow,i)
    pin_base = 10
    board.digital[pin_base].mode = SERVO
    sleep(1.2)
    for i in range(140,90,-1):
        rotateServo(pin_elbow,i)
    openGripper()
    sleep(0.8)
    for i in range(0, 180):
        rotateServo(pin_base, i)

def gohome3():
    print("Blue container")
    pin_elbow = 11
    board.digital[pin_elbow].mode = SERVO
    for i in range(150, 105, -1):
        rotateServo(pin_elbow,i)
    pin_base = 10
    board.digital[pin_base].mode = SERVO
    sleep(1.2)
    for i in range(180,95,-1):
        rotateServo(pin_elbow,i)
    openGripper()
    sleep(0.8)
    for i in range(40, 180):
        rotateServo(pin_base, i)


while (1):

    _, imageFrame = webcam.read()
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)

    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    green_lower = np.array([25, 52, 72], np.uint8)
    green_upper = np.array([102, 255, 255], np.uint8)
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

    kernel = np.ones((5, 5), "uint8")

    red_mask = cv2.dilate(red_mask, kernel)
    res_red = cv2.bitwise_and(imageFrame, imageFrame,
                                  mask=red_mask)

    green_mask = cv2.dilate(green_mask, kernel)
    res_green = cv2.bitwise_and(imageFrame, imageFrame,
                                    mask=green_mask)

    blue_mask = cv2.dilate(blue_mask, kernel)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                                   mask=blue_mask)

    contours, hierarchy = cv2.findContours(red_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                           (x + w, y + h),
                                           (0, 0, 255), 2)

            cv2.putText(imageFrame, "Red", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (0, 0, 255))
            openGripper()
            goToContainer()
            closeGripper()
            gohome2()
            sleep(5)
    contours, hierarchy = cv2.findContours(green_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                           (x + w, y + h),
                                           (0, 255, 0), 2)
            cv2.putText(imageFrame, "Green", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 255, 0))
            openGripper()
            goToContainer()
            closeGripper()
            gohome1()
            sleep(5)

    contours, hierarchy = cv2.findContours(blue_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                           (x + w, y + h),
                                           (255, 0, 0), 2)
            cv2.putText(imageFrame, "Blue", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0))
            openGripper()
            goToContainer()
            closeGripper()
            gohome3()
            sleep(5)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
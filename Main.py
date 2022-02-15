import cv2
from djitellopy import Tello
import numpy as np
import time

width, height = 360, 240
Forward_Backward_Range = [3000, 5000]
pid = [0.4,0.4,0]
pError = 0
drone = Tello()
drone.connect()

print(drone.get_battery())
battery = (drone.get_battery())  # We want to learn battery from drone
if battery > 50:
    drone.streamon()  # Turn on video streaming
    drone.takeoff()
    drone.move_up(30)
else:
    print("please charge it")


# uses open cv2 with the face cascade provided to recognize the face
def Face_Detection(img):
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    Detect = faceCascade.detectMultiScale(imgGray,1.2, 8)

    CenterList = []
    AreaList = []

    for (x, y, w, h) in Detect:
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0),5)
        cX = x + w // 2  # Center X of Rectangle from Haarcascade info
        cY = y + h // 2  # Center Y of Rectangle from Haarcascade info
        area = w * h
        cv2.circle(img, (cX, cY), 4, (0, 0, 255), cv2.FILLED)  # According to BGR we create red circle radius=4
        CenterList.append([cX, cY])
        AreaList.append(area)
        print(type(CenterList))
        print(type(AreaList))
        print(area)
        print(w,h)
    if len(AreaList) != 0:
        i = AreaList.index(max(AreaList))
        return img, [CenterList[i], AreaList[i]]
    else:
        return img, [[0, 0], 0]


def trackFace(faceInfo,w,pid,pError):
    area = faceInfo[1]
    x, y = faceInfo[0]
    fb = 0
    error_x = x - w // 2  # w= width; finding center of face
    speed_x = pid[0] * error_x + pid[1] * (error_x - pError)   # determining the yaw
    speed_x = int(np.clip(speed_x, -100, 100))


    if area > Forward_Backward_Range[0] and area < Forward_Backward_Range[1]:
        fb = 0  #Speed_x değerinde yaw haraketi yaparak yüzü arıyor.

    elif area > Forward_Backward_Range[1]:
        fb = -25
        #Yüz yakınlaştıkça geri gidiyor

    elif area < Forward_Backward_Range[0] and area != 0:  # If the drone is too far get closer
        fb = 25
        # Yüz uzaklaştıkça ileri gidiyor

    elif area ==0:
       drone.rotate_clockwise(20)
       time.sleep(0.8)



    if x == 0:
        speed_x = 0
        error_x = 0
    drone.send_rc_control(0, fb, 0, speed_x)
    print(speed_x, fb)
    return error_x


while True:
    frame = drone.get_frame_read().frame
    img = cv2.resize(frame, (width, height))


    img,info = Face_Detection(img)
    pError = trackFace(info,width,pid,pError)

    cv2.imshow("Drone Video:", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        drone.land()
        break


cv2.destroyAllWindows()
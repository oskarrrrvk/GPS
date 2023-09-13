from serial import Serial
from serial import serialutil
from threading import Thread
import math
import cv2
import time



EC_RADIUS = 6378137
ECCENTRICITY = 0.00669437999013
PORADIUS = math.sqrt(EC_RADIUS**2 / (ECCENTRICITY + 1))
HUSO = 30
LAMBDA0G = HUSO * 6 - 183
LAMBDA0 = LAMBDA0G * (math.pi/180)
E = ECCENTRICITY/(1 - ECCENTRICITY)

IMG = cv2.imread("imagenMapa.png", cv2.IMREAD_COLOR)
Y,X,_ = list(IMG.shape)

ESQ_SUP_IZQ =  [446099.00, 4471072.00]
ESQ_INF_DCHA = [446652.00, 4470685.00]
DISTANCIA_X = abs(ESQ_INF_DCHA[0] - ESQ_SUP_IZQ[0])
DISTANCIA_Y = abs(ESQ_INF_DCHA[1] - ESQ_SUP_IZQ[1])

insiaMap = []
coords_glob = []
time_glob = 000000.00
loop = True

def transformMinutes(data):
    grades = float (data) // 100
    minutes = float (data) % 100
    return grades + minutes / 60

def takeN(latitude):
    return EC_RADIUS/math.sqrt(1 - ECCENTRICITY * (math.sin(latitude) ** 2))

def takeT(latitude):
    return math.tan(latitude) ** 2

def takeC(latitude):
    return E * (math.cos(latitude) ** 2)

def takeA(latitude, length):
    return math.cos(latitude) * (length - LAMBDA0)

def takeM(latitude):
    return EC_RADIUS * ((1 - ECCENTRICITY/4 - 3/64 * (ECCENTRICITY ** 2) - 5/256 * (ECCENTRICITY ** 3))* latitude -
                       (3/8 * ECCENTRICITY + 3/32 * (ECCENTRICITY ** 2) + 45/1024 * (ECCENTRICITY ** 3)) * math.sin(2 * latitude) +
                       (15/256 * (ECCENTRICITY ** 2) + 45/1024 * (ECCENTRICITY ** 3)) * math.sin(4 * latitude) -
                       (35/3072 * (ECCENTRICITY ** 3))* math.sin(6 * latitude))

def  calculateUTM_Easting(N,T,C,A):
    return 0.9996 * N * (A + (1 - T + C) * (A ** 3)/6 + (5 - 18 * T + (T ** 2) + 72 * C - 58 * ECCENTRICITY) * (A ** 5)/120) + 500000


def calculateUTM_Northing(latitude,N,T,C,A,M):
    return 0.9996 * (M + N * math.tan(latitude) * ( (A ** 2) / 2 + (5 - T + 9 * C + 4 * (C ** 2)) * (A ** 4) / 24 + 
                                                   (61 - 58 * T + (T ** 2) + 600 * C - 330 * ECCENTRICITY) * (A ** 6) / 720))


def calculateCartesianCoordinate(latitude,length):
    N,T,C,A,M = takeN(latitude),takeT(latitude),takeC(latitude),takeA(latitude,length),takeM(latitude)
    return [calculateUTM_Easting(N,T,C,A),calculateUTM_Northing(latitude,N,T,C,A,M)]

def transformPixelsX(coord):
    return abs(coord - ESQ_SUP_IZQ[0]) * X / DISTANCIA_X

def transformPixelsY(coord):
    return abs(coord - ESQ_SUP_IZQ[1]) * Y / DISTANCIA_Y

def showResult(list,latitude,length,coordinates):
    print(list,"\ngrades: [",latitude,",",length,"]\nUTM: ",coordinates,"\n")

def getCoordMap():
    with open("Mapa_INSIA2.txt","r") as file:
        lines = file.readlines()
        lines = [line.rstrip() for line in lines]

    for line in lines:
        insiaMap.append(line.split("\t"))

def printSpeed(color,speed):
    cv2.rectangle(IMG,[1303,837],[1474,936],color,thickness=cv2.FILLED)
    cv2.rectangle(IMG,[1303,837],[1474,936],(0,0,0),thickness=2)
    cv2.putText(IMG,str(speed)+" Km/h",[1315,900],cv2.FONT_HERSHEY_SIMPLEX, 1 ,color=(0,0,0),thickness=2)

def takeHour(t):
    h = int(t // 10000)
    return h * 3600

def takeMin(t):
    m  = int((t % 10000) //100)
    return m * 60

def takeSec(t):
    return t % 100

def getTimeHours(t):
    return (takeHour(t) + takeMin(t) + takeSec(t))

def calculateDistance(c1,c2):
    return math.sqrt((c2[0] - c1[0])**2 + (c2[1] - c1[1])**2)

def calculateSpeed(d,t):
    return (d / t) * 3.6 #1km/1000m * 3600s/1h

def findSpeed(coord):
    distance = 1000000000000
    for coords in insiaMap:
        dist = calculateDistance([float(coords[1]),float(coords[0])],coord)
        if  dist < distance:
            distance = dist
            speed = int(coords[2])
    return speed

def adviseSpeed(speed,coords):
    speedLimit = findSpeed(coords)
    speedLimitH = speedLimit + speedLimit * 0.1
    speedLimitL = speedLimit - speedLimit * 0.1
    if speed < speedLimitH and speed > speedLimitL:
        printSpeed((0,255,0),speedLimit)
    elif speed >= speedLimitH:
        printSpeed((0,0,255),speedLimit)
    else:
        printSpeed((0,255,255),speedLimit)

def defineCoordinates(data):
    global coords_glob
    global time_glob

    list = data.split(",")
    time_glob = float(list[1])
    latitude,_,length,dirLength = list[2:6]
    latitude,length = transformMinutes(latitude),transformMinutes(length)
    if 'w' in dirLength.lower():
        length = -length
    coords_glob =  calculateCartesianCoordinate(math.radians(latitude),math.radians(length))
    return latitude,length, list

def ReadCoordinates():
    global loop
    try:
        port = Serial('COM6',4800,8,stopbits=1)
        while loop:
            data = port.readline().decode("ascii")
            if "GGA" in data:
                latitude,length,List = defineCoordinates(data)
                showResult(List,latitude,length,coords_glob)
                time.sleep(0.1)
    except serialutil.SerialException:
        print("the port cannnot be opened!!!")
        loop = False
        

def writeCoordinates():
    global coords_glob
    while loop:
        if len(coords_glob) > 0:
            x, y = round(transformPixelsX(coords_glob[0])), round(transformPixelsY(coords_glob[1]))
            b,g,r = IMG[x][y]

            cv2.circle(IMG,(x,y),2,(0,0,255),-1)
            cv2.circle(IMG,(x,y),2,(int(b),int(g),int(r)),-1)
            time.sleep(0.1)

def controlSpeed():
    t = time_glob
    while loop:
        if  t != time_glob:
            coord = coords_glob
            time.sleep(1)
            t,d = getTimeHours(time_glob) - getTimeHours(t), calculateDistance(coord,coords_glob)
            speed = calculateSpeed(d,t)
            adviseSpeed(speed,coord)
            t = time_glob

def showCoordinates():
    global loop
    while loop:
        resize = cv2.resize(IMG,(1300,800))
        cv2.imshow("mapa",resize)
        if(cv2.waitKey(1) == 27):
            loop = False
        time.sleep(0.1)



getCoordMap()

readTh = Thread(target=ReadCoordinates)
writeTh = Thread(target=writeCoordinates)
ctrlTh = Thread(target=controlSpeed)
shTh = Thread(target=showCoordinates)

readTh.start()
writeTh.start()
ctrlTh.start()
shTh.start()

readTh.join()
writeTh.join()
ctrlTh.join()
shTh.join()
#Author: Mr.Chanapai Chuadchum 
#Project name: Catbot 
#Project description: This project was devlop to be the robot that cooperating with human in the family be a part of daily life and learn from human 
#date developing: 28/5/2019 - Now 
      # Code running on the single board computer 
from gpiozero import LED  # Control direct GPIO for the rpi zero w 
from time import sleep
from picamera.array import PiRGBArray
from picamera import PiCamera # processing image from picamera
import cv2       #image processing function
import time      #timing control function
import pyfirmata #hardware interface  pyfirmata protocol official verison
import math
import numpy as np # Numpy for the array calculation function 
import microgear.client as microgear # Microgear for the IoT communication with other computer 
   # Prototype version of the hardware using arduino Mega 
from nanpy import(ArduinoApi,SerialManager) # Nanpy protocol hardware interface fo$camera = PiCamera()
import sklearn  
import serial # Serial communication uart GPS and other system 
import pyttsx3 # pyttsx for text to speech
import pyaudio # Microphone and audio deep control 
    # Google drive library 
from pydrive.auth import GoogleAuth  #Authentication 
from pydrive.drive import GoogleDrive # Google drive 
camera.resolution = (384, 288)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(384, 288))
led  = LED(17)  # Tail  LED 
led1 = LED(18)  # Right eye 
led2 = LED(19)  # Left  ese 
display_window = cv2.namedWindow("Catbot vision system")
#Haarcascade for training to detection
face_cascade = cv2.CascadeClassifier('/usr/share/opencv/opencv/data/haarcascades/haarcascade_frontalface_alt.xml')
cat_cascade = cv2.CascadeClassifier('/usr/share/opencv/opencv/data/haarcascades/haarcascade_frontalcatface.xml')
emotion_cascade = cv2.CascadeClassifier('/usr/share/opencv/opencv/data/Faceemo/cascade.xml')
dog_cascade = cv2.CascadeClassifier('/usr/share/opencv/opencv/data/Dogcascade-/cascade.xml') 
time.sleep(1)
r = 0
# 3D space angle convert function radians to degrees
angleDegX = 0
angleDegY = 0
angleDegZ = 0
c = 0
  # Gyro scope address
    #Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                     #Hardware connection part 
try: 
    hardware = pyfirmata.ArduinoMega('/dev/ttyACM0') # Hardware try connection 
except: 
    print("Hardware STM32 not found !")
    print("Prepare for seccond protocol ....")
    try: 
       connection = SerialManager()  # Firmware update hardware serial finder 
       catbot = ArduinoApi(connection=connection) 
    except:    
        print("All hardware fail connection")
                    
                    # GPS 
try: 
    gps = serial.Serial("/dev/ttyS0",115200) #GPS connection 
except: 
    print("GPS device notfound !")  
x = str(gps.read(1200))
pos1 = x.find("$GPRMC")
pos2 = x.find("\n",pos1)
loc = x[pos1:pos2]
data = loc.split(',')

if data[2] == 'V':
      print('No location found')
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                     #Hardware functioning actuator control 
try: 
   NeckL = hardware.get_pin('d:2:s')
   NeckR = hardware.get_pin('d:3:s')
   ServoFrontLegL = hardware.get_pin('d:4:s')
   ServoFrontLegR = hardware.get_pin('d:5:s')
   ServoBackLegL  = hardware.get_pin('d:6:s')
   ServoBackLegR  = hardware.get_pin('d:7:s')
   ServoMiddlebodyrotate = hardware.get_pin('d:10:s') 
   tailx = hardware.get_pin('d:8:s')
   taily = hardware.get_pin('d:9:s')
                    #Pump for the power actuator servo hydraulic 
   hydraulicpump = hardware.get_pin('d:12:s') #PWM function for the hydrauic servo motor 
                    # Hydraulic servos function
            # Sensor iteration loop  
   it = pyfirmata.utils.Iterator(hardware)
   it.start() 
    # Hardware analog read report back 
   hardware.analog[0].enable_reporting()
   hardware.analog[1].enable_reporting()
   hardware.analog[2].enable_reporting()
   hardware.analog[3].enable_reporting() 
except: 
     print("Switching to the back up prototype protocol") 
     
# Gyro sensor calculation function on the body dynamic
def read_byte(reg):
    return bus.read_byte_data(address, reg)

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
#Gyroscope sensor
bus = smbus.SMBus(1) # bus = smbus.SMBus(0) fuer Revision 1
address = 0x68       # via i2cdetect
         # Aktivieren, um das Modul ansprechen zu koennen
bus.write_byte_data(address, power_mgmt_1, 0)
print("Gyroscope")
print("--------")
gyroskop_xout = read_word_2c(0x43)
gyroskop_yout = read_word_2c(0x45)
gyroskop_zout = read_word_2c(0x47)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                 # Motion control and stabilization 
     #Hydraulic Servo networking function 
      # Driver unit 3 for the robotics solenoid control 
def FrontLeftleg(goalfl,errorfl):
    if errorfl == goalfl:
        hardware.digital[49].write(0) 
        hardware.digital[52].write(0)

    if errorfl < goalfl: 
       hardware.digital[49].write(1)
       hardware.digital[52].write(0)
    
    if errorfl > goalfl: 
       hardware.digital[49].write(0)
       hardware.digital[52].write(1)8

       
def FrontRightleg(goalfr,errorfr): 
    if errorfr == goalfr: 
        hardware.digital[51].write(0) 
        hardware.digital[27].write(0)

    if errorfr < goalfr: 
        hardware.digital[51].write(1) 
        hardware.digital[57].write(0)

    if errorfr > goalfr: 
        hardware.digital[51].write(0) 
        hardware.digital[27].write(1)


def BackLeftleg(goalbl,errorbl): 
    if errorbl == goalbl: 
        hardware.digital[28].write(0) 
        hardware.digital[44].write(0)

    if errorbl < goalbl: 
        hardware.digital[28].write(1) 
        hardware.digital[44].write(0)

    if errorbl > goalbl: 
        hardware.digital[28].write(0) 
        hardware.digital[44].write(1)

def BackRightleg(goalbr,errorbr): 
    if errorbr == goalbr: 
        hardware.digital[45].write(0) 
        hardware.digital[46].write(0)

    if errorbr < goalbr: 
        hardware.digital[45].write(1) 
        hardware.digital[46].write(0)

    if errorbr > goalbr:
        hardware.digital[49].write(0) 
        hardware.digital[52].write(1)
   # Hydrualic at the chess control for the stabilisation system 
def Chessfl(goalcfl,errorcfl): 
    if errorcfl == goalcfl:
      hardware.digital[21].write(0)
      hardware.digital[22].write(0)
    if errorcfl < goalcfl: 
      hardware.digital[21].write(1)
      hardware.digital[22].write(0)
    if errorcfl > goalcfl: 
      hardware.digital[21].write(0)
      hardware.digital[22].write(1)

def Chessfr(goalcfr,errorcfr): 
    if errorcfr == goalcfr:
      hardware.digital[23].write(0)
      hardware.digital[35].write(0)
    if errorcfr < goalcfr: 
      hardware.digital[23].write(1)
      hardware.digital[35].write(0)
    if errorcfr > goalcfr: 
      hardware.digital[23].write(0)
      hardware.digital[35].write(1)
def Chessbl(goalcbl,errorcbl): 
    if errorcfl == goalcfl:
      hardware.digital[36].write(0)
      hardware.digital[39].write(0)
    if errorcfl < goalcfl: 
      hardware.digital[36].write(1)
      hardware.digital[39].write(0)
    if errorcfl > goalcfl: 
      hardware.digital[36].write(0)
      hardware.digital[39].write(1)
def Chessbr(goalcbr,errorcbr):
    if errorcfl == goalcfl:
      hardware.digital[40].write(0)
      hardware.digital[62].write(0)
    if errorcfl < goalcfl: 
      hardware.digital[40].write(1)
      hardware.digital[63].write(0)
    if errorcfl > goalcfl: 
      hardware.digital[40].write(0)
      hardware.digital[62].write(1)
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
       # Audio function 
engine = pyttsx3.init() 
engine.setProperty('rate',150)
engine.setProperty('volume',0.9)
     # Stabilizer function for the catbot 
def Stabilizationfunction(Kx,Ky,Kz,goalMid,goalFl,goalFr,goalBl,goalBr,angleDegX,angleDegY,angleDegZ):  #Middle body stabilization control  
        #Middle Angle stabilization control
        # X - Angle Stabilizer   
       Middlepd = (goalMid - Kx*abs(angleDegX))
       ServoMiddlebodyrotate.write(Middlepd) # Servo angle control middle rotation of the catbot stabilization  
        # Y - Angle Stabilizer 
       if angleDegY == 90: 
           print("Stabilizing in line ....")
       if angleDegY < 90: 
           print("Stabilizer increasing ....")
       if angleDegY > 90: 
           print("Stabilizer decreasing ....")
        # Z - Angle Stabilizer 
       if angleDegZ == 90: 
           print("Stabilizing in line ....")
       if angleDegZ < 90: 
           print("Stabilizer increasing ....")
       if angleDegZ > 90: 
           print("Stabilizer decreasing ....")
#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
def Calculatedeep(w,h):
      d = w*h
      r = 226*(d/52441)
      return r
def HeadCoordinate(x,y): 
     NeckL.write((x/384)*180)
     NeckR.write((y/288)*180)
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True): 
    image = frame.array
        # Gyroscope function for the robot stabilization 
    print("gyroscope_xout: ", ("%5d" % gyroskop_xout), " radian x : ", (gyroskop_xout /131))
    print("gyroscope_yout: ", ("%5d" % gyroskop_yout), " radian y: ", (gyroskop_yout /131))
    print( "gyroscope_zout: ", ("%5d" % gyroskop_zout), "radian z: ", (gyroskop_zout/131))
    print("Catbot Gyroscope sensor")
    print("---------------------")
    beschleunigung_xout = read_word_2c(0x3b)
    beschleunigung_yout = read_word_2c(0x3d)
    beschleunigung_zout = read_word_2c(0x3f)
    beschleunigung_xout_skaliert = beschleunigung_xout / 16384.0
    beschleunigung_yout_skaliert = beschleunigung_yout / 16384.0
    beschleunigung_zout_skaliert = beschleunigung_zout / 16384.0
           # Angle calculation function convert to degrees
    angleDegX = math.degrees(beschleunigung_xout_skaliert)
    angleDegY = math.degrees(beschleunigung_yout_skaliert)
    angleDegZ = math.degrees(beschleunigung_zout_skaliert)
    print("AngleDegX",(angleDegX))  # Angle X
    print("AngleDegY",(angleDegY))  # Angle Y
    print("AngleDegZ",(angleDegZ))  # Angle Z
    print("Outdoor positioning system") # GPS device location detection 
    print("Latitude =" + data[3])
    print("Longtitude = " + data[5])
    print("Compass=" + data[4] +"," + data[6] )
    print("speed = " + data[7])
    print("Angle on compass",math.degrees(math.atan(data[3]/data[5]))) # Angle compass rotation  
          # Actuator control for the stabilization 
    #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
                     # Analog Potentiometer sensors read 
    goalfl = hardware.analog[0].read() # front leg analog read 
    goalfr = hardware.analog[1].read() 
    goalbl = hardware.analog[2].read() # badck leg analog read 
    goalbr = hardware.analog[3].read() 
    #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    print(goalfl,goalfr,goalbl,goalbr) # display the angle value 
    # Objects and human detect through haarcascade training ... 
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)
    for (x,y,w,h) in faces:
       # font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
        cv2.putText(image,'Human face',(x-w,y-h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0),1,-1)  
       # cv2.putText(image,'Human face',(x-w,y-h))
        HeadCoordinate(x,y)
        print(x,y,w,h)
        print("Calculate deep of picture detected:")
        print(Calculatedeep(w,h))
        print("meter")
        
    cats = cat_cascade.detectMultiScale(gray,1.1, 5)
    for(x,y,w,h) in cats:
            #font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.rectangle(image,(x,y),(x+w,y+h),(54,255,255),2)
            cv2.putText(image,'Cat face',(x-w,y-h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),1,-1)
            HeadCoordinate(x,y)
            print("Cat detected")
#    emotion = emotion_cascade.detectMultiScale(gray, 1.1, 5)
 #   for(x,y,w,h) in emotion:
  #          cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
   #         print("Emotion detected")
    dogs = dog_cascade.detectMultiScale(gray, 1.1, 5)
    for(x,y,w,h) in dogs:
        #    font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,0,255),2)
            cv2.putText(image,'Siberian husky',(x-w,y-h),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),1,-1)
            HeadCoordinate(x,y)
         #   cv2.puText(image,'Seiberian husky',(x-w,y-h))
            print("Dog species seiberien husky")
    
    cv2.imshow("Catbot vision system", image)
    key = cv2.waitKey(1)
    rawCapture.truncate(0)

    if key == 27:
        camera.close()
        cv2.destroyAllWindows()
        break
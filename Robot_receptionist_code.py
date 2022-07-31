#Libraries
import RPi.GPIO as GPIO
import time
import pyttsx3
from gpiozero import Button
from gpiozero import LED
from picamera import PiCamera
from time import sleep
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from imutils.video import VideoStream
import numpy as np
import imutils
import cv2
import os

#Ignore warning for now
GPIO.setwarnings(False)

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO pins
GPIO_TRIGGER = 4
GPIO_ECHO = 17
red = LED(19)
green = LED(26)
Nbutton = Button(18)
Ybutton = Button(23)
zOneButton = Button(24)
zTwoButton = Button(25)

#declaring variables
timeout = 60
wait = 1
en = pyttsx3.init()
camera = PiCamera()

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.0001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
    return distance

def detect_and_predict_mask(frame, faceNet, maskNet):
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), (104.0, 177.0, 123.0))
    faceNet.setInput(blob)
    detections = faceNet.forward()
    faces = []
    locs = []
    preds = []
    for i in range(0, detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > 0.5:
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")
            (startX, startY) = (max(0, startX), max(0, startY))
            (endX, endY) = (min(w - 1, endX), min(h - 1, endY))
            face = frame[startY:endY, startX:endX]
            face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
            face = cv2.resize(face, (224, 224))
            face = img_to_array(face)
            face = preprocess_input(face)
            faces.append(face)
            locs.append((startX, startY, endX, endY))
            if len(faces) > 0:
            faces = np.array(faces, dtype="float32")
            preds = maskNet.predict(faces, batch_size=32)
            return (locs, preds)
            prototxtPath = "face_detector/deploy.prototxt"
            weightsPath =  "face_detector/res10_300x300_ssd_iter_140000.caffemodel"
            faceNet = cv2.dnn.readNet(prototxtPath, weightsPath)
            print("[INFO] loading face mask detector model...")
            maskNet = load_model("MaskDetector.h5")
            print("[INFO] starting video stream...")
            vs = VideoStream(src=0).start()
            time.sleep(2.0)
          if key == ord("q"):
              break
          cv2.destroyAllWindows()
          vs.stop()

if __name__ == '__main__':
    try:
        while 1:
            dist = distance()
            if dist < 30:
                red.on()
                en.say("Hi there! I hope you are doing good today")
                en.runAndWait()
                print ("\nHi there, I hope you are doing good today")
                time.sleep(1)
                en.say("Please look into the camera for mask detection")
                en.runAndWait()
                time.sleep(1)
                camera.start_preview()
                sleep(5)
                while True:
                    frame = vs.read()
                    frame = imutils.resize(frame, width=500)
                    (locs, preds) = detect_and_predict_mask(frame, faceNet, maskNet)
                    for (box, pred) in zip(locs, preds):
                    (startX, startY, endX, endY) = box
                    (mask, withoutMask) = pred
                    if mask > withoutMask:
                        en.say("Face mask is not detected.")
                        en.runAndWait()
                        print ("\n Face mask is not detected.")
                        time.sleep(2)
                    else:
                        en.say("Face mask is detected.")
                        en.runAndWait()
                        print ("\n Face mask is detected.")
                        time.sleep(2)
                camera.stop_preview()
                en.say("Lets start with self assessment questions. Please listen to all the questions thoroughly.")
                en.say(" If you answer yes to any of them or feel ill, please select button labelled Y for three seconds.")
                en.say(" Else, press button labelled N for three seconds")
                en.runAndWait()
                print ("\nLets start with self assessment questions. Please listen to all the questions thoroughly. If you answer yes to any of them or feel ill, please select button labelled Y for three seconds. Else, press button labelled N for three seconds")
                en.say("Do you have any of the following new or worsening symptoms?")
                en.runAndWait()
                print ("\nDo you have any of the following new or worsening symptoms?")
                print ("\n1. Fever \n2. Cough \n3. Barking cough \n4. Shortness of breath \n5. Sore throat \n6. Runny nose \n7. Difficulty swallowing \n8. Decrease sense of taste or smell \n9. Nausea \n10. Chills \n11. Headache \n12. Pink eye")
                time.sleep(3)
                en.say("Have you travelled outside of Canada in past 14 days?")
                en.runAndWait()
                print ("\nHave you travelled outside of Canada in the past 14 days?")
                time.sleep(1)
                e.say("Have you tested positive for covid nineteen or currently awaiting results from covid nineteen test and have told to self isolate")
                e.runAndWait()
                print ("\nHave you tested positive for Covid-19 or currently awaiting results from covid nineteen test and have told to self isolate")
                time.sleep(1)
                en.say("Have you been in close contact with someone who has covid nineteen or currently has covid nineteen symptoms")
                en.say("or have you been told to stay at home and self isolate?")
                en.runAndWait()
                print ("\nHave you been in close contact with someone who has covid nineteen or currently has covid nineteen symptoms or have you been told to stay at home and self isolate?")
                time.sleep(1)
                en.say("Please press the button for three seconds now")
                en.runAndWait()
                print ("\nPlease press the button for 3 seconds now")
                time.sleep(4)
                start = time.time()
                if Ybutton.is_pressed and time.time() - start < timeout:
                    print ("\nButton Y was pressed!")
                    en.say("Sorry your entry to this premises is not permitted")
                    en.runAndWait()
                    print ("\nSorry your entry to this premises is not permitted")
                elif Nbutton.is_pressed and time.time() - start < timeout:
                    print ("\nButton N was pressed!")
                    red.off()
                    green.on()
                    en.say("You have passed covid nineteen self assessment test. Please select the zone in which you would like to work today")
                    en.runAndWait()
                    print ("\nYou have passed Covid-19 self assessment test. Please select the zone in which you would like to work today")
                    time.sleep(4)
                    start = time.time()
                    if zOneButton.is_pressed and time.time() - start < timeout:
                        en.say( "Zone 1 was seleted ")
                        en.runAndWait()
                        print ("\nZone1 was selected")
                        i = 0
                        if (i<2):
                            en.say("Please proceed to selected area")
                            en.runAndWait()
                            print ("\nPlease proceed to selected area")
                            i=i+1
                        else:
                            en.say("Zone 1 is full. Please select some other zone")
                            en.runAndWait()
                            print ("\nZone 1 is full. Please select some other zone")
                            time.sleep(3)
                            if (zTwoButton.is_pressed and time.time() - start < timeout && i<2):
                                en.say("Zone 2 was seleted. Please proceed to selected area.")
                                en.runAndWait()
                                print ("\nZone2 was selected. Please proceed to selected area.")
                                i=i+1
                            else
                                en.say("Zone 2 is full. Please select some other zone")
                                en.runAndWait()
                                print ("\nZone 2 is full. Please select some other zone")
                                time.sleep(3)    
                    elif zTwoButton.is_pressed and time.time() - start < timeout:
                        en.say("Zone 2 was seleted ")
                        en.runAndWait()
                        print ("\nZone2 was selected")
                        i = 0
                        if (i<2):
                            en.say("Please proceed to selected area")
                            en.runAndWait()
                            print ("\nPlease proceed to selected area")
                            i=i+1
                        else:
                            en.say("Zone 2 is full. Please select some other zone")
                            en.runAndWait()
                            print ("\nZone 2 is full. Please select some other zone")
                            time.sleep(3)
                            if (zOneButton.is_pressed and time.time() - start < timeout && i<2):
                                en.say("Zone 1 was seleted. Please proceed to selected area.")
                                en.runAndWait()
                                print ("\nZone1 was selected. Please proceed to selected area.")
                                i=i+1
                            else
                                en.say("Zone 1 is full. Please select some other zone")
                                en.runAndWait()
                                print ("\nZone 1 is full. Please select some other zone")
                                time.sleep(3)
                    else:
                        en.say( "No button was pressed")
                        en.runAndWait()
                        print ("\nNo button was pressed")
                    time.sleep(1)
                en.say("Have a nice day ahead")
                en.runAndWait()
                print ("\nHave a nice day ahead")
                red.off()
                green.off()
                time.sleep(5)
    except KeyboardInterrupt:
        print ("Program stopped by User")
        GPIO.cleanup()
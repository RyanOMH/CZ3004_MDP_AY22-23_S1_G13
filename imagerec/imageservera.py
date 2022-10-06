# import the necessary packages
from imutils import build_montages
from datetime import datetime
import numpy as np
import imagezmq
import argparse
import imutils
import cv2
import torch
import stitchImages

# initialize the ImageHub object
imageHub = imagezmq.ImageHub()
# initialize the list of class labels MobileNet SSD was trained to
# detect, then generate a set of bounding box colors for each class

# load our serialized model from disk
print("[INFO] loading model...")

#change path accordingly
#model =  torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt', force_reload=True) 

#run locally 
model = torch.hub.load('/Users/gordon/Desktop/imagerec/yolov5', 'custom', path='5Oct.pt', source='local')  # local repo


print("Model initialisation done")

# initialize the consider set (class labels we care about and want
# to count), the object count dictionary, and the frame  dictionary
CONSIDER = set(["dog", "person", "car"])
objCount = {obj: 0 for obj in CONSIDER}
frameDict = {}
# initialize the dictionary which will contain  information regarding
# when a device was last active, then store the last time the check
# was made was now
lastActive = {}
lastActiveCheck = datetime.now()
# stores the estimated number of Pis, active checking period, and
# calculates the duration seconds to wait before making a check to
# see if a device was active
ESTIMATED_NUM_PIS = 1
ACTIVE_CHECK_PERIOD = 10
ACTIVE_CHECK_SECONDS = ESTIMATED_NUM_PIS * ACTIVE_CHECK_PERIOD


# start looping over all the frames
while True:
    # receive RPi name and frame from the RPi and acknowledge
	# the receipt
    (rpiName, frame) = imageHub.recv_image()
    print(rpiName + " connected")
	# if a device is not in the last active dictionary then it means
	# that its a newly connected device
    if rpiName not in lastActive.keys():
        print("[INFO] receiving data from {}...".format(rpiName))
	# record the last active time for the device from which we just
	# received a frame
    
    lastActive[rpiName] = datetime.now()
    #frame = imutils.resize(frame, width=640)
    resized = cv2.resize(frame,(640,640))
    (h, w) = resized.shape[:2]

    #frame = imutils.resize(frame, width=400)
    #set confidence for model 
    model.conf = 0.85

    results = model(resized)
    print(results)
    info = results.pandas().xyxy[0].to_dict(orient = "records")
    if len(info) != 0:
        #id = info[0]['class']
        name = info[0]['name']
        confidence = info[0]['confidence']
        if confidence > 0.3: 
            #encoded_id = str(id).encode()
            encoded_name = str(name).encode()
            imageHub.send_reply(encoded_name)
            results.render()
            results.show()
            results.save()
            stitchImages.stitching()
            # img = cv2.imread('runs/stitched/stitchedOutput.png')
            # img_resize = cv2.resize(img, (960,540))
            # cv2.imshow('Stitched Image', img_resize)
        else: 
            imageHub.send_reply(b'n')
            # results.render()
            # results.show()
            # results.save()



    else:
        imageHub.send_reply(b'n')
        # results.render()
        # results.show()
        # results.save()


   

    #image 1/1: 1080x1920 1 up
    #Speed: 54.5ms pre-process, 9.4ms inference, 1.3ms NMS per image at shape (1, 3, 384, 640)

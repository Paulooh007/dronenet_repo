import numpy as np
import cv2
import random


w, h = 1000, 680
takeoff_height = 10

def distance(pointA, pointB):
    """Get Euclidean distance between 2 points"""
    return (
        ((pointA[0] - pointB[0]) ** 2) +
        ((pointA[1] - pointB[1]) ** 2)
    ) ** 0.5  

def drawMarker(corners, ids):
    """Draw marker bounding box and return the center coordinates, height and width"""
    for (markerCorner, markerID) in zip(corners, ids):
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        markerWidth = int(distance(topLeft,topRight))
        markerHeight = int(distance(topLeft, bottomLeft))

        # draw the bounding box of the ArUCo detection
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

        # compute and draw the center (x, y)-coordinates of the
        # ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

        # draw the ArUco marker ID on the frame
        cv2.putText(frame, str(markerID),
        (topLeft[0], topLeft[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)

    return [(cX, cY), (markerWidth, markerHeight)]  #[ midpoint, w and h]

def drawQuadrant():
    """Divide frame into 4 regions/quadrants"""
    cv2.line(frame, (centerX, 0), (centerX,h), (0, 0, 255), 1)
    cv2.line(frame, (0, centerY), (w,centerY), (0, 0, 255), 1)
    cv2.circle(frame, (centerX, centerY), 4, (0, 255, 255), -1)


if (__name__ == "__main__"):

    cap = cv2.VideoCapture(0)

    centerX, centerY = w//2, h//2  #Get center of Frame

    rand = random.randint(0, 1000)
    debug_image_writer = cv2.VideoWriter(f"vid_record_" + str(rand) + ".avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25.0,(w,h))

    while True:
        
        # frame = me.get_frame_read().frame
        ret, frame = cap.read()

        frame = cv2.resize(frame, (w,h))

        cv2.putText(frame, f"Takeoff Alt: {takeoff_height} Relative Alt: {takeoff_height}",
                        (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)  
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        
        drawQuadrant()

        #Find aruCo marker in frame
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        parameters =  cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # print(corners)
        if len(corners) > 0:   #if All 4 corners of marker is detected

            ids = ids.flatten()
            markerInfo = drawMarker(corners, ids)  

            cX, cY = markerInfo[0]   #Get marker midpoint

            #Calculate error i.e Distance btw aruco midpoint and frame midpoint
            error = int(distance((cX, cY), (centerX, centerY)))
            markerWidth, markerHeight = markerInfo[1]

            rW, rH = round(markerWidth/w, 1), round(markerHeight/h, 1)    #Calculate ratio
            # rW, rH = 0.4, 0.4
            #Draw line between center of frame and center of marker
            cv2.line(frame, (cX, cY), (centerX,centerY), (254, 255, 0), 3)


        cv2.imshow('video',frame)
        debug_image_writer.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # me.land()
            # print(me.get_battery())
            break

# pip uninstall opencv-contrib-python

# 4.6.0.66
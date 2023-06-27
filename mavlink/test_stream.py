import cv2


cap = cv2.VideoCapture(1)

while True:
    _, img = cap.read()

    img = cv2.resize(img, (640, 320))
    cv2.imshow("Video out", img)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
import detect_corn as dc
import cv2
import time

detector = dc.detector()

roi_img = cv2.imread("./log/captured.png")
roi_img = cv2.cvtColor(roi_img, cv2.COLOR_BGR2RGB)

detector.set_roi_img(roi_img)

try:
    while True:
        detector.detect_cone()
        cone_direction = detector.cone_direction
        cone_probability = detector.probability
        print(cone_direction, cone_probability)
        print("is detected: ", detector.detected)
        time.sleep(0.1)
except Exception as e:
    print(e)
    exit(-1)

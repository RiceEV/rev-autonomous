import cv2
import numpy as np

# define a video capture object
vid = cv2.VideoCapture(0)

def detect_blue(frame):
    '''
    detect_red color with a given threshold. <:
    '''
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([90, 120, 0], dtype="uint8")
    upper_red = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_red, upper_red)
    #cv2.imshow("Red_color",mask)
    return mask

while (True):

    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Display the resulting frame
    cv2.imshow('frame', detect_blue(frame))



    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
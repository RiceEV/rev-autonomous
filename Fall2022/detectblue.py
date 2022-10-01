import cv2
import numpy as np

# define a video capture object
vid = cv2.VideoCapture(0)

'''
Takes in a numpy frame of dims <height>x<width>x3 in HSV form
Returns a mask of 0s and 255s depending on if the pixel is in the HSV range
The output dims are <height> x <width>
'''
def detect_blue(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 120, 0], dtype="uint8")
    upper_red = np.array([20, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_red, upper_red)
    return mask

'''
Detects the edges in an input greyscale image
In this case, we are passing a mask of 0s and 255s
It will return a 0s and 255s numpy array of edges or non-edge pixels
'''
def edge_detect(mask):
    edges = cv2.Canny(mask, threshold1=70, threshold2=100)
    print("edges shape:", edges.shape)
    return edges

'''
Crops out the top half of the image
You can change polygon to your heart's desire!
'''
def crop_roi(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (width / 6,  height/2),
        (width * 5/ 6 , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)

    cropped_edges = cv2.bitwise_and(edges, mask)
    print("crop_roi output shape:", cropped_edges.shape)

    return cropped_edges

'''
Detects the line segments of a minimum length from the cropped edges
HoughLinesP has parameters you can adjust. Look at the documentation in OpenCV!
'''
def detect_line_segments(cropped_edges):
    #detects line segment via HoughLines
    rho = 1
    theta = np.pi / 180
    min_threshold = 10

    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    print("Number of line segments found:", line_segments.size)

    return line_segments

'''
Adds the lines to the original image and returns that image
'''
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    #this just displays the boundary lines we previously found on the image
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                #line displayed here
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

while (True):
    # Displaying multiple windows for each step in the pipeline
    # Capture the video frame by frame
    ret, frame = vid.read()
    cv2.imshow('image', frame)

    # Converts BGR (OpenCV hates RGB for some reason) to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Display the resulting frame
    blue_here = detect_blue(frame)
    cv2.imshow('frame', blue_here)

    edges = edge_detect(blue_here)
    cv2.imshow('edge detect', edges)

    cropped = crop_roi(edges)
    cv2.imshow('cropped', cropped)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()
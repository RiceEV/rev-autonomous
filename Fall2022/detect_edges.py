import cv2
import numpy as np

def edge_detect(mask):
    edges = cv2.Canny(mask, 50, 100)
    print("edges shape:", edges.shape)
    return edges

def crop_roi(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)

    cropped_edges = cv2.bitwise_and(edges, mask)
    #cv2.imshow("roi",cropped_edges)
    print("crop_roi output shape:", cropped_edges.shape)

    return cropped_edges

def detect_line_segments(cropped_edges):
    #detects line segment via HoughLines
    rho = 1
    theta = np.pi / 180
    min_threshold = 10

    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    print("Number of line segments found:", line_segments.size)

    return line_segments

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

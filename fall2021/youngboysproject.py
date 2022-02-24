import cv2
import numpy as np
import math
import sys
import time
import matplotlib.pyplot as plt
import Adafruit_BBIO.PWM as PWM
'''
YoungBoys
Made by Christian, Son, Mahmoud, and Robbie

Code inspired by:
User raja_961, "Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV"
https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/
'''

def detect_red(frame,threshold=1000):
    '''
    detect_red color with a given threshold. <:
    '''
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red = np.array([150, 70, 50], dtype="uint8")
    upper_red = np.array([179, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv, lower_red, upper_red)
    #cv2.imshow("Red_color",mask)
    return mask.sum()>threshold

def detect_edges(frame):
    #print("Start ED")
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype = "uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue)
    #cv2.imshow("mask",mask)

    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    #cv2.imshow("edges",edges)
    #print("End ED")
    return edges

def region_of_interest(edges):
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

    return cropped_edges

def detect_line_segments(cropped_edges):
    #detects line segment via HoughLines
    rho = 1
    theta = np.pi / 180
    min_threshold = 10

    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold,
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def average_slope_intercept(frame, line_segments):
    #finds average slope intercept
    lane_lines = []

    if line_segments is None:
        #print("no line segments detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2: #skip if slope is infinity
                continue

            #find line
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            #plot to where it converges at the center
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    #helper function for drawing
    height, width, _ = frame.shape

    slope, intercept = line

    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0:
        slope = 0.1

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

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

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    #this generates and displays the heading lines
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    #find angle
    steering_angle_radian = steering_angle / 180.0 * math.pi

    #find the points for the line
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    #display heading line
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

def get_steering_angle(frame, lane_lines):
    #produces the steering angle for the heading line and 
    #used to find deviation
    height,width,_ = frame.shape

    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)

    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)

    #fun cosine math to find angle
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90

    return steering_angle
    
def cam_test(res):
    '''
    Helper function for testing.
    Seeing the camera input and lines being detected.
    '''
    #Capturing
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH,320)#320
    video.set(cv2.CAP_PROP_FRAME_HEIGHT,60)#240
    while(True):      
        ret,frame = video.read()
        frame = cv2.resize(frame, (res, res))
        #frame = cv2.flip(frame,-1)
        #cv2.imshow("original",frame)
        edges = detect_edges(frame)
        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame,line_segments)
        lane_lines_image = display_lines(frame,lane_lines)
        steering_angle = get_steering_angle(frame, lane_lines)
        heading_image = display_heading_line(lane_lines_image,steering_angle)
        cv2.imshow("heading line",heading_image)
        print("Frame processed\n")
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
           break
    video.release()
    cv2.destroyAllWindows()
    
def main(res=100, kpr=12, kdr=8, driving_speed = 7.92 ):
    steering_port = "P9_14"
    driving_port = "P8_13"
    
    # Set up PWM
    PWM.start(steering_port, 7.5, 50)
    PWM.start(driving_port, 7.5, 50)

    #Capturing
    video = cv2.VideoCapture(0)
    video.set(cv2.CAP_PROP_FRAME_WIDTH,320)#320
    video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)#240

    #Saving Vid
    savedvid = cv2.VideoWriter('carvid.avi',cv2.VideoWriter.fourcc(*'MJPG'), 10 , (res,res))

    # Calibration Setup
    rotbase = 7.5
    lastTime = 0
    lastError = 0
    j= 0

    #Adding data points
    der_r = np.array([])
    pro_r = np.array([])
    err = np.array([])
    steering = np.array([])
    speed = np.array([])
    
    time.sleep(3) # calibration for esc
    PWM.set_duty_cycle(driving_port, driving_speed)
    
    #Red detection
    red_detected = False
    a=50
    
    while(True):
        ret,frame = video.read()
        frame = cv2.resize(frame, (res, res))
        
        #Don't attempt to read the stop sign for the first a's iteration.
        if (j>a):
            if detect_red(frame):
                if red_detected: #Terminate if we already saw stop sign before.
                  break
                a = j + 100
                red_detected = True
                PWM.set_duty_cycle(driving_port,7.5)
                time.sleep(2)
        
        #Edge Detection
        edges = detect_edges(frame)
        roi = region_of_interest(edges)
        line_segments = detect_line_segments(roi)
        lane_lines = average_slope_intercept(frame,line_segments)
        steering_angle = get_steering_angle(frame, lane_lines)
        
        #deviation calculation for P and D
        now = time.time()
        dt = now - lastTime
        deviation = steering_angle - 90
        
        if deviation < 3 and deviation > -3: #Min margin before turning
             #Find new derivative and proportional gain
            deviation = 0
            derivative_r = kdr * (deviation - lastError) / dt
            proportional_r = kpr * deviation
            PD_r = derivative_r + proportional_r
            rot = rotbase - PD_r/90
            PWM.set_duty_cycle(steering_port, 7.5) #going straight
        else:
            #Find new derivative and proportional gain
            derivative_r = kdr * (deviation - lastError) / dt
            proportional_r = kpr * deviation
            PD_r = derivative_r + proportional_r
            rot = rotbase - PD_r/90
            
            #Making sure rot stay in the correct duty cycle range.
            if rot>9.5: 
                rot = 9.5
            elif rot<5.5:
                rot = 5.5
            PWM.set_duty_cycle(steering_port, rot)
        new_speed = driving_speed+.0008*(j//20)
        PWM.set_duty_cycle(driving_port, new_speed)
        
        #Collecting data
        speed=np.append(speed, new_speed)
        steering=np.append(steering, rot)
        der_r=np.append(der_r,derivative_r)
        pro_r=np.append(pro_r,proportional_r)
        err=np.append(err,deviation/10)
        
        #Showing and Saving Video - Comment Out for better performance
        lane_lines_image = display_lines(frame,lane_lines)
        screen_data = f"E:{str(deviation)[:4]}, Str:{str(rot)[:4]}, Spd:{str(driving_speed)[:4]}"
        heading_image = display_heading_line(lane_lines_image,steering_angle)
        heading_image = cv2.putText(heading_image, screen_data, (10,30), cv2.FONT_HERSHEY_PLAIN, 1.2, (0,255,255), cv2.LINE_4) 
        savedvid.write(heading_image)
        cv2.imshow("heading line",heading_image)

        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        lastError = deviation
        lastTime = time.time()
        j += 1
        
    #setting values back to default
    PWM.set_duty_cycle(driving_port,7.5)
    PWM.set_duty_cycle(steering_port,7.5)
    savedvid.release()
    video.release()
    cv2.destroyAllWindows()
    j_array = np.array(range(j))

    
    #plotting error and pwm
    fig, ax = plt.subplots()
    ax.plot(j_array, steering, label="steering PWM")
    ax.plot(j_array, speed, label="speed PWM")
    ax.set_xlabel("Frame")
    ax.set_ylabel("PWM")
    ax.legend()

    ax2 = ax.twinx()
    ax2.plot(j_array, err, 'r-.', label="error")
    plt.grid(True)
    ax2.legend(loc=0)
    plt.title("PWM and Error")
    plt.show()

    plt.savefig("pwm.jpg")
    plt.clf()

    #Plotting proportional gain, der gain, error
    fig, ax = plt.subplots()
    ax.plot(j_array, der_r, 'g', label='derivative_r')
    ax.plot(j_array, pro_r, 'b', label='proportional_r')
    ax.set_xlabel("Frame")
    ax.legend()
    ax.set_ylabel("Proportional and Derivative Gain")

    ax2=ax.twinx()
    ax2.plot(j_array,err,'r-.', label='error')
    ax2.set_ylabel("Error")
    ax2.legend(loc=0)
    plt.title('KPR, KDR, and Error')
    plt.show()

    plt.savefig("prop.jpg")
    

#cam_test(res=80) #For testing the camera
main(res=250, kpr=10, kdr=0, driving_speed =7.88)




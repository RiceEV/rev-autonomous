import cv2
import numpy
import matplotlib.pyplot as plt
import time


vid = cv2.VideoCapture('Highway - 10364 (1).mp4')
#vid = cv2.VideoCapture("C:\\Users\\Taeho Choe\\Desktop\\Self_Projects\\AI Driving\\videos\\rice_campus_drive.mp4")

class Lane:
    def __init__(self, vid):
        self.vid = vid

        # Manual: REGION OF INTEREST
        # self.roi_points = numpy.float32([
        #     (390, 270), # Top-left corner
        #     (0, 450), # Bottom-left corner
        #     (780, 450), # Bottom-right corner
        #     (530, 270) # Top-right corner
        # ])
        #

        self.roi_points = numpy.float32([
            [260, 260], # Top-left corner
            [0, 300], # Bottom-left corner
            [854, 300], # Bottom-right corner
            [640, 260] # Top-right corner
        ])



    def lane_ident(self, frame):
        #frame = self.rescale(frame)
        self.width, self.height = frame.shape[1], frame.shape[0]

        self.frame = self.canny_img(frame)
        self.perspective_warp()
        lines = self.lane_lines()
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(self.image, (x1, y1), (x2, y2), (255, 0, 0), 3)

    def lane_lines(self):
        self.histogram = numpy.sum(self.frame[int(self.frame.shape[0]/2):,:], axis=0)

        # # Draw both the image and the histogram
        # figure, (ax1, ax2) = plt.subplots(2,1) # 2 row, 1 columns
        # figure.set_size_inches(10, 5)
        # ax1.imshow(self.frame, cmap='gray')
        # ax1.set_title("Warped Binary Frame")
        # ax2.plot(self.histogram)
        # ax2.set_title("Histogram Peaks")
        # plt.show()
        # plt.close()

        rho = 1
        theta = numpy.pi / 180
        min_threshold = 10

        line_segments = cv2.HoughLinesP(self.roi_points, rho, theta, min_threshold, numpy.array([]),
                                        minLineLength=5, maxLineGap=150)
        return line_segments

    def perspective_warp(self):
        # # The desired corner locations  of the region of interest after we perform perspective transformation.
        self.padding = int(0.3 * self.width) # padding from side of the image in pixels
        self.desired_roi_points = numpy.float32([
            [self.padding, 0], # Top-left corner
            [self.padding, self.height], # Bottom-left corner
            [self.width-self.padding, self.height], # Bottom-right corner
            [self.width-self.padding, 0] # Top-right corner
        ])

        # line_top = cv2.line(self.frame, [380, 260], [530, 260], (255,255,255), thickness=1)
        # line_left = cv2.line(self.frame, [260, 260], [0, 300], (255,255,255), thickness=1)
        # line_right = cv2.line(self.frame, [640, 260], [854, 300], (255,255,255), thickness=1)
        # line_bott = cv2.line(self.frame, [0, 400], [854, 400], (255,255,255), thickness=1)

        matrix = cv2.getPerspectiveTransform(self.roi_points, self.desired_roi_points)
        self.frame = cv2.warpPerspective(self.frame, matrix, (854, 480))


    def canny_img(self, img):
        gray_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_frame = cv2.GaussianBlur(gray_frame, (3,1), cv2.BORDER_DEFAULT)
        threshold, thresh = cv2.threshold(gray_frame, 200, 255, cv2.THRESH_BINARY)
        canny_frame = cv2.Canny(thresh, 250, 250)
        return canny_frame

    def rescale(self, frame, scale=0.5):
        self.width, self.height = int(self.width * scale), int(self.scale * scale)
        dimensions = (self.width, self.height)
        return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

    def run(self):
        while True:
            success, self.frame = self.vid.read()
            self.lane_ident(self.frame)
            cv2.imshow("Video", self.frame)
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break
        self.vid.release()
        cv2.destroyAllWindows()


test_lane = Lane(vid)
test_lane.run()



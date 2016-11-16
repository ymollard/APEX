from collections import deque
import numpy as np
import imutils
import cv2

# From http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/


class BallTracking(object):
    def __init__(self):
        self.buffer_size = 32
        # define the lower and upper color boundaries H,S, V
        self.lower = (27, 45, 185)
        self.upper = (38, 200, 255)
        self.lower_arena = (95, 110, 150)
        self.upper_arena = (110, 250, 250)


        # initialize the lists of tracked points in a map and the coordinate deltas
        self.pts = {}
        self.dX, self.dY = (0, 0)

        self.camera = None

    def read(self):
        if not self.camera:
            return False, None
        return self.camera.read()

    def get_images(self, frame):
        """
        Get HSV and masks
        :return:
        """
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask_ball = cv2.inRange(hsv, self.lower, self.upper)
        mask_ball = cv2.erode(mask_ball, None, iterations=4)
        mask_ball = cv2.dilate(mask_ball, None, iterations=10)

        mask_arena = cv2.inRange(hsv, self.lower_arena, self.upper_arena)
        mask_arena = cv2.erode(mask_arena, None, iterations=4)
        mask_arena = cv2.dilate(mask_arena, None, iterations=10)

        return hsv, mask_ball, mask_arena

    def find_center(self, name, frame, mask, min_radius):
        if name not in self.pts:
            self.pts[name] = deque(maxlen=self.buffer_size)

        # find contours in the mask and initialize the current (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > min_radius:
                # draw the circle and centroid on the frame, then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                self.pts[name].appendleft(center)
                return center, radius
        return None, None

    def draw_images(self, frame, hsv, mask_ball, mask_arena, arena_center, arena_ring_radius):
        self.draw_history(frame, 'ball')
        self.draw_history(frame, 'arena')
        cv2.circle(frame, arena_center, arena_ring_radius, (0, 128, 255), 2)


        rgbs = cv2.split(frame)
        hsvs = cv2.split(hsv)
        #cv2.imshow("Red", rgbs[0])
        #cv2.imshow("Green", rgbs[1])
        #cv2.imshow("Blue", rgbs[2])
        cv2.imshow("Hue", hsvs[0])
        #cv2.imshow("Saturation", hsvs[1])
        #cv2.imshow("Value", hsvs[2])
        cv2.imshow("Mask ball", mask_ball)
        cv2.imshow("Mask arena", mask_arena)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def draw_history(self, frame, name):
        # loop over the set of tracked points
        if name in self.pts:
            for i in np.arange(1, len(self.pts[name])):
                # if either of the tracked points are None, ignore
                # them
                if self.pts[name][i - 1] is None or self.pts[name][i] is None:
                    continue

                # check to see if enough points have been accumulated in
                # the buffer
                if len(self.pts[name]) >= 10 and i == 1 and self.pts[name][-10] is not None:
                    # compute the difference between the x and y
                    # coordinates and re-initialize the direction
                    # text variables
                    self.dX = self.pts[name][-10][0] - self.pts[name][i][0]
                    self.dY = self.pts[name][-10][1] - self.pts[name][i][1]

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(self.buffer_size / float(i + 1)) * 2.5)
                cv2.line(frame, self.pts[name][i - 1], self.pts[name][i], (0, 0, 255), thickness)
            if len(self.pts[name]) > 1:
                # show the movement deltas of movement
                cv2.putText(frame, "dx: {}, dy: {}".format(self.dX, self.dY),
                            (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, (0, 0, 255), 1)

    def open(self):
        try:
            self.camera = cv2.VideoCapture(1)
        except:
            return False
        else:
            return True

    def close(self):
        # cleanup the camera and close any open windows
        self.camera.release()
        cv2.destroyAllWindows()

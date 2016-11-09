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

        # initialize the list of tracked points, the frame counter,
        # and the coordinate deltas
        self.pts = deque(maxlen=self.buffer_size)
        self.dX, self.dY = (0, 0)
        self.direction = ""

        self.camera = cv2.VideoCapture(1)

    def read(self):
        return self.camera.read()

    def get_images(self, frame):
        """
        Get HSV and Mask images
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
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        return hsv, mask

    def find_center(self, frame, hsv, mask):
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 20:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                self.pts.appendleft(center)
                return center
        return None

    def draw_images(self, frame, hsv, mask):
        # loop over the set of tracked points
        for i in np.arange(1, len(self.pts)):
            # if either of the tracked points are None, ignore
            # them
            if self.pts[i - 1] is None or self.pts[i] is None:
                continue

            # check to see if enough points have been accumulated in
            # the buffer
            if len(self.pts) >= 10 and i == 1 and self.pts[-10] is not None:
                # compute the difference between the x and y
                # coordinates and re-initialize the direction
                # text variables
                self.dX = self.pts[-10][0] - self.pts[i][0]
                self.dY = self.pts[-10][1] - self.pts[i][1]
                (dirX, dirY) = ("", "")

                # ensure there is significant movement in the
                # x-direction
                if np.abs(self.dX) > 20:
                    dirX = "East" if np.sign(self.dX) == 1 else "West"

                # ensure there is significant movement in the
                # y-direction
                if np.abs(self.dY) > 20:
                    dirY = "North" if np.sign(self.dY) == 1 else "South"

                # handle when both directions are non-empty
                if dirX != "" and dirY != "":
                    self.direction = "{}-{}".format(dirY, dirX)

                # otherwise, only one direction is non-empty
                else:
                    self.direction = dirX if dirX != "" else dirY

            # otherwise, compute the thickness of the line and
            # draw the connecting lines
            thickness = int(np.sqrt(self.buffer_size / float(i + 1)) * 2.5)
            cv2.line(frame, self.pts[i - 1], self.pts[i], (0, 0, 255), thickness)

        if len(self.pts) > 1:
            # show the movement deltas and the direction of movement on
            # the frame
            cv2.putText(frame, self.direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (0, 0, 255), 3)
            cv2.putText(frame, "dx: {}, dy: {}".format(self.dX, self.dY),
                (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.35, (0, 0, 255), 1)

        rgbs = cv2.split(frame)
        hsvs = cv2.split(hsv)
        cv2.imshow("R", rgbs[0])
        cv2.imshow("G", rgbs[1])
        cv2.imshow("B", rgbs[2])
        cv2.imshow("H", hsvs[0])
        cv2.imshow("S", hsvs[1])
        cv2.imshow("V", hsvs[2])
        cv2.imshow("Mask", mask)
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

    def close(self):
        # cleanup the camera and close any open windows
        self.camera.release()
        cv2.destroyAllWindows()

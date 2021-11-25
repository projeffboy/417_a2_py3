import numpy as np
import cv2
import imutils
import random
from collections import OrderedDict

#rgb - 182, 0, 19

### webcam
#bgr_color = 44, 21, 179
### recorded video
#bgr_color = 32, 23, 138
#bgr_color = 19, 0, 182
bgr_color = 31, 0, 142
thresh = 100


hsv_color = cv2.cvtColor( np.uint8([[bgr_color]] ), cv2.COLOR_BGR2HSV)[0][0]
HSV_lower = np.array([hsv_color[0] - thresh, hsv_color[1] - thresh, hsv_color[2] - thresh])
HSV_upper = np.array([hsv_color[0] + thresh, hsv_color[1] + thresh, hsv_color[2] + thresh])

show_mask=False

class kf_2D:
    def __init__(self):
        self.Q = 1
        self.R = 1

    def predict(self):
        #predict the state
        #predict the covariance
        return


    def update(self, z_position):
        #update the kalman gain
        #update the state
        #update the covariance
        return

    def sensor_reading(self):
        return


class kf_1D:
    def __init__(self):
        self.Q = 1
        self.R = 1
        return


def detect_ball(frame):
    x, y, radius = -1, -1, -1
    # resize the frame, blur it, and convert it to the HSV
    # color space
    #frame = imutils.resize(frame, width=10)
    #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    #mask = cv2.inRange(hsv, color_lower, color_upper)
    mask = cv2.inRange(hsv_frame, HSV_lower, HSV_upper)
    mask = cv2.erode(mask, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #contours = contours[0] if imutils.is_cv2() else contours[1]
    contours = cnts[0]
    center = (-1, -1)

    # only proceed if at least one contour was found
    if len(contours) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(mask)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius <= 2:
            return -1, -1, 0, frame
        # # check that the radius is larger than some threshold
        # if radius > 2:
        #     #outline ball
        #     cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        #     #show center
        #     cv2.circle(frame, center, 5, (0, 255, 0), -1)
        #
        # if show_mask:
        #     cv2.imshow('mask', mask)

    return center[0], center[1], radius, frame #x, y , radius




#cap = cv2.VideoCapture(0)
#while(True):

#cap = cv2.VideoCapture('vids/ball1-noise.mp4')
# cap = cv2.VideoCapture('vids/VID_20181101_181543_90_quake.mp4')
# #cap = cv2.VideoCapture('vids/VID_20181101_181543_noise.mp4')
 #cap = cv2.VideoCapture('test.mp4')

i=0
cap = cv2.VideoCapture('ball_pid.mp4')
csv_file = 'pics.csv'
resize_height = 600
done_positions = []
# pos_to_file = OrderedDict()
pos_to_file = {}
while(cap.isOpened()):

    # Capture frame-by-frame
    ret, frame = cap.read()
    if frame is None:
        break
    frame = imutils.resize(frame, height=resize_height)

    # Our operations on the frame come here
#    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # if i == 0:
    x, y, radius, frame = detect_ball(frame)

    if y != -1:
        log_height = resize_height - y
        if log_height not in done_positions:
            done_positions.append(log_height)
            hash = random.getrandbits(32)
            str_hash = str("%08x" % hash)
            cv2.imwrite('pid_pics/{}.png'.format(str_hash), frame)
            pos_to_file[log_height] = '{}.png'.format(str_hash)

    # Display the resulting frame
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    i+=1
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

sorted_keys = []
for key in sorted(pos_to_file.keys()):
    sorted_keys.append(key)

min_pos = np.min(sorted_keys)
max_pos = np.max(sorted_keys)
print(('pos range: {} to {}'.format(min_pos, max_pos)))



#write to csv file
with open(csv_file, 'a') as the_file:
    for i in range(min_pos, max_pos + 1):
        if i in sorted_keys:
            filepath = pos_to_file[i]
        else:
            idx = (np.abs(np.array(sorted_keys) - i)).argmin()
            closest_i = sorted_keys[idx]
            filepath = pos_to_file[closest_i]
        the_file.write('{},'.format(i-min_pos) + filepath + '\n')


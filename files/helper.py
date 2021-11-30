import cv2
import numpy as np

def account_for_noise(frame):
    bgr_color = 31, 0, 142
    thresh = 100
    mean = frame.mean()
    if mean < 60 or mean > 90: # if there's noise
        # 9x9 averaging filter kernel
        kernel_size = 9
        kernel = np.ones((kernel_size, kernel_size), np.float32) \
            / (kernel_size ** 2)
        filter_num_times = 6
        for _ in range(filter_num_times):
            frame = cv2.filter2D(frame, -1, kernel) # https://docs.opencv.org/3.4/d4/d13/tutorial_py_filtering.html
            frame = cv2.normalize(
                frame, np.zeros((800, 800)), 0, 255, cv2.NORM_MINMAX
            ) # https://www.pythonpool.com/cv2-normalize/
        bgr_color = 82, 74, 152
        thresh = 25
    
    return bgr_color, thresh, frame

def pid(self, t):
    p_error = self.Kp * self.error_pos

    delta_t = t - self.last_t
    i_error = self.acc_pos_error + self.Ki * (self.error_pos + self.last_error_pos) / 2 * delta_t
    
    if delta_t != 0:
        error_vel = self.Kd * (self.error_pos - self.last_error_pos) / delta_t
        d_error = error_vel
    else:
        d_error = 0

    output = p_error + i_error + d_error
    self.acc_pos_error = i_error
    
    fan_rpm = output + self.bias
    fan_rpm = max(fan_rpm, 0) # can't have negative rpm

    return fan_rpm
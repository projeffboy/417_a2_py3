#!/usr/bin/python

__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

#Implement the ball tracking using OpenCV
import simulation
import pid_plotter_vispy
from multiprocessing import Process, Manager, Lock
import multiprocessing

__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

from helpers import *
import simulation

import random
import cv2
import math
import numpy as np
import csv
import os, sys
import pygame
import pickle
import time

class PID_controller:
    def __init__(self, target_pos = -0.1):
        #TODO YOU CAN ADD new variables as needed

        ###EVERYTHING HERE MUST BE INCLUDED###
        self.target_pos = target_pos

        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0
        self.bias = 0.0
        #
        self.Kp = 6000
        self.Ki = 500
        self.Kd = 500
        self.bias = 0.0

        # self.Kp = 10000
        # self.Ki = 5000
        # self.Kd = 1000
        # self.bias = 0.0

        self.detected_pos = 0.0
        self.rpm_output = 0.0
        ######################################

        self.error_pos = 0.0
        self.last_pos = 0.0
        self.last_error_pos = 0.0
        self.acc_pos_error = 0.0

        self.last_t = None

        self.min = 1000000
        self.max = 0
        return

    def set_target(self, target_pos):
        self.target_pos = target_pos

    def reset(self):
        self.detected_pos = 0.0
        self.rpm_output = 0.0
        self.error_pos = 0.0
        self.last_pos = 0.0
        self.last_error_pos = 0.0
        self.acc_pos_error = 0.0
        self.last_t = None
        return

    def detect_ball(self, frame):
        #TODO
        #TODO You are given a basic opencv ball tracker. However, this won't work well for the noisy case.
        #TODO Play around to get it working. You can reuse the one you implemented previously

        bgr_color = 31, 0, 142
        thresh = 100
        hsv_color = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        HSV_lower = np.array([hsv_color[0] - thresh, hsv_color[1] - thresh, hsv_color[2] - thresh])
        HSV_upper = np.array([hsv_color[0] + thresh, hsv_color[1] + thresh, hsv_color[2] + thresh])

        x, y, radius = -1, -1, -1
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        # mask = cv2.inRange(hsv, color_lower, color_upper)
        mask = cv2.inRange(hsv_frame, HSV_lower, HSV_upper)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = cnts[0]
        center = (-1, -1)
        # only proceed if at least one contour was found
        try:
            if len(contours) > 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(mask)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                #Ball is lost(went above the camera's line of sight)
                if radius <= 2:
                    return -1, -1
        except e:
            #no contour found ...
            center = (-1, -1)
            pass
        return center[0], center[1]  # x, y , radius

    def get_kalman_filter_estimation(self, observation):
        #TODO Implement the kalman filter ...
        #observation is the estimated x,y position of the detect image

        kalman_filter_estimate = [155, 0] #Return the kalman filter adjusted values
        return kalman_filter_estimate[0], kalman_filter_estimate[1]


    # def get_fan_rpm(self, image_frame=None, position=None):
    def get_fan_rpm(self, image_frame=None, position=None):
        #TODO Get the FAN RPM to push the ball to the target position
        #The slide moving up and down is where the ball is supposed to be
        pos = 0.0
        if image_frame is not None:
            observation = self.detect_ball(image_frame)
            x, y = self.get_kalman_filter_estimation(observation)
            if y >= 0:
                if y > self.max:
                    self.max = y
                if y < self.min:
                    self.min = y
            # print("min: {}, max: {}".format(self.min, self.max))
            # range is: 165 - 478, or 600-y is: 122 - 435
            # target is 121
            range = 485 - 0
            pos = (float(485 - y)) / (485 - 0)  # scaled position
            self.detected_pos = pos
        if position != None:
            pos = position
        output = 0.0
        p_error = 0.0
        d_error = 0.0
        i_error = 0.0

        target_pos = self.target_pos
        target_vel = 0.0
        self.error_pos = target_pos - pos
        error_vel = 0.0
        # print('detected at: {}, {}'.format(x, y))
        t = time.time()
        fan_rpm = 0
        if self.last_t is not None:
            fan_rpm = 1

        self.last_t = t
        self.last_pos = pos
        self.last_error_pos = self.error_pos
        # print('p_e: {:10.4f}, d_e: {:10.4f}, i_e: {:10.4f}, output: {:10.4f}'.format(p_error, d_error, i_error, output))
        return fan_rpm



def run_simulator(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target, validation_mode, save_mode, noisy_mode):
    global xdata
    global ydata
    env = simulation.env(PID_controller, graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target)
    if validation_mode:
        env.run_validation(noisy_mode, save_mode)
    else:
        env.run(noisy_mode)

def run_pid_plotter(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target, headless_mode):
    try:
        plotter = pid_plotter_vispy.PIDPlotter(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target)
        plotter.run()
    except Exception as e:
        if not headless_mode:
            print(e)
    return

if __name__ == '__main__':
    save_mode = False
    headless_mode = False
    if len(sys.argv) > 1:
        if sys.argv[1] == 'headless':
            print("Headless mode activated. ")
            os.environ['SDL_VIDEODRIVER'] = 'dummy'
            pygame.init()
            pygame.display.set_mode((1, 1))
            save_mode = True
            headless_mode = True

    print("welcome to the red ball simulator")
    validation_mode = False
    noisy_mode = False
    exit = False
    while not exit:
        print('v - Validation Mode')
        print('vn - Validation Noisy Mode')

        print('vs - Validation Save Video Mode')
        print('quit - Exit')

        inputString = input("Select Job To Run: ")
        # inputString = "v 0.5"
        # inputString = "e"
        commands = inputString.split(";")
        for command in commands:
            argList = command.strip()
            argList = argList.split(" ")
            job = argList[0]
            if job == 'v' or job == 'vs' or job == 'vn':
                validation_mode = True
            if job == 'vs':
                save_mode = True

            if job == "vn":
                noisy_mode = True

            if job == "quit":
                quit()

            if job == 'v' or job == 'vs' or job=='vn':
                # graph_lock = Lock()
                max_size = 432000 #1 hour at 120 fps
                graph_time = multiprocessing.Array('d', max_size)
                graph_position = multiprocessing.Array('d', max_size)
                graph_error = multiprocessing.Array('d', max_size)
                graph_fan = multiprocessing.Array('d', max_size)
                graph_target = multiprocessing.Array('d', max_size)
                graph_index = multiprocessing.Value('i')
                graph_index.value = 0

                processes = []
                process_plotter = Process(target=run_pid_plotter,
                                          args=(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target, headless_mode))
                process_plotter.start()
                processes.append(process_plotter)

                process_sim = Process(target=run_simulator, args=(graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target, validation_mode, save_mode, noisy_mode))
                process_sim.start()
                processes.append(process_sim)

                try:
                    for process in processes:
                        process.join()
                except Exception as e:
                    print("test")
                    print(e)
                print("Exiting Main Thread")


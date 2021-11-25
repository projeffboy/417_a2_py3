__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

import simulation_obj
import pid_plotter
import pygame
from multiprocessing import Lock
from interface_obj import *
from helpers import *
from PIL import Image
import cv2

class env:
    def __init__(self, PID_controller, graph_index=None, graph_time=None, graph_position=None, graph_error=None, graph_fan=None, graph_target=None):
        self.graph_index,  self.graph_time, self.graph_position, self.graph_error, self.graph_fan, self.graph_target = graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target

        self.run_pid = True

        # world settings
        self.gravity = 9.81
        self.elasticity = 0.9
        self.mass_density_of_air = 1.225  # kg/m^3
        # self.mass_density_of_air = 100  # kg/m^3
        # https://met213.tech.purdue.edu/French/Example%20Problems/Ping%20Pong%20Ball%20Drop%20-%20Published%20Article.pdf
        # http://aero-comlab.stanford.edu/Papers/AIAA-2011-3668-697.pdf
        self.drag_coefficient = 0.65

        # ball settings
        self.ball_radius = 0.02  # 20 mm radius
        self.ball_mass = 0.0027  # 2.7 grams for ping pong ball

        # graphing settings
        self.t_series = []
        self.y_series = []
        self.v_series = []
        self.fan_series = []

        self.target_series = []
        self.real_pos = []
        self.real_error = []

        # pygame settings
        self.target_fps = 120.0
        self.dt = 1.0 / self.target_fps
        self.world_scale = 600.0  # pixels/meter
        self.sim_win_height = 600
        self.sim_win_width = 337
        self.win_height = 600
        self.win_width = 680

        self.world_width = float(self.sim_win_width) / self.world_scale
        self.world_height = float(self.sim_win_height) / self.world_scale

        #initialize pygame
        # self.screen = pygame.display.set_mode((self.sim_win_width, self.sim_win_height))
        self.screen = pygame.display.set_mode((self.win_width, self.win_height))
        pygame.display.set_caption('Red Ball Simualator')
        pygame.font.init()
        font = pygame.font.Font('resources/COMIC.TTF', 25)
        textsurface = font.render('Loading, please wait...', False, (255, 0, 0))
        self.screen.blit(textsurface, (5, self.sim_win_height / 2))
        pygame.display.flip()

        # add simulation objects
        self.start_height = self.ball_radius + 0.0005
        self.ball = simulation_obj.Ball(self.ball_radius, self.ball_mass, self.world_width / 2, self.start_height, self)  # position in meters
        self.fan = simulation_obj.Fan(self)
        pid_target_pos = 0.0
        self.pid = PID_controller(pid_target_pos)

        # add interface objects
        self.slider_target = VSlider(self.screen, 'Target', 0.0, 0.0, 1.0, (360, 25), 550)
        self.slider_kp = VSlider(self.screen, 'Kp', self.pid.Kp, 0.0, 50000, (425, 25), 550)
        self.slider_ki = VSlider(self.screen, 'Ki', self.pid.Ki, 0, 10000, (490, 25), 550)
        self.slider_kd = VSlider(self.screen, 'Kd', self.pid.Kd, 0, 10000, (555, 25), 550)
        self.slider_bias = VSlider(self.screen, 'Bias', self.pid.bias, 0.0, 3000, (620, 25), 550)
        self.slides = [self.slider_target, self.slider_kp, self.slider_ki, self.slider_kd, self.slider_bias]

        return

    def reset(self):
        self.ball.reset(self.world_width / 2, self.start_height)
        self.ball.display()
        self.pid.reset()
        self.fan.set_rpm(0)
        self.slider_target.val = 0.0
        self.pid.target_pos = 0.0

        self.t_series = []
        self.y_series = []
        self.v_series = []
        self.fan_series = []

        self.target_series = []
        self.real_pos = []
        self.real_error = []

        if self.graph_index is not None:
            for i in range(self.graph_index.value):
                self.graph_time[i] = 0
                self.graph_position[i] = 0
                self.graph_error[i] = 0
                self.graph_fan[i] = 0
                self.graph_target[i] = 0
            self.graph_index.value = 0


    def run(self, noisy_mode, validation_mode=False, save_mode=False, target_height=0.0):
        running = True
        start_sim = True
        plot = False
        mouse_down = False
        clock = pygame.time.Clock()
        self.ball.set_noisy(noisy_mode)
        mouse_fan = 0.0
        experiment_time = 0.0

        random_steps = 0
        random_interval = 150
        random_frames = []
        sampled_target_height = 0

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        self.fan.set_rpm(1900.0)
                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_UP:
                        self.fan.set_rpm(0.0)
                    if event.key == pygame.K_s:
                        start_sim = True
                    if event.key == pygame.K_g:
                        # running = False
                        pid_plotter.plot_matplotlib(self.t_series, self.target_series, self.y_series, self.real_pos,
                                                    self.real_error,
                                                    self.fan_series)
                        plot = True
                    if event.key == pygame.K_r: #reset
                        self.reset()
                    if event.key == pygame.K_0:
                        self.pid.target_pos = 0.0
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_1:
                        self.pid.target_pos = 0.38
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_2:
                        self.pid.target_pos = 0.41
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_3:
                        self.pid.target_pos = 0.44
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_4:
                        self.pid.target_pos = 0.47
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_5:
                        self.pid.target_pos = 0.5
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_6:
                        self.pid.target_pos = 0.53
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_7:
                        self.pid.target_pos = 0.58
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_8:
                        self.pid.target_pos = 0.61
                        self.slider_target.val = self.pid.target_pos
                    if event.key == pygame.K_9:
                        self.pid.target_pos = 0.64
                        self.slider_target.val = self.pid.target_pos

                if event.type == pygame.MOUSEBUTTONDOWN:
                    pos = pygame.mouse.get_pos()
                    mouse_down = True
                    for slide in self.slides:
                        if slide.button_rect.collidepoint(pos):
                            slide.hit = True
                if event.type == pygame.MOUSEBUTTONUP:
                    mouse_down = False
                    self.fan.set_rpm(0.0)
                    for slide in self.slides:
                        slide.hit = False

            if validation_mode:
                if experiment_time >= 3.0: #run inpulse after 3 seconds
                    if random_steps == 0:
                        old_sampled_target = sampled_target_height
                        sampled_target_height = np.random.uniform(0.0, 1.25)
                    change = sampled_target_height - old_sampled_target
                    self.slider_target.val = self.slider_target.val + change * 1/random_interval
                    self.pid.set_target(self.slider_target.val + change * 1/random_interval)
                    random_steps += 1
                    if random_steps == random_interval:
                        random_steps = 0
                        np.random.randint(15, 150)

                    if random_steps % 6 == 0:
                        frame = cv2.cvtColor(self.ball.current_frame, cv2.COLOR_BGR2RGB)
                        random_frames.append(np.expand_dims(np.copy(frame), axis=0))

                if experiment_time >= 15:
                    if save_mode:
                        random_frames = np.concatenate(random_frames, axis=0).astype(np.uint8)
                        np.save("../output/sample_video.npy", random_frames)
                        imgs = [Image.fromarray(img, mode="RGB") for img in random_frames]
                        imgs[0].save("../output/result.gif", save_all=True, append_images=imgs[1:], duration=30, loop=0)

                    running = False
                    #print some stats
                    idx = find_nearest_index(self.t_series, self.t_series[-1]-2.0) #index of two seconds ago
                    detection_error = np.abs(np.array(self.real_pos[idx:]) - np.array(self.y_series[idx:]))
                    steadystate_error = np.abs(np.array(self.target_series[idx:]) - np.array(self.real_pos[idx:]))
                    print(('Average detection is: {}'.format(np.mean(detection_error))))
                    print(('Average stead state error is: {}'.format(np.mean(steadystate_error))))
                    #show and save plot
                    pid_plotter.plot_matplotlib(self.t_series, self.target_series, self.y_series, self.real_pos, self.real_error,
                                                self.fan_series)


            if mouse_down:
                (mouseX, mouseY) = pygame.mouse.get_pos()
                if mouseX <= 337:
                    # set fan thrust proportional to mouse position
                    mouse_fan = float(self.sim_win_height - mouseY) / self.sim_win_height
                    self.fan.set_rpm(2000.0 * mouse_fan)

            if start_sim:
                if len(self.t_series) == 0:
                    self.t_series.append(0)
                    self.target_series.append(self.pid.target_pos)
                    self.y_series.append(self.pid.detected_pos)
                    self.v_series.append(0)
                    self.fan_series.append(self.fan.rpm)
                    self.real_pos.append(self.ball.pos[1])
                    self.real_error.append(self.pid.target_pos - self.ball.pos[1])
                    if self.graph_index is not None:
                        self.graph_time[self.graph_index.value] = 0
                        self.graph_position[self.graph_index.value] = self.pid.detected_pos
                        self.graph_error[self.graph_index.value] = self.pid.target_pos - self.pid.detected_pos
                        self.graph_fan[self.graph_index.value] = self.fan.rpm
                        self.graph_target[self.graph_index.value] = self.pid.target_pos
                        self.graph_index.value += 1
                else:
                    new_time = self.t_series[len(self.t_series) - 1] + self.dt
                    self.t_series.append(new_time)
                    self.target_series.append(self.pid.target_pos)
                    self.y_series.append(self.pid.detected_pos)
                    self.v_series.append(self.ball.vel[1])
                    self.fan_series.append(self.fan.rpm)
                    self.real_pos.append(self.ball.pos[1])
                    self.real_error.append(self.pid.target_pos - self.ball.pos[1])
                    if self.graph_index is not None:
                        self.graph_time[self.graph_index.value] = new_time
                        self.graph_position[self.graph_index.value] = self.pid.detected_pos
                        self.graph_error[self.graph_index.value] = self.pid.target_pos - self.pid.detected_pos
                        self.graph_fan[self.graph_index.value] = self.fan.rpm
                        self.graph_target[self.graph_index.value] = self.pid.target_pos
                        self.graph_index.value += 1

                # physics loop here
                force_external = [0.0, self.fan.force]
                self.ball.update_physics(self.dt, force_external)
                self.screen.fill((0, 0, 0))
                self.ball.display()
                if self.run_pid:
                    pid_output = 0.0
                    if self.ball.current_frame is not None:
                        pid_output = self.pid.get_fan_rpm(self.ball.current_frame)
                    else:
                        pid_output = self.pid.get_fan_rpm(position=self.ball.pos[1])
                    self.fan.set_rpm(pid_output)
                    # print('pid output {}'.format(pid_output))
            else:
                self.screen.fill((0, 0, 0))
                # pygame.font.init()
                # myfont = pygame.font.Font('COMIC.TTF', 30)
                textsurface = self.myfont.render('Ready, press S to Start', False, (0, 255, 0))
                self.screen.blit(textsurface, (5, self.sim_win_height / 2))

            # Move and draw slides
            for slide in self.slides:
                if slide.hit:
                    slide.move()
                slide.draw()

            self.pid.Kp = self.slider_kp.val
            self.pid.Ki = self.slider_ki.val
            self.pid.Kd = self.slider_kd.val
            self.pid.bias = self.slider_bias.val

            self.pid.target_pos = self.slider_target.val

            pygame.display.flip()
            clock.tick(self.target_fps)
            experiment_time += self.dt

            # if plot:
            #     pid_plotter.plot_matplotlib(self.t_series, self.y_series, self.v_series, self.fan_series)
        return


    def run_validation(self, noisy_mode, save_mode=False):
        self.run(noisy_mode, validation_mode=True, save_mode=save_mode)
        return

    # def run_random(self):
    #     self.run(random_mode=True)
    #     return
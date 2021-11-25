__author__ = "Travis Manderson"
__copyright__ = "Copyright 2018, Travis Manderson"

import sys
import numpy as np
from vispy import app, scene
from vispy.visuals.transforms import (STTransform, LogTransform,
                                      MatrixTransform, PolarTransform)
from vispy.scene.visuals import Text
import threading
import time
from multiprocessing import Lock
from vispy.geometry import Rect


class PIDPlotter:
    def __init__(self, graph_index=None, graph_time=None, graph_position=None, graph_error=None, graph_fan=None, graph_target=None):
        self.graph_index, self.graph_time, self.graph_position, self.graph_error, self.graph_fan, self.graph_target = graph_index, graph_time, graph_position, graph_error, graph_fan, graph_target
        self.last_graph_index = graph_index.value - 1
        # vertex positions of data to draw
        # N = 100
        # self.pos = np.zeros((N, 2), dtype=np.float32)
        # x_lim = [0., 100.]
        # y_lim = [-2., 2.]
        # self.pos[:, 0] = np.linspace(x_lim[0], x_lim[1], N)
        # self.pos[:, 1] = np.random.normal(size=N)
        #
        # # color array
        # self.color = np.ones((N, 4), dtype=np.float32)
        # self.color[:, 0] = np.linspace(0, 1, N)
        # self.color[:, 1] = self.color[::-1, 0]

        self.line_opacity = 0.8

        self.data_pos = np.zeros((0, 2), dtype=np.float32)
        self.data_pos_error = np.zeros((0, 2), dtype=np.float32)
        self.color = np.ones((0, 4), dtype=np.float32)
        self.data_fan = np.zeros((0, 2), dtype=np.float32)
        self.data_target = np.zeros((0, 2), dtype=np.float32)

        self.canvas = scene.SceneCanvas(keys='interactive', show=True, title='Ping-Pong Ball PID Oscilloscope')
        grid = self.canvas.central_widget.add_grid(spacing=0)

        #https://github.com/vispy/vispy/blob/master/vispy/scene/cameras/panzoom.py
        self.viewbox_fan = grid.add_view(row=0, col=1, camera='panzoom', parent=self.canvas.scene)
        self.viewbox_pos = grid.add_view(row=0, col=1, camera='panzoom', parent=self.canvas.scene)

        # self.viewbox.camera.aspect = 1
        # self.viewbox2.camera.aspect = 1  # no auto-scale
        # self.viewbox.camera.link(self.viewbox2.camera)

        self.viewbox_pos.events.resize.connect(self.on_resize)

        # add some axes
        x_axis = scene.AxisWidget(orientation='bottom')
        x_axis.stretch = (1, 0.1)
        grid.add_widget(x_axis, row=1, col=1)
        x_axis.link_view(self.viewbox_pos)

        y_axis = scene.AxisWidget(orientation='left', text_color='red')
        y_axis.stretch = (0.1, 1)
        grid.add_widget(y_axis, row=0, col=0)
        y_axis.link_view(self.viewbox_pos)

        y2_axis = scene.AxisWidget(orientation='right', text_color='green')
        y2_axis.stretch = (0.1, 10)
        grid.add_widget(y2_axis, row=0, col=2)
        y2_axis.link_view(self.viewbox_fan)
        # y2_axis.transform = STTransform(scale=(1, 0.2, 1))

        t1 = Text('Detected Ball Position [m]', parent=self.canvas.scene, color='red')
        t1.font_size = 10
        # t1.pos = self.canvas.size[0] // 2, self.canvas.size[1] // 3
        t1.pos = 167, 20
        t2 = Text('Detected Ball Position Error [m]', parent=self.canvas.scene, color='yellow')
        t2.font_size = 10
        t2.pos = 185, 35
        t3 = Text('Fan RPM [RPM]', parent=self.canvas.scene, color='green')
        t3.font_size = 10
        t3.pos = 130, 50
        t4 = Text('Target Height [m]', parent=self.canvas.scene, color='blue')
        t4.font_size = 10
        t4.pos = 135, 65

        #
        # t2 = Text('Text in viewbox (18 pt)', parent=self.viewbox.scene, color='green',
        #           rotation=30)
        # t2.font_size = 18
        # t2.pos = 0.5, 0.3

        # add a line plot inside the viewbox
        self.line_pos = None
        self.line_pos_error = None
        self.line_target = None
        self.line_fan = None
        #self.line = scene.Line(self.pos, self.color, parent=self.viewbox.scene)

        # add horizontal lines
        hor_line1 = scene.InfiniteLine(0.0, [0.5, 0.5, 0.5, 0.5], vertical=False, parent=self.viewbox_pos.scene)

        # auto-scale to see the whole line.
        self.viewbox_pos.camera.set_range()

        self.auto_scale = True
        self.auto_scroll = False
        self.canvas.events.mouse_press.connect(self.on_mouse_press)
        self.canvas.events.mouse_release.connect(self.on_mouse_release)
        self.canvas.events.key_press.connect(self.on_key_press)

        return

    def on_resize(self, event):
        scale = 10000
        r1 = self.viewbox_pos.camera.rect
        self.viewbox_fan.camera.rect = Rect(r1.left, r1.bottom * scale, r1.width, r1.height * scale)
        return

    def on_mouse(self, event):
        self.viewbox_fan.viewbox_mouse_event(event)
        return


    def on_mouse_move(self, event):
        # print(event.press_event)
        return

    def on_key_press(self, event):
        #print(event.key.name)
        if event.key.name == 'R' or event.key.name == 'Escape':
            self.auto_scale = True
        if event.key.name == 'S':
            self.auto_scroll = not self.auto_scroll
            self.auto_scale = True
        return

    def on_mouse_press(self, event):
        self.auto_scale = False
        # print(event.pos)
        return

    def on_mouse_release(self, event):
        # print(event.pos)
        return

    def test_update(self, ev):
        new_pos = np.array([[self.data_pos[-1, 0] + 1, np.random.normal(size=1)[0]]])
        self.data_pos = np.append(self.data_pos, new_pos, axis=0)

        new_color = np.array([[1, 0, 0, 0.5]])
        self.color = np.append(self.color, new_color, axis=0)

        # color = np.roll(color, 1, axis=0)
        self.line_pos.set_data(pos=self.data_pos, color=self.color)

        if self.auto_scale == True:
            # auto-scale to see the whole line.
            self.viewbox_pos.camera.set_range(x=(0, np.shape(self.data_pos)[0]), y=(-2, 2))

        # self.pos[:, 1] = np.random.normal(size=100)
        # color = np.roll(self.color, 1, axis=0)
        # self.line.set_data(pos=self.pos, color=self.color)

        return

    def update(self, ev=None):
        #todo: change this to append instead of overwrite
        # self.graph_time = graph_time
        # self.graph_position = graph_position
        # vertex positions of data to draw
        if self.graph_time is not None and self.graph_position is not None:
            # self.graph_lock.acquire()
            if self.graph_index.value > 1: #at least 2 elements to start graphing
                s = self.last_graph_index + 1
                e = self.graph_index.value
                self.last_graph_index = e
                N = e - s
                if N < 0: #this would be due to a reset
                    s = 0
                    self.last_graph_index = e
                    self.data_pos = np.zeros((0, 2), dtype=np.float32)
                    self.data_pos_error = np.zeros((0, 2), dtype=np.float32)
                    self.color = np.ones((0, 4), dtype=np.float32)
                    self.data_fan = np.zeros((0, 2), dtype=np.float32)
                    self.data_target = np.zeros((0, 2), dtype=np.float32)

                    N = e - s
                if N > 0:
                    new_pos = np.zeros((N, 2), dtype=np.float32)
                    new_pos_error = np.zeros((N, 2), dtype=np.float32)
                    new_data_fan = np.zeros((N, 2), dtype=np.float32)
                    new_data_target = np.zeros((N, 2), dtype=np.float32)
                    new_color = np.ones((N, 4), dtype=np.float32)
                    for i in range(N):
                        new_pos[i, 0] = self.graph_time[s+i]
                        new_pos[i, 1] = self.graph_position[s+i]
                        new_pos_error[i, 0] = self.graph_time[s + i]
                        new_pos_error[i, 1] = self.graph_error[s + i]
                        new_data_fan[i, 0] = self.graph_time[s + i]
                        new_data_fan[i, 1] = self.graph_fan[s + i]
                        new_data_target[i, 0] = self.graph_time[s + i]
                        new_data_target[i, 1] = self.graph_target[s + i]
                        new_color[i] = [1, 0, 0, self.line_opacity]
                    # self.graph_lock.release()

                    self.data_pos = np.append(self.data_pos, new_pos, axis=0)
                    self.data_pos_error = np.append(self.data_pos_error, new_pos_error, axis=0)
                    self.color = np.append(self.color, new_color, axis=0)
                    self.data_fan = np.append(self.data_fan, new_data_fan, axis=0)
                    self.data_target = np.append(self.data_target, new_data_target, axis=0)
                    # # color array
                    # self.color = np.ones((N, 4), dtype=np.float32)
                    # self.color[:, 0] = np.linspace(0, 1, N)
                    # color = np.roll(color, 1, axis=0)

                    auto_scrolling_lines = 500
                    if self.line_pos is None:
                        self.line_pos_error = scene.Line(self.data_pos_error, [1, 1, 0, self.line_opacity], parent=self.viewbox_pos.scene)
                        self.line_target = scene.Line(self.data_target, [0, 0, 1, self.line_opacity], parent=self.viewbox_pos.scene, width=2)
                        self.line_fan = scene.Line(self.data_fan, [0, 1, 0, self.line_opacity], parent=self.viewbox_fan.scene, )
                        self.line_pos = scene.Line(self.data_pos, self.color, parent=self.viewbox_pos.scene)
                        # self.line_fan = scene.visuals.Line(self.data_fan, [0, 1, 0, 1], parent=self.viewbox.scene, )
                        # self.line_fan.transform = STTransform(scale=(1, 5, 1))
                    else:
                        i = 0
                        j = self.data_pos.shape[0]
                        if j > auto_scrolling_lines and self.auto_scale and self.auto_scroll:
                            i = j - auto_scrolling_lines

                        self.line_pos_error.set_data(pos=self.data_pos_error[i:j, :], color=[1, 1, 0, self.line_opacity])
                        self.line_target.set_data(pos=self.data_target[i:j, :], color=[0, 0, 1, self.line_opacity])
                        self.line_fan.set_data(pos=self.data_fan[i:j, :], color=[0, 1, 0, self.line_opacity])
                        self.line_pos.set_data(pos=self.data_pos[i:j, :], color=self.color[i:j, :])


                    if self.auto_scale == True:
                        i = 0
                        j = self.data_pos.shape[0]
                        if j > auto_scrolling_lines and self.auto_scale and self.auto_scroll:
                            i = j - auto_scrolling_lines
                        # auto-scale to see the whole line.
                        self.viewbox_pos.camera.set_range(x=(self.data_pos[i, 0], self.data_pos[-1, 0]), y=(-1.1, 1.1))
                        # self.viewbox_fan.camera.set_range(x=(self.pos[0, 0], self.pos[-1, 0]), y=(-0.0, 0.2))


    def run(self):
        # # timer.connect(self.test_update)
        # timer.connect(self.test_update2)
        # timer.start(0.1)
        timer = app.Timer('auto', connect=self.update, start=True)
        app.run()
        return

if __name__ == '__main__' and sys.flags.interactive == 0:
    pid_plotter = PIDPlotter()
    pid_plotter.run()




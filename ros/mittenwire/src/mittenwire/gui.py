#!/usr/bin/env python3

import sdl2
import sdl2.ext
import numpy as np
from OpenGL import GL
import OpenGL

sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO)


class Window:

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.sdl_window.hide()
        del self.sdl_window

    def __init__(self, name, width, height):
        self.width = width
        self.height = height
        self.sdl_window = sdl2.ext.Window(name,
                                          size=(width, height),
                                          flags=sdl2.SDL_WINDOW_HIDDEN | sdl2.SDL_WINDOW_RESIZABLE | sdl2.SDL_WINDOW_OPENGL
                                          )
        self.shown = False
        self.context = sdl2.SDL_GL_CreateContext(self.sdl_window.window)

    def begin(self):

        if not self.shown:
            self.shown = True
            self.sdl_window.show()

        self.width = self.sdl_window.size[0]
        self.height = self.sdl_window.size[1]

        GL.glClearColor(0.0, 0.0, 0.3, 1)
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

    def end(self):

        pass

    def swap(self):

        sdl2.SDL_GL_SwapWindow(self.sdl_window.window)

        # sdl2.ext.

        # surface = self.sdl_window.get_surface()
        # dst = sdl2.ext.pixels3d(surface)

        # dst[:, :, :] = 0

        # src = np.rot90(src)
        # src = src[::-1, :, :]

        # w = min(src.shape[0], dst.shape[0])
        # h = min(src.shape[1], dst.shape[1])

        # sx = max(0, src.shape[0] - w) // 2
        # sy = max(0, src.shape[1] - h) // 2

        # dx = max(0, dst.shape[0] - w) // 2
        # dy = max(0, dst.shape[1] - h) // 2

        # dst[dx:w + dx, dy:h + dy, 0:3] = src[sx:w + sx, sy:h + sy, 0:3]

        # self.width = dst.shape[0]
        # self.height = dst.shape[1]

        # if not self.shown:
        #     self.shown = True
        #     self.sdl_window.show()

        # self.sdl_window.refresh()

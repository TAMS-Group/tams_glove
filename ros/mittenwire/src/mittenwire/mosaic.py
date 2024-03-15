#!/usr/bin/env python3

import numpy as np
import cv2
import math


class Mosaic:

    def __init__(self, images, mosaic_size, cell_size):

        count = len(images)

        def cell_scale(width, height, count, cols):
            rows = math.ceil(count / cols)
            cell_max_width = mosaic_size[1] / cols
            cell_max_height = mosaic_size[0] / rows
            scale = min(
                cell_max_width / width,
                cell_max_height / height
            )
            return scale

        cells = []

        if count > 0:

            cols, scale = sorted([(cols, cell_scale(cell_size[1], cell_size[0], count, cols))
                                  for cols in range(1, count + 1)], key=lambda x: x[1])[-1]
            rows = math.ceil(count / cols)

            for i in range(count):

                img = images[i]

                col = i % cols
                row = i // cols

                x = mosaic_size[1] * (col * 2 + 1) // (cols * 2)
                y = mosaic_size[0] * (row * 2 + 1) // (rows * 2)

                s = cell_scale(
                    img.shape[1], img.shape[0], count, cols) * 0.99

                w = int(math.floor(img.shape[1] * s))
                h = int(math.floor(img.shape[0] * s))

                x = x - w // 2
                y = y - h // 2

                cells.append((x, y, w, h))

        self.mosaic_size = mosaic_size
        self.cell_size = cell_size
        self.cells = cells
        self.images = images

    def render(self, dtype):

        cells = self.cells
        mosaic_size = self.mosaic_size
        images = self.images

        mosaic = np.zeros((mosaic_size[0], mosaic_size[1], 3), dtype=dtype)

        for i in range(len(cells)):

            cell = cells[i]
            x = cell[0]
            y = cell[1]
            w = cell[2]
            h = cell[3]

            img = images[i]
            img = cv2.resize(img, (w, h), cv2.INTER_AREA)

            if img.ndim == 3 and img.shape[2] == 3:
                mosaic[y:y + h, x:x + w] = img

            if img.ndim == 2 or (img.ndim == 3 and img.shape[2] == 1):
                mosaic[y:y + h, x:x + w, 0] = img
                mosaic[y:y + h, x:x + w, 1] = img
                mosaic[y:y + h, x:x + w, 2] = img

        return mosaic

    def map(self, i, x, y):
        cell = self.cells[i]
        cx = cell[0]
        cy = cell[1]
        cw = cell[2]
        ch = cell[3]
        return (
            int(round(cx + x * cw / self.images[i].shape[1])),
            int(round(cy + y * ch / self.images[i].shape[0])),
        )


def make_mosaic(images, mosaic_size, cell_size, dtype):
    m = Mosaic(images, mosaic_size, cell_size)
    return m.render(dtype)

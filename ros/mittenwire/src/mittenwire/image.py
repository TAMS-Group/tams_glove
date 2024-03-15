#!/usr/bin/env python3

import numpy as np
import cv2
from . import profiler
import pymittenwire


# def preprocess(img):

#     with profiler.Profiler("convert", 0):
#         img = img.astype(np.float32) * (1.0 / 255)
#         img = np.square(img)
#         img = (img * 65535).astype(np.uint16)

#     with profiler.Profiler("debayer", 0):
#         img = cv2.cvtColor(img, cv2.COLOR_BAYER_GR2RGB)

#     return img


# def postprocess(img):

#     # img = img.astype(np.float32) * (1.0 / 65535)

#     # with profiler.Profiler("tonemap", 1):
#     #     pymittenwire.tonemap2(img, 5, 1.3)

#     # with profiler.Profiler("srgb", 1):
#     #     pymittenwire.linear2srgb(img)

#     # with profiler.Profiler("ofmt", 1):
#     #     img = img * 255
#     #     img[img > 255] = 255
#     #     img = img.astype(np.uint8)
#     #     # img = cv2.convertScaleAbs(img, img, alpha=1)

#     return img

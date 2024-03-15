
import cv2
import numpy as np


def make_colors(n):
    colors = cv2.applyColorMap(np.linspace(
        0, 255, n + 1, dtype=np.uint8)[:-1], cv2.COLORMAP_HSV)
    colors = colors.astype(np.float32) * (1.0 / 255)

    colors = [list(l[0]) for l in colors]
    return colors


def point2dict(p):
    return {
        "x": float(p[0]),
        "y": float(p[1]),
        "z": float(p[2]),
    }


def dict2point(p):
    return [
        float(p["x"]),
        float(p["y"]),
        float(p["z"]),
    ]


def save_table(simtac, name, headerprefix=""):
    simtac = np.array(simtac)
    print(simtac.shape, simtac)
    simtac = np.reshape(simtac, [simtac.shape[0], -1])
    if len(simtac) > 0:
        headers = ["i"] + [headerprefix+str(i) for i in range(simtac.shape[1])]
        simtac = np.hstack(
            [np.reshape(np.array(range(0, len(simtac))), [-1, 1]), simtac])
        np.savetxt(name, simtac, header=" ".join(headers), comments="")


def vector_message_to_array(msg):
    return np.array([msg.x, msg.y, msg.z])

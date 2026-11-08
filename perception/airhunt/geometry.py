import numpy as np


def get_cam_intr(hfov, img_height, img_width):
    hfov_rad = hfov * np.pi / 180
    vfov_rad = 2 * np.arctan(np.tan(hfov_rad / 2) * img_height / img_width)
    fx = 1.0 / np.tan(hfov_rad / 2.0) * img_width / 2.0
    fy = 1.0 / np.tan(vfov_rad / 2.0) * img_height / 2.0
    cx = img_width // 2
    cy = img_height // 2
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

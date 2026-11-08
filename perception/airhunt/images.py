import numpy as np
from PIL import Image


def resize_image(image, target_h, target_w):
    image = Image.fromarray(image)
    image = image.resize((target_w, target_h))
    return np.array(image)

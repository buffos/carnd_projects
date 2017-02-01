import numpy as np
import cv2


# noinspection PyPep8Naming
def side_Kernel(size=3, left=True):
    if size % 2 == 0:
        return None  # no even arguments

    if left is True:
        sideColumn = 0
    else:
        sideColumn = -1

    kernel_shape = (size, size)
    midpoint = int((size - 1) / 2)
    kernel = np.zeros(kernel_shape, dtype=np.float64)
    kernel[:, sideColumn] = 1
    kernel[midpoint, midpoint] = -size
    kernel /= size
    return kernel


# noinspection PyPep8Naming
def sideChannelFilter(channel, kernel_size):
    left_convolution = np.full_like(channel, -255., dtype=np.float)
    right_convolution = np.full_like(channel, -255., dtype=np.float)

    side_convolution = channel.copy().astype(np.float)

    for k in range(3, kernel_size + 2, 2):
        left_convolution = np.maximum(left_convolution, cv2.filter2D(side_convolution, -1, side_Kernel(k, left=True)))
        right_convolution = np.maximum(right_convolution, cv2.filter2D(side_convolution, -1, side_Kernel(k, left=True)))

    side_convolution += np.minimum(left_convolution, right_convolution)
    return side_convolution


# noinspection PyPep8Naming
def sideImageFilter(image, kernel_size):
    imageChannels = image.shape[2] if len(image.shape) == 3 else 1

    if imageChannels == 1:
        return sideChannelFilter(image, kernel_size)
    else:
        imageCopy = image.copy()
        for i in range(0, imageChannels):
            imageCopy[:, :, i] = sideChannelFilter(imageCopy[:, :, i], kernel_size).astype('uint8')

    return imageCopy


# noinspection PyPep8Naming
def sideImageFilterInChannel(image, kernel_size, channel=0):
    imageChannels = image.shape[2] if len(image.shape) == 3 else 1

    if imageChannels == 1:
        return sideChannelFilter(image, kernel_size)
    else:
        imageCopy = image.copy()
        imageCopy[:, :, channel] = sideChannelFilter(imageCopy[:, :, channel], kernel_size).astype('uint8')

    return imageCopy

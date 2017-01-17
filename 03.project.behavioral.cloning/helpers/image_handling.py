import cv2
import os
import glob
import numpy as np
from global_variables import *


# http://www.xavierdupre.fr/blog/2016-03-30_nojs.html
def make_video(images, out_video=None, fps=5, size=None, is_color=True, video_format="X264"):
    """
    Create a video from a list of images.

    :param      out_video:    output video
    :param      images:       list of images to use in the video
    :param      fps:          frame per second
    :param      size:         size of each frame
    :param      is_color:     color
    :param      video_format: see http://www.fourcc.org/codecs.php
    :return     see http://opencv-python-tutroals.readthedocs.org/
                    en/latest/py_tutorials/py_gui/py_video_display/py_video_display.html

    The function relies on http://opencv-python-tutroals.readthedocs.org/en/latest/.
    By default, the video will have the size of the first image.
    It will resize every image to this size before adding them to the video.
    """
    fourcc = cv2.VideoWriter_fourcc(*video_format)
    vid = None
    for image in images:
        if not os.path.exists(image):
            raise FileNotFoundError(image)
        img = cv2.imread(image)
        if vid is None:
            if size is None:
                size = img.shape[1], img.shape[0]
            vid = cv2.VideoWriter(out_video, fourcc, float(fps), size, is_color)
        if size[0] != img.shape[1] and size[1] != img.shape[0]:
            img = cv2.resize(img, size)
        vid.write(img)
    vid.release()
    return vid


def make_videos(paths, path_key, camera):
    """
    Create video from images
    :param paths: the dictionary that holds the path to each lane pass
    :param path_key: which lane we want to get images from
    :param camera: which camera
    :return:
    """
    # USAGE: make_videos(data_paths, 'left', 'center')
    image_list = get_image_list(paths[path_key] + 'img/' + camera + '*.jpg')
    # print(path_key + '_' + camera + '.avi')
    make_video(image_list,
               out_video=path_key + '_' + camera + '.avi',
               fps=20,
               video_format="FMP4")


def crop_image_to_square(img, crop_size=None, position=None):
    """
    Creates a square crop of the original image
    :param img: the original image
    :param crop_size: If not provided it is the smallest of the two sides
    :param position: None is crop from center. 0 is crop from left, 1 is crop from right
    :return: the cropped version of the image
    """
    if crop_size is None:
        crop_size = min(img.shape[0], img.shape[1])  # finding the smallest size
    if position is None:
        start_y = int((img.shape[0] - crop_size) / 2)  # the starting pixel to crop is from the center - crop_size/2
        start_x = int((img.shape[1] - crop_size) / 2)  # in both axis
        cropped = img[start_y:start_y + crop_size, start_x:start_x + crop_size]
    if position == 0:
        start_y = int((img.shape[0] - crop_size) / 2)
        start_x = 0
        cropped = img[start_y:start_y + crop_size, start_x:start_x + crop_size]
    if position == 1:
        start_y = int((img.shape[0] - crop_size) / 2)
        start_x = img.shape[1]
        cropped = img[start_y:start_y + crop_size, start_x - crop_size:start_x]
    return cropped


def crop_image_to_road(img, top_margin=0.35, bottom_margin=0.125):
    """
    Remove the sky (35% of the image) and car hood(12.5% from the bottom)
    :param img: the image to crop
    :param top_margin: the top % of the image to crop
    :param bottom_margin: the bottom % of the image to crop
    :return:
    """
    # chop 35% of top 12.5% bottom of image
    start_y = int(img.shape[0] * top_margin)
    end_y = int(img.shape[0] * (1 - bottom_margin))
    cropped = img[start_y:end_y, :]
    return cropped


def resize_image(image, new_x_size, new_y_size=None):
    """
    Resize an image into new dimensions
    :param image:
    :param new_x_size:
    :param new_y_size:
    :return:
    """
    # image.shape = (height, width) = (y,x)
    if new_y_size is None:
        new_y_size = int(new_x_size * image.shape[0] / image.shape[1])
    resized = cv2.resize(image, (new_x_size, new_y_size), interpolation=cv2.INTER_AREA)
    return resized


def adjust_image_brightness(image):
    """
    Adjust brightness (value) channel by compressing or expanding the values in that channel
    As a result we get a brighter or darker image
    :param image:
    :return:
    """
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # convert to Hue Saturation Value Channels
    random_brightness = np.random.uniform(0.40, 1.10)  # 0.40 and 1.10 originally
    image[:, :, 2] = image[:, :, 2] * random_brightness  # transform the brightness channel randomly
    image = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)  # return to RGB space
    return image


def equalize_brightness(image):
    """
    We make a blunt version of the original image by adding to its mean a percent of the initial variation
    :param image:
    :return:
    """
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # convert to Hue Saturation Value Channels
    image_mean = np.mean(image[:, :, 2])
    variation = np.random.uniform(0.3, 1)
    image[:, :, 2] = (image[:, :, 2] - image_mean) * variation + image_mean
    image = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)  # return to RGB space
    return image


def change_brightness(image):
    """
    Manipulate the brightness channel. This function was just an experiment gone wrong
    :param image:
    :return:
    """
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # convert to Hue Saturation Value Channels
    i_min = np.amin(image, 2)
    i_max = np.amax(image, 2)
    image[:, :, 2] = (i_max - i_min) / 2
    image = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)  # return to RGB space
    return image


def equalize_brightness_with_histogram(image):
    """
    Equalize brightness by using cv2 histogram function
    :param image:
    :return:
    """
    image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)  # convert to Hue Saturation Value Channels
    image[:, :, 2] = cv2.equalizeHist(image[:, :, 2])
    image = cv2.cvtColor(image, cv2.COLOR_HSV2RGB)  # return to RGB space
    return image


def preprocess_image(image, new_x_size, new_y_size=None, driving_mode=False):
    """
    Apply all the necessary actions to the image
    :param image:
    :param new_x_size:
    :param new_y_size:
    :param driving_mode: if on some actions may not apply
    :return:
    """
    image = crop_image_to_road(image)
    image = resize_image(image, new_x_size, new_y_size)
    if driving_mode is False:
        # image = change_brightness(image)
        image = adjust_image_brightness(image)
        # image = equalize_brightness(image)
    return image


def flip_image(image):
    """
    Flip the original image in the vertical axis
    :param image:
    :return:
    """
    return np.fliplr(image)


def load_image(file_name):
    """
    Loading an image from a file and returning the RGB version of the image
    :param file_name:
    :return:
    """
    img = cv2.imread(file_name, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    return img


def load_images(glob_pattern):
    filelist = glob.glob(glob_pattern)
    return np.array([np.array(load_image(f_name)) for f_name in filelist])


def load_images_with_keys(paths_dictionary, keylist, filelist):
    return np.array(
        [np.array(load_image(paths_dictionary[keylist[index]] + f_name)) for index, f_name in enumerate(filelist)]
    )


def load_images_from_file_list(path, filelist):
    return np.array(
        [np.array(load_image(path + f_name)) for index, f_name in enumerate(filelist)]
    )


def get_image_list(glob_pattern):
    return glob.glob(glob_pattern)

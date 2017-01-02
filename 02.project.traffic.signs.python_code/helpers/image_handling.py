import cv2
import numpy as np
import glob
from math import floor


# return a gray scale version of the original image
def grayscale(image):
    """
    create a grayscale version of the original image
    :param image: a colored image as an nparray
    :return: a grayscale version of the original image
    """
    converted = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return converted


# rescale image date from [0, 255] to [-0.5,0.5]
def normalize(image_data):
    """
    normalize image data by first scaling to [0,1] by dividing with 255
    and then centering the values around zero [-0.5, 0.5]
    :param image_data: an images as an nparray
    :return: normalized image as an nparray
    """
    return image_data / 255 - 0.5


# pre process the image by applying first grayscale and then normalizing
def pre_process(image):
    """
    provide a grayscale and normalized version of an image
    :param image: an image as an nparray
    :return: a grayscale and normalized image array
    """
    return normalize(grayscale(image))


def erase_pixels(image, percent=0.):
    x = image.shape[0]
    y = image.shape[1]

    # creating a black out channel for one channel and applying to all
    zero_one = np.random.choice([0, 1], size=x * y, p=[percent, 1 - percent])
    zero_one = np.reshape(zero_one, (x, y))
    if len(image.shape) >= 3:  # grayscale image
        ch = image.shape[2]
        for i in range(0, ch):
            image[:, :, i] = image[:, :, i] * zero_one
    else:
        image *= zero_one
    return image


def pre_process_dataset(sample, func=pre_process):
    """
    preprocess a sample set of images
    :param sample: an nparray holding many images
    :param func: the function to apply in each image
    :return: an nparray holding the images processed
    """
    return np.asarray([func(image) for image in sample])


def transform_image(img, ang_range, shear_range, trans_range, erase_percent=0.):
    """
    This function transforms images to generate new images.
    The function takes in following arguments,
    1- Image
    2- ang_range: Range of angles for rotation
    3- shear_range: Range of values to apply affine transform to
    4- trans_range: Range of values to apply translations over.

    A Random uniform distribution is used to generate different parameters for transformation
    """

    # taking care of the case where we are using grayscale images
    if len(img.shape) == 3:
        rows, cols, ch = img.shape
    else:
        rows, cols = img.shape

    # Rotation
    if ang_range != 0:
        ang_rot = np.random.uniform(ang_range) - ang_range / 2
        rotation_matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), ang_rot, 1)
        img = cv2.warpAffine(img, rotation_matrix, (cols, rows))

    # Translation
    if trans_range != 0:
        tr_x = trans_range * np.random.uniform() - trans_range / 2
        tr_y = trans_range * np.random.uniform() - trans_range / 2
        translation_matrix = np.float32([[1, 0, tr_x], [0, 1, tr_y]])
        img = cv2.warpAffine(img, translation_matrix, (cols, rows))

    # Shear
    if shear_range != 0:
        pts1 = np.float32([[5, 5], [20, 5], [5, 20]])

        pt1 = 5 + shear_range * np.random.uniform() - shear_range / 2
        pt2 = 20 + shear_range * np.random.uniform() - shear_range / 2

        pts2 = np.float32([[pt1, 5], [pt2, pt1], [5, pt2]])

        shear_matrix = cv2.getAffineTransform(pts1, pts2)
        img = cv2.warpAffine(img, shear_matrix, (cols, rows))

    if erase_percent > 0:
        return erase_pixels(img, erase_percent)
    else:
        return img


def generate_images_for_class(images, labels, image_class, target_value, transformations_per_image, erase_percent=0.):
    """
    generate new images for a specific sing class
    :param images: the image dataset
    :param labels: the corresponding labels dataset
    :param image_class: the sign class i want to generate new images for
    :param target_value: the number of new images i want to generate (for an even distribution)
    :param transformations_per_image: how many images should i create from a single image
    :param erase_percent: how many pixels should I erase from the original image
    :return: a tuple (new_images, new_labels)
    """
    # create a view of images of the selected class
    image_pool = images[labels[:] == image_class]
    # calculate the number of images to generate
    images_to_generate = int(floor((target_value - len(image_pool)) / transformations_per_image))
    # pick random images from the image pool
    images_to_transform = np.random.choice(len(image_pool), images_to_generate, replace=False)

    generated_images = []
    generated_labels = []

    for index in images_to_transform:
        for i in range(0, transformations_per_image):
            generated_images.append(transform_image(image_pool[index], 0, 10, 0, erase_percent))
            generated_labels.append(image_class)
    return generated_images, generated_labels


def generate_images(images, image_labels, sign_classes, images_per_class, transformations_per_image=10,
                    erase_percent=0.0):
    """
    Generate images for all sign classes as needed
    :param images: the image dataset
    :param image_labels: the corresponding labels dataset
    :param sign_classes: an array with all the available sign classes (one value per class)
    :param images_per_class: the number images that are currently in each sign class
    :param transformations_per_image: how many images should i create from a single image
    :param erase_percent: how many pixels should I erase from the original image
    :return: new_image_set, new_corresponding_labels
    """
    new_images = []
    new_labels = []

    for sign_class in sign_classes:
        xx, yy = generate_images_for_class(images,  # images np.array
                                           image_labels,  # image labels array
                                           sign_class,  # the names of the classes
                                           images_per_class,  # how many images to generate per class
                                           transformations_per_image,
                                           erase_percent
                                           )
        if len(xx) > 0:
            new_images.extend(xx)
            new_labels.extend(yy)
    images = np.concatenate((images, np.asarray(new_images)), axis=0)
    image_labels = np.concatenate((image_labels, np.asarray(new_labels)), axis=0)
    return images, image_labels


def load_image(file_name):
    img = cv2.imread(file_name, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img


def load_images(glob_pattern):
    filelist = glob.glob(glob_pattern)
    return np.array([np.array(load_image(f_name)) for f_name in filelist])

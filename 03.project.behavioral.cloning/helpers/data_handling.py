import pandas as pd
import numpy as np


def create_training_sets(num_inputs, test_set=True, train_size=0.7):
    """
    Create 3 indexes. training, validation and testing.
    We are 'dividing' the number of inputs in those 3 slots
    :param num_inputs: the number of image - zone pairs
    :param test_set: create a test set or just validation
    :param train_size:  the portion of training samples
    :return: three arrays containing indexes for training validation and testing image-zone pairs
    """
    from sklearn.model_selection import train_test_split
    n = np.arange(num_inputs)
    train_split, val_split = train_test_split(n, test_size=1 - train_size, random_state=0)
    if test_set is True:
        val_split, test_split = train_test_split(val_split, test_size=0.5, random_state=0)
        return train_split, val_split, test_split
    else:
        return train_split, val_split


def create_training_angles_from_data(filepath, only_center=True, delta_angle=0.30, fraction=0.1):
    """
    The function will get the data from a csv file and create 3 datasets (if only_center is False)
    The initial csv will be split based on the segregation angle
    :param filepath: the csv file
    :param only_center: if we want in the new csv(s) to include only the center images or the left and right camera data
    :param delta_angle: if we use the left and right cameras we have to augment the center steering angle by delta
    :param fraction: what percent of the zero steering angle we should use
    :return:
    """
    from global_variables import SEGREGATION_ANGLE

    def append(from_data, to_data, columns):
        tmp = pd.DataFrame()
        tmp = tmp.append(from_data.loc[:, columns], ignore_index=True)

        sg = - 1 if columns[0] == 'right'else 1

        tmp['steering'] = tmp.apply(lambda row: row['steering'] + sg * delta_angle, axis=1)  # offset right image
        tmp.columns = ['center', 'steering']
        to_data = to_data.append(tmp, ignore_index=True)
        return to_data

    data_list = pd.read_csv(filepath + '/driving_log.csv')

    data_with_steering_angle_left = data_list[data_list['steering'] <= -SEGREGATION_ANGLE]
    data_with_steering_angle_right = data_list[data_list['steering'] >= SEGREGATION_ANGLE]
    data_with_center = data_list[(data_list['steering'] >= -SEGREGATION_ANGLE) &
                                 (SEGREGATION_ANGLE >= data_list['steering'])].sample(frac=fraction)

    data_train_center = pd.DataFrame()
    data_train_left = pd.DataFrame()
    data_train_right = pd.DataFrame()

    data_train_center = data_train_center.append(data_with_center.loc[:, ['center', 'steering']], ignore_index=True)
    data_train_left = data_train_left.append(data_with_steering_angle_left.loc[:, ['center', 'steering']],
                                             ignore_index=True)
    data_train_right = data_train_right.append(data_with_steering_angle_right.loc[:, ['center', 'steering']],
                                               ignore_index=True)

    if only_center is False:
        # left angles dataset
        data_train_left = append(data_with_steering_angle_left, data_train_left, ['right', 'steering'])
        data_train_left = append(data_with_steering_angle_left, data_train_left, ['left', 'steering'])

        data_train_right = append(data_with_steering_angle_right, data_train_right, ['right', 'steering'])
        data_train_right = append(data_with_steering_angle_right, data_train_right, ['left', 'steering'])

        data_train_center = append(data_with_center, data_train_center, ['right', 'steering'])
        data_train_center = append(data_with_center, data_train_center, ['left', 'steering'])

    data_train_center['center'] = data_train_center['center'].str.strip()
    key = pd.Series(np.full(fill_value=filepath + '/', dtype=object, shape=(len(data_train_center))))
    data_train_center['key'] = key

    data_train_left['center'] = data_train_left['center'].str.strip()
    key = pd.Series(np.full(fill_value=filepath + '/', dtype=object, shape=(len(data_train_left))))
    data_train_left['key'] = key

    data_train_right['center'] = data_train_right['center'].str.strip()
    key = pd.Series(np.full(fill_value=filepath + '/', dtype=object, shape=(len(data_train_right))))
    data_train_right['key'] = key

    return data_train_left, data_train_center, data_train_right


def create_training_angles_csv(filename='training_angles', only_center=True):
    from global_variables import AUGMENTATION_ANGLE, FRACTION_OF_ZERO_ANGLES

    data_train_left = pd.DataFrame()
    data_train_center = pd.DataFrame()
    data_train_right = pd.DataFrame()

    data = [data_train_left, data_train_center, data_train_right]

    def create_and_append(data_path, the_data):
        dl, dc, dr = create_training_angles_from_data(data_path, only_center=only_center,
                                                      delta_angle=AUGMENTATION_ANGLE,
                                                      fraction=FRACTION_OF_ZERO_ANGLES)

        return [the_data[0].append(dl), the_data[1].append(dc), the_data[2].append(dr)]

    data = create_and_append('data.center', data)
    data = create_and_append('data.reverse.center', data)
    data = create_and_append('data.extra', data)
    # data = create_and_append('data.recovery', data)

    data[0].to_csv(filename + '_left.csv', index=False)
    data[1].to_csv(filename + '_center.csv', index=False)
    data[2].to_csv(filename + '_right.csv', index=False)


def batch_generator_for_angle_training(data, index, batch_size=128, actions=None, options=None):
    """
    The Batch generator used to train the network.
    :param data:
    :param index:
    :param batch_size:
    :param actions:
    :param options:
    :return:
    """
    from global_variables import IMAGE_SIZE_Y, IMAGE_SIZE_X, ANGLE_NOISE
    import image_handling as imh

    noise_angle = ANGLE_NOISE

    while 1:
        out_images = []
        angle_list = []
        for i in np.arange(batch_size):
            # select randomly a left a center or a right image
            bag = np.random.randint(3)
            # select index from the bag
            img_index = np.random.randint(len(index[bag]))
            img_filename = data[bag].loc[img_index, 'center']
            path = data[bag].loc[img_index, 'key']
            angle = data[bag].loc[img_index, 'steering'] * (1 + np.random.uniform(low=-noise_angle,
                                                                                  high=noise_angle,
                                                                                  size=1))
            image = imh.load_image(path + img_filename)
            coin = np.random.randint(2)
            if coin == 1:  # flip image and angle
                image = np.fliplr(image)
                angle = -angle
            image = imh.preprocess_image(image, IMAGE_SIZE_X, IMAGE_SIZE_Y)
            out_images.append(image)
            angle_list.append(angle)

        yield np.array(out_images), np.array(angle_list)

import pickle
import csv


def read_data_from_files(config, normalized=False):
    """ Read from files the training and testing data sets
    :param config: a dictionary containing a 'files' key, with the filenames needed
    :param normalized: if true load the normalized files
    :return: returns training and testing datasets with labels
    """
    if normalized is False:
        training_file = config['files']['training']
        testing_file = config['files']['testing']
        signs_file = config['files']['sign_labels']
    else:
        training_file = config['files']['normalized_training']
        testing_file = config['files']['normalized_testing']
        signs_file = config['files']['sign_labels']
    try:
        with open(training_file, mode='rb') as f:
            train_set = pickle.load(f)
        with open(testing_file, mode='rb') as f:
            test = pickle.load(f)
        with open(signs_file, mode='r') as f:
            signs = csv.reader(f)
            next(signs)  # skip the first row which as headers
            label_signs = {int(rows[0]): rows[1] for rows in signs}

        return train_set['features'], train_set['labels'], test['features'], test['labels'], label_signs

        # print(train.keys())  # dict_keys(['labels', 'features', 'coords', 'sizes'])
        # print(type(X_train))  # numpy.ndarray
        # print(X_train.shape)  # (39209, 32, 32, 3) (samples, height, width, color channels)
        # print(y_train.shape)  # (39209, 32, 32, 3) (samples, height, width, color channels)
        # print(sign_labels)
    except Exception as e:
        print(e)
        print(e.args)
        quit()


def save_object_to_file(obj, filename):
    """
    Save object to filename
    :param obj: the object we want to save
    :param filename: the filename to save
    :return: None
    """
    try:
        with open(filename, mode='wb') as f:
            pickle.dump(obj, f)
    except Exception as e:
        print(e)
        print(e.args)
        quit()

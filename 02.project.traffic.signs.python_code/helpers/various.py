from collections import Counter


def signs_distribution(signs):
    """
    How many signs are there per class category
    :param signs: the data labels of the dataset
    :return: a tuple (code_labels, quantities_per_label)
    """
    ctr = Counter(sorted(signs))
    code_labels, quantities_per_label = zip(*(ctr.items()))
    return code_labels, quantities_per_label


def batches(data_dict, b_size):
    """
    A generator to help process the data dictionary in batches
    :param data_dict: the data dictionary. a "x" and "y" keys should exist
    :param b_size: the batch size
    :return: the next batch
    """
    size = len(data_dict['x'])
    start = 0
    end = start + b_size
    while end < size:
        yield data_dict['x'][start:end], data_dict['y'][start:end], b_size
        start = end
        end += b_size
    end = size
    yield data_dict['x'][start:end], data_dict['y'][start:end], end - start


def list_batches(data_list, b_size):
    size = len(data_list)
    start = 0
    end = start + b_size
    while end < size:
        yield data_list[start:end], b_size
        start = end
        end += b_size
    end = size
    yield data_list[start:end], end - start

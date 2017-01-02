from fileio import *
from build_network import *
from image_handling import *
from conv_network import *

from sklearn.model_selection import train_test_split

import matplotlib.pyplot as plt

# its a [conv layer, number_filters =16, filter size= 5x5, Use Relu, Don't Use Drop Out

network = [
    ['conv', 16, 5, False, False],
    ['conv', 32, 5, True, False],
    ['conv', 64, 5, True, False],
    ['conv', 128, 1, True, False],
    ['conv', 256, 1, False, False],
    ['fc', 1024, True, True],
    ['fc', 512, True, False],
    ['fc', 43, False, False],
]

configuration = {'files': {'training': './project.data/train.p',
                           'testing': './project.data/test.p',
                           'sign_labels': './project.data/signnames.csv',
                           'normalized_training': './project.data/norm.train.p',
                           'normalized_testing': './project.data/norm.test.p'
                           }
                 }

X_train, y_train, X_test, y_test, sign_labels = read_data_from_files(configuration, normalized=True)

# adding the extra axis that was squashed when image was made grayscale
X_train = np.expand_dims(X_train, axis=3)
X_test = np.expand_dims(X_test, axis=3)

# creating a training and a validation set (90-10) to monitor overfitting
X_train, X_validation, y_train, y_validation = train_test_split(X_train, y_train, test_size=0.1, random_state=1000)

# training, validation & test data in dictionaries
train_dict = {
    'x': X_train,
    'y': y_train
}

validation_dict = {
    'x': X_validation,
    'y': y_validation
}

test_dict = {
    'x': X_test,
    'y': y_test
}

# Part 7b: Network Params ----------------------------------------------------------------------------------------------
n_classes = len(sign_labels.keys())

options = {
    'checkpoints_dir': './project.data.checkpoints/',
    'step_info_every': 1,
    'learning_rate': 1e-3,
    'batch_size': 128
}

tf_graph = build_graph(network, [32, 32, 1], n_classes, options)

session = tf.Session()
saver = tf.train.Saver()
session.run(tf.global_variables_initializer())

# ---------------------------------------Sample Training Session -------------------------------------------------------
training_results_plt = []
validation_results_plt = []

t, v = train(session, saver, tf_graph, 60, train_dict, validation_dict, options, file_name='traffic_1', save_if=0.0)
training_results_plt += t
validation_results_plt += v

training_loss, training_accuracy = zip(*training_results_plt)
validation_loss, validation_accuracy = zip(*validation_results_plt)

plt.plot(training_accuracy, 'r-', validation_accuracy, 'b-')
plt.show()
# ----------------------------------------------------------------------------------------------------------------------

# saver.restore(session, './project.data.checkpoints/traffic_3-83')
train_accuracy(session, tf_graph, test_dict, options)

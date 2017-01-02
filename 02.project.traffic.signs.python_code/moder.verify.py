from fileio import *
from build_network import *
from image_handling import *
from conv_network import *
from plotting import *

from sklearn.model_selection import train_test_split

import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf

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

# adding the extra axis (maybe not needed)
X_train = np.expand_dims(X_train, axis=3)
X_test = np.expand_dims(X_test, axis=3)

# creating a training and a validation set (90-10) to monitor overfitting
X_train, X_validation, y_train, y_validation = train_test_split(X_train, y_train, test_size=0.1, random_state=1000)

# ----------------------------------------- Creating the Data Dictionaries ---------------------------------------------

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
# ----------------------------------------------------------------------------------------------------------------------


# ----------------------------------------------------- Network Params -------------------------------------------------
n_classes = len(sign_labels.keys())

options = {
    'checkpoints_dir': './project.data.checkpoints/',
    'checkpoint_every': 10,
    'step_info_every': 1,
    'learning_rate': 1e-3,
    'batch_size': 128
}

tf_graph = build_graph(network, [32, 32, 1], n_classes, options)

session = tf.Session()
saver = tf.train.Saver()
session.run(tf.global_variables_initializer())

# HOW TO : Request variable by name
# saver.restore(session, './project.data.checkpoints/traffic-8')
# print(session.run(tf.get_default_graph().get_tensor_by_name('w1:0')))

saver.restore(session, './project.data.checkpoints/traffic_3-83')


def training_results():
    train_accuracy(session, tf_graph, test_dict, options)
    train_accuracy(session, tf_graph, train_dict, options)
    train_accuracy(session, tf_graph, validation_dict, options)


def misclassified_sample():
    is_correct, lbl = misclassified_images(session, tf_graph, test_dict, options, top5=True)
    plot_random_images(images=np.squeeze(X_test[is_correct == False]),
                       rows=3, columns=3,
                       labels=lbl[is_correct == False],
                       captions=sign_labels,
                       cmap='Greys_r')


def plot_success_per_sign_category():
    fig, ax = plt.subplots()
    fig.set_size_inches(10, 8)
    acc, tt5 = zip(*success_per_class(n_classes, session, tf_graph, test_dict, sign_labels, options))
    bar_acc = ax.bar(np.arange(n_classes) + 0.35, acc, width=0.35, color='r')
    bar_t5 = ax.bar(np.arange(n_classes), tt5, width=0.35, color='b')
    ax.legend((bar_acc[0], bar_t5[0]), ('accuracy', 'top5 accuracy'))
    plt.show()


def plot_sign_from_sign_category(category):
    plot_random_images(np.squeeze(X_test[y_test == category]),  # plot random images from category 24
                       rows=3, columns=3,  # 3x3 grid
                       labels=y_test[y_test == category],  # labels
                       captions=sign_labels,  # verbal name of sign
                       cmap='Greys_r')  # for grayscale images


def print_accuracy_for_category(category):
    print(session.run(tf_graph['accuracy'],
                      feed_dict={tf_graph['x']: X_test[y_test == category],
                                 tf_graph['y_']: y_test[y_test == category],
                                 tf_graph['keep_prob']: 1.0}
                      ))


plot_confidence_per_image(session,tf_graph,test_dict,9)


def explore_external_images():
    dir_name = 'project.signs.extra/*.jpg'
    external_images = load_images('project.signs.extra/*.jpg')
    external_labels = np.array([1, 1, 14, 14, 17, 17, 25, 27, 38, 38, 40])
    mask = np.array([True, True, True, True, True, False, False, False, False, False, False])
    # plot_random_images(external_images[mask], 1, 5, external_labels[mask], sign_labels, cmap='Greys_r')
    external_images = np.expand_dims(pre_process_dataset(external_images), axis=3)
    extra_set = {'x': external_images[mask], 'y': external_labels[mask]}

    train_accuracy(session,tf_graph,extra_set,options)
    plot_random_images(external_images[mask], 5, 1, external_labels[mask], sign_labels,)
    pred = session.run(tf_graph['prediction'],
                       feed_dict={tf_graph['input']: extra_set['x'],
                                  tf_graph['label']: extra_set['y'],
                                  tf_graph['keep_prob']: 1.0})
    return pred


# ------------------------------ Load graph only from checkpoint using META GRAPH --------------------------------------------
# Load the trained model from disk
# save_dir = 'checkpoints/'
#
# file_name = save_dir+'mdl3-97.09-140001'
# # file_name = save_dir+'mdl3-97.02-130000'
# meta_name = file_name+'.meta'
#
# tf.reset_default_graph()
# session = tf.Session()
#
# saver = tf.train.import_meta_graph(meta_name)
# saver.restore(sess=session, save_path=file_name)
#
# model = tf.get_collection('model')[0] # should have created collection first
# X, y_true, dropout_keep_prob = tf.get_collection('inputs')
# logits = tf.get_collection('logits')[0]
# y_pred = tf.nn.softmax(logits)
# top5 = tf.nn.top_k(y_pred, 5)
# ----------------------------------------------------------------------------------------------------------------------

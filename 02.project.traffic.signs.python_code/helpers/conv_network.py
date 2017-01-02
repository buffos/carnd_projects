import tensorflow as tf
import time
import numpy as np
from various import batches
from datetime import timedelta


def weight_variable(shape, name, mean=0.0, dev=0.05):
    """Create a weight variable with appropriate initialization."""
    initial = tf.truncated_normal(shape, mean=mean, stddev=dev)
    return tf.Variable(initial, name=name)


def bias_variable(length, name, value=0.1):
    """Create a bias variable with appropriate initialization."""
    initial = tf.constant(value, shape=[length])
    return tf.Variable(initial, name=name)


def conv2d(x_, w, b, strides=1):
    x_ = tf.nn.conv2d(x_, w, strides=[1, strides, strides, 1], padding='SAME')
    x_ = tf.nn.bias_add(x_, b)
    return tf.nn.relu(x_)


def maxpool2d(x_, k=2):
    return tf.nn.max_pool(
        x_,
        ksize=[1, k, k, 1],
        strides=[1, k, k, 1],
        padding='SAME')


# the convolution neural network
def conv_net(x_, w, b, pr, name='logits'):
    """
    Creating the convolution network
    :param x_: the input
    :param w: the weight dictionary (per layer)
    :param b: the biases (per layer)
    :param pr: probabilities vector for dropout
    :param name: assign the output tf variable a name
    :return: the logits ready for softmax
    """

    # Layer 1 - 32*32*1 to 16*16*16
    conv1 = conv2d(x_, w['layer_1'], b['layer_1'])
    conv1 = maxpool2d(conv1)

    # Layer 2 - 16*16*16 to 8*8*32
    conv2 = conv2d(conv1, w['layer_2'], b['layer_2'])
    conv2 = maxpool2d(conv2)

    # Layer 3 - 8*8*32 to 4*4*64
    conv3 = conv2d(conv2, w['layer_3'], b['layer_3'])
    conv3 = maxpool2d(conv3)

    # Layer 4 - 4*4*64 to 4*4*128
    conv4 = conv2d(conv3, w['layer_4'], b['layer_4'])
    # conv4 = maxpool2d(conv4)
    # conv4 = tf.nn.dropout(conv4, pr)

    # Fully connected layer - 4*4*128 to 1024
    # Reshape conv4 output to fit fully connected layer1 input
    fc1 = tf.reshape(conv4, [-1, w['fc1'].get_shape().as_list()[0]])
    fc1 = tf.add(tf.matmul(fc1, w['fc1']), b['fc1'])
    fc1 = tf.nn.relu(fc1)
    fc1 = tf.nn.dropout(fc1, pr)

    fc2 = tf.add(tf.matmul(fc1, w['fc2']), b['fc2'])
    fc2 = tf.nn.relu(fc2)

    out = tf.add(tf.matmul(fc2, w['out']), b['out'], name=name)

    return out


# ------------------------------------------------TRAINING FUNCTIONS ---------------------------------------------------


def print_iteration(i, t):
    """
    Helper print function
    :param i: iteration
    :param t: elapsed time till ith iteration
    :return:
    """
    print("Iteration: {0:>6}".format(i + 1))
    print("Elapsed Time: " + str(timedelta(seconds=int(round(t)))))


def train(sess, the_saver, graph, iterations, tr_data, v_data, config, file_name='traffic', save_if=0.0):
    """
    The main training function of a network
    :param sess: the active session
    :param the_saver: the saver object to save and restore checkpoints
    :param graph: the tensorflow graph
    :param iterations: number of iterations to run the training loop
    :param tr_data: the training data dictionary
    :param v_data: the validation data dictionary
    :param config: the general parameters . i.e where to save, batch size, learning rate etc
    :param file_name: the filename of the checkpoint
    :param save_if: the loop saves when a better accuracy is achieved. Initially its 0.0.
            If we restart training from a saved checkpoint this is tells the algorithm not to save
            if the accuracy is not greater then this value
    :return: Two (2) lists that monitor the accuracy of the training and validation data set while training
    """
    save_path = config['checkpoints_dir'] + file_name
    batch_size = config['batch_size']
    x = graph['input']
    y_ = graph['label']
    keep_prob = graph['keep_prob']

    # monitoring loss and accuracy so we can plot later
    acc_train = []
    acc_validation = []

    # read data from latest checkpoint if available
    # recover the global training step and then iterate from there on
    ckpt = tf.train.get_checkpoint_state(config['checkpoints_dir'])
    if ckpt and ckpt.model_checkpoint_path:
        # Restores from checkpoint
        the_saver.restore(sess, ckpt.model_checkpoint_path)
        # Assuming model_checkpoint_path looks something like:
        #   /my-favorite-path/cifar10_train/model.ckpt-0,
        # extract global_step from it.
        global_step = int(ckpt.model_checkpoint_path.split('/')[-1].split('-')[-1])
    else:
        print('No checkpoint file found. Starting from scratch')
        global_step = 0

    # Start Training Loop
    start_time = time.time()
    for step in range(iterations):
        # Loop over all batches
        for b_x, b_y, s in batches(tr_data, batch_size):
            # Run optimization op (backpropagation) and cost op (to get loss value)
            sess.run(graph['optimizer'], feed_dict={x: b_x, y_: b_y, keep_prob: 0.5})

        # Display logs per epoch step
        duration = time.time() - start_time
        print_iteration(step, duration)
        l, a = train_accuracy(sess, graph, v_data, config)

        # Save if better accuracy
        if a > save_if:
            print("Creating Checkpoint -----------------------------------")
            the_saver.save(sess, save_path, global_step=global_step + step + 1)
            print("Training Accuracy: ")
            acc_train.append(train_accuracy(sess, graph, tr_data, config))
            print("Validation Accuracy")
            acc_validation.append(train_accuracy(sess, graph, v_data, config))
            save_if = a

    print("Optimization Finished!")
    return acc_train, acc_validation


def train_accuracy(sess, graph, data, config):
    """
    With the results of a session calculate the accuracy of the training model
    in the data dictionary
    :param sess: the session
    :param graph: the tensorflow graph
    :param data: the data dictionary. 'x' and 'y' keys are presumed
    :param config: any parameters needed are passed in this dictionary. i.e batch size
    :return: returns the loss and precision of the model on the data
    """
    batch_size = config['batch_size']
    num_examples = len(data['x'])
    x = graph['input']
    y_ = graph['label']
    keep_prob = graph['keep_prob']

    loss, accurate, top5 = 0., 0., 0.

    for b_x, b_y, s in batches(data, batch_size):
        l, a, t = sess.run([graph['loss'],
                            graph['accuracy'],
                            graph['accuracy_top5']],
                           feed_dict={x: b_x, y_: b_y, keep_prob: 1.0}
                           )
        loss, accurate, top5 = loss + l * s, accurate + a * s, top5 + t * s

    loss, precision, top5 = loss / num_examples, accurate / num_examples, top5 / num_examples

    msg = "Num examples: {0:>8}  Correct Predictions: {1:>8.0f}  Precision: {2:6.2%} Top5 Precision: {3:6.2%}"
    print(msg.format(num_examples, accurate, precision, top5))
    return loss, precision


def misclassified_images(sess, graph, data, config, top5=False):
    batch_size = config['batch_size']
    num_examples = len(data['x'])
    x = graph['input']
    y_ = graph['label']
    keep_prob = graph['keep_prob']

    correct = np.zeros(shape=num_examples, dtype=int)
    predict = np.zeros(shape=num_examples, dtype=int)
    start, end = 0, 0

    for b_x, b_y, s in batches(data, batch_size):
        end = start + s
        if top5:
            correct[start: end], predict[start: end] = sess.run([graph['is_correct_top5'],
                                                                 graph['prediction']],
                                                                feed_dict={x: b_x, y_: b_y, keep_prob: 1.0})
        else:
            correct[start: end], predict[start: end] = sess.run([graph['is_correct'],
                                                                 graph['prediction']],
                                                                feed_dict={x: b_x, y_: b_y, keep_prob: 1.0})
        start = end
    return correct, predict


def success_per_class(n_cat, sess, graph, data, sign_labels, config):
    per_class = []
    msg = "class: {0:>3} {4:>50}     items: {1:>6}    accuracy: {2:>7.2%}    top5: {3:>7.2%}"
    x = graph['input']
    y_ = graph['label']
    keep_prob = graph['keep_prob']

    for j in range(n_cat):
        batch_size = config['batch_size']
        mask = (data['y'] == j)
        masked_data = {'x': data['x'][mask], 'y': data['y'][mask]}
        num_items = len(data['x'][mask])
        cat_accuracy, cat_t5_accuracy = 0., 0.

        for b_x, b_y, s in batches(masked_data, batch_size):
            ac, t5 = sess.run([graph['accuracy'],
                               graph['accuracy_top5']],
                              feed_dict={x: b_x, y_: b_y, keep_prob: 1.0})
            cat_accuracy, cat_t5_accuracy = cat_accuracy + ac * s, cat_t5_accuracy + t5 * s

        per_class.append((cat_accuracy / num_items, cat_t5_accuracy / num_items))
        print(msg.format(j, num_items, cat_accuracy / num_items, cat_t5_accuracy / num_items, sign_labels[j]))
    return per_class


def certain_in_class(n, sess, data, graph):
    x = graph['input']
    y_ = graph['label']
    keep_prob = graph["keep_prob"]
    dx = data['x']
    dy = data['y']
    a = sess.run(graph['best2'],
                 feed_dict={x: dx[dy == n],
                            y_: dy[dy == n],
                            keep_prob: 1.0}
                 )
    how_certain = abs(a[0][:, 0] - a[0][:, 1])
    items = len(how_certain)
    how_certain_1p = (how_certain < 1).sum()
    how_certain_5p = (how_certain < 5).sum()
    how_certain_10p = (how_certain < 10).sum()
    how_certain_15p = (how_certain < 15).sum()
    how_certain_20p = (how_certain < 20).sum()
    s = how_certain_1p + how_certain_5p + how_certain_10p + how_certain_15p + how_certain_20p
    # scaling to make comparison between classes possible
    return [how_certain_1p / s, how_certain_5p / s, how_certain_10p / s, how_certain_15p / s, how_certain_20p / s]


def certain_in_classes(sess, data, graph, n):
    cc = []
    for i in range(n):
        cc.append(certain_in_class(i, sess, data, graph))
    return list(zip(*cc))

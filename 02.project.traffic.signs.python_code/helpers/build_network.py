import tensorflow as tf
import numpy as np


def new_weights(shape):
    return tf.Variable(tf.truncated_normal(shape, stddev=0.05))


def new_conv_layer(layer_input,  # The previous layer.
                   num_input_channels,  # Num. channels in prev. layer.
                   num_filters,  # Number of filters.
                   filter_size,  # Width and height of each filter.
                   use_pooling=True,
                   use_dropout=False,
                   keep_probability=None):  # Use 2x2 max-pooling.

    # Shape of the filter-weights for the convolution.
    # This format is determined by the TensorFlow API.
    shape = [filter_size, filter_size, num_input_channels, num_filters]

    # Create new weights aka. filters with the given shape.
    weights = new_weights(shape=shape)

    # Create new biases, one for each filter.
    biases = new_biases(length=num_filters)

    # Create the TensorFlow operation for convolution.
    # Note the strides are set to 1 in all dimensions.
    # The first and last stride must always be 1,
    # because the first is for the image-number and
    # the last is for the input-channel.
    # But e.g. strides=[1, 2, 2, 1] would mean that the filter
    # is moved 2 pixels across the x- and y-axis of the image.
    # The padding is set to 'SAME' which means the input image
    # is padded with zeroes so the size of the output is the same.
    layer = tf.nn.conv2d(input=layer_input,
                         filter=weights,
                         strides=[1, 1, 1, 1],
                         padding='SAME')

    # Add the biases to the results of the convolution.
    # A bias-value is added to each filter-channel.
    layer += biases

    # Use pooling to down-sample the image resolution?
    if use_pooling:
        # This is 2x2 max-pooling, which means that we
        # consider 2x2 windows and select the largest value
        # in each window. Then we move 2 pixels to the next window.
        layer = tf.nn.max_pool(value=layer,
                               ksize=[1, 2, 2, 1],
                               strides=[1, 2, 2, 1],
                               padding='SAME')

    # Rectified Linear Unit (ReLU).
    # It calculates max(x, 0) for each layer_input pixel x.
    # This adds some non-linearity to the formula and allows us
    # to learn more complicated functions.
    layer = tf.nn.relu(layer)

    # Note that ReLU is normally executed before the pooling,
    # but since relu(max_pool(x)) == max_pool(relu(x)) we can
    # save 75% of the relu-operations by max-pooling first.

    # We return both the resulting layer and the filter-weights
    # because we will plot the weights later.
    if use_dropout:
        layer = tf.nn.dropout(layer, keep_probability)

    return layer, weights


def new_biases(length):
    return tf.Variable(tf.constant(0.05, shape=[length]))


# https://github.com/Hvass-Labs/TensorFlow-Tutorials/blob/master/02_Convolutional_Neural_Network.ipynb


def flatten_layer(layer):
    # Get the shape of the input layer.
    layer_shape = layer.get_shape()

    # The shape of the input layer is assumed to be:
    # layer_shape == [num_images, img_height, img_width, num_channels]

    # The number of features is: img_height * img_width * num_channels
    # We can use a function from TensorFlow to calculate this.
    num_features = layer_shape[1:4].num_elements()

    # Reshape the layer to [num_images, num_features].
    # Note that we just set the size of the second dimension
    # to num_features and the size of the first dimension to -1
    # which means the size in that dimension is calculated
    # so the total size of the tensor is unchanged from the reshaping.
    layer_flat = tf.reshape(layer, [-1, num_features])

    # The shape of the flattened layer is now:
    # [num_images, img_height * img_width * num_channels]

    # Return both the flattened layer and the number of features.
    return layer_flat, num_features


def new_fc_layer(layer_input,  # The previous layer.
                 num_inputs,  # Num. inputs from prev. layer.
                 num_outputs,  # Num. outputs.
                 use_relu=True,  # Use Rectified Linear Unit (ReLU)?
                 use_dropout=False,  # Use dropout?
                 keep_probability=None):  # if we use this give the keep probability

    # Create new weights and biases.
    weights = new_weights(shape=[num_inputs, num_outputs])
    biases = new_biases(length=num_outputs)

    # Calculate the layer as the matrix multiplication of
    # the input and weights, and then add the bias-values.
    layer = tf.matmul(layer_input, weights) + biases

    # Use ReLU?
    if use_relu:
        layer = tf.nn.relu(layer)

    if use_dropout:
        layer = tf.nn.dropout(layer, keep_probability)

    return layer


def build_network(network_definition, input_placeholder, dropout_probability_placeholder):
    output_layer = input_placeholder
    depth = int(output_layer.get_shape()[-1])
    previous_layer_type = ''
    for layer in network_definition:
        if layer[0] == 'conv':
            output_layer, weights = new_conv_layer(layer_input=output_layer,
                                                   num_input_channels=depth,
                                                   num_filters=layer[1],
                                                   filter_size=layer[2],
                                                   use_pooling=layer[3],
                                                   use_dropout=layer[4],
                                                   keep_probability=dropout_probability_placeholder)
            depth = int(output_layer.get_shape()[-1])
            previous_layer_type = 'conv'
            # print(output_layer)
        else:  # it is fully connected
            if previous_layer_type == 'conv':
                output_layer, depth = flatten_layer(output_layer)
            output_layer = new_fc_layer(layer_input=output_layer,
                                        num_inputs=depth,
                                        num_outputs=layer[1],
                                        use_relu=layer[2],
                                        use_dropout=layer[3],
                                        keep_probability=dropout_probability_placeholder
                                        )
            depth = int(output_layer.get_shape()[-1])
            previous_layer_type = 'fc'
            # print(output_layer)
    return output_layer


def build_graph(network_definition, input_shape, n_classes, options):
    # placeholders
    x = tf.placeholder(tf.float32, shape=[None] + input_shape, name='input')
    y_ = tf.placeholder(tf.int32, shape=[None], name='labels')
    y_onehot = tf.one_hot(y_, n_classes, name="one_hot_labels")
    keep_prob = tf.placeholder('float', name='keep_prob')

    # Create the ConvNet
    y = tf.identity(build_network(network_definition, x, keep_prob), name="softmax_probabilities")  # create an alias with a name
    y_softmax = tf.nn.softmax(y, name="softmax_probabilities")

    # Compute predicted class
    cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(y, y_onehot))
    # train_step = tf.train.GradientDescentOptimizer(options['learning_rate']).minimize(cross_entropy)
    train_step = tf.train.AdamOptimizer(options['learning_rate']).minimize(cross_entropy)

    # the prediction the model give (label)
    prediction = tf.argmax(y, 1, name='prediction')
    # is the prediction correct? : true or false
    correct_prediction = tf.equal(tf.argmax(y, 1), tf.argmax(y_onehot, 1), name="is_correct")
    # average correct predictions
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32), name="accuracy")
    # is the prediction correct in top5 sense? : true or false
    correct_top5_prediction = tf.nn.in_top_k(y, y_, 5, "is_correct_top5")
    # average top 5 accuracy
    topFiver = tf.reduce_mean(tf.cast(correct_top5_prediction, tf.float32), name="accuracy_top5")
    # certainty distribution. top 2
    certainty = tf.nn.top_k(y, k=2)

    tf_graph = {'input': x,
                'label': y_,
                'soft_prediction': y,
                'prediction': prediction,
                'is_correct': correct_prediction,
                "is_correct_top5": correct_top5_prediction,
                'loss': cross_entropy,
                'optimizer': train_step,
                'accuracy': accuracy,
                'accuracy_top5': topFiver,
                'best2': certainty,
                "keep_prob": keep_prob,
                "softmax_probabilities": y_softmax
                }

    return tf_graph

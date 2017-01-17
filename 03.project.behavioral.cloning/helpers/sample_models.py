from global_variables import *

from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten, ELU, Lambda, Activation
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.regularizers import l2, l1


def vgg_like(include_top=True, input_shape=None, weights=None, typeof='class'):
    model = Sequential()
    # normalize image values
    model.add(Lambda(lambda x: x / 127.5 - 1, input_shape=input_shape, name='Normalization'))
    # color space normalisation
    model.add(Convolution2D(3, 1, 1, activation='elu', border_mode='same', name='conv_color_space', init='normal'))

    # Block 1
    model.add(Convolution2D(64, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block1_conv1'))
    model.add(Convolution2D(64, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block1_conv2'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block1_pool'))

    # Block 2
    model.add(Convolution2D(128, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block2_conv1'))
    model.add(Convolution2D(128, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block2_conv2'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block2_pool'))

    # Block 3
    model.add(Convolution2D(256, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block3_conv1'))
    model.add(Convolution2D(256, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block3_conv2'))
    model.add(Convolution2D(256, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block3_conv3'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block3_pool'))

    # Block 4
    model.add(Convolution2D(512, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block4_conv1'))
    model.add(Convolution2D(512, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block4_conv2'))
    model.add(Convolution2D(512, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block4_conv3'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block4_pool'))

    # Block 5
    model.add(Convolution2D(512, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block5_conv1'))
    model.add(Convolution2D(512, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block5_conv2'))
    model.add(Convolution2D(512, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block5_conv3'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block5_pool'))

    if include_top:
        # Classification block
        model.add(Flatten(name='Flatten'))
        model.add(Dense(1024, activation='elu', init='he_normal', name='fc1'))
        model.add(Dropout(0.5, name='fc1_dropout'))
        model.add(Dense(256, activation='elu', init='he_normal', name='fc2'))
        model.add(Dropout(0.5, name='fc2_dropout'))
        model.add(Dense(128, activation='elu', init='he_normal', name='fc3'))
        model.add(Dropout(0.5, name='fc3_dropout'))
        model.add(Dense(64, activation='elu', init='he_normal', name='fc4'))
        model.add(Dropout(0.5, name='fc4_dropout'))
        model.add(Dense(32, activation='elu', init='he_normal', name='fc5'))
        model.add(Dropout(0.5, name='fc5_dropout'))

        if typeof == 'class':
            model.add(Dense(NUMBER_OF_ZONES, name='output', init='zero'))
            model.add(Activation('softmax'))
        else:
            model.add(Dense(1, name='output', init='zero'))

    if weights is not None:
        import os.path
        if os.path.exists(weights):
            model.load_weights(weights, by_name=True)

    return model


def vgg_3blocks(include_top=True, input_shape=None, weights=None, typeof='class'):
    model = Sequential()
    # normalize image values
    model.add(Lambda(lambda x: x / 127.5 - 1, input_shape=input_shape, name='Normalization'))
    # color space normalisation
    model.add(Convolution2D(3, 1, 1, activation='elu', border_mode='same', name='conv_color_space', init='normal'))

    # Block 1
    model.add(Convolution2D(64, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block1_conv1'))
    model.add(Convolution2D(64, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block1_conv2'))
    # model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block1_pool'))
    model.add(Dropout(0.5, name='block1_dropout'))

    # Block 2
    model.add(Convolution2D(128, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block2_conv1'))
    model.add(Convolution2D(128, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block2_conv2'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block2_pool'))
    model.add(Dropout(0.5, name='block2_dropout'))

    # Block 3
    model.add(Convolution2D(256, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block3_conv1'))
    model.add(Convolution2D(256, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block3_conv2'))
    model.add(Convolution2D(256, 3, 3, activation='elu', border_mode='same', init='he_normal', name='block3_conv3'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block3_pool'))
    model.add(Dropout(0.5, name='block3_dropout'))

    if include_top:
        # Classification block
        model.add(Flatten(name='Flatten'))
        model.add(Dense(1024, activation='elu', init='he_normal', name='fc1'))
        model.add(Dropout(0.5, name='fc1_dropout'))
        model.add(Dense(256, activation='elu', init='he_normal', name='fc2'))
        model.add(Dropout(0.5, name='fc2_dropout'))
        model.add(Dense(32, activation='elu', init='he_normal', name='fc3'))
        model.add(Dropout(0.5, name='fc3_dropout'))

        if typeof == 'class':
            model.add(Dense(NUMBER_OF_ZONES, name='output', init='zero'))
            model.add(Activation('softmax'))
        else:
            model.add(Dense(1, name='output', init='zero'))

    if weights is not None:
        import os.path
        if os.path.exists(weights):
            model.load_weights(weights, by_name=True)

    return model


def simple_1(include_top=True, input_shape=None, weights=None, typeof='class'):
    model = Sequential()
    # normalize image values
    model.add(Lambda(lambda x: x / 127.5 - 1, input_shape=input_shape, name='Normalization'))
    # color space normalisation
    model.add(Convolution2D(3, 1, 1, activation='elu', border_mode='same', name='conv_color_space', init='normal'))

    # Block 1
    model.add(Convolution2D(32, 3, 3, activation='elu', border_mode='same', init='normal', name='block1_conv1'))
    model.add(Convolution2D(32, 3, 3, activation='elu', border_mode='same', init='normal', name='block1_conv2'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block1_pool'))
    model.add(Dropout(0.5, name='block1_dropout'))

    # Block 2
    model.add(Convolution2D(64, 3, 3, activation='elu', border_mode='same', init='normal', name='block2_conv1'))
    model.add(Convolution2D(64, 3, 3, activation='elu', border_mode='same', init='normal', name='block2_conv2'))
    model.add(MaxPooling2D((2, 2), strides=(2, 2), name='block2_pool'))
    model.add(Dropout(0.5, name='block2_dropout'))

    if include_top:
        # Classification block
        model.add(Flatten(name='Flatten'))
        model.add(Dense(256, activation='elu', init='normal', name='fc1'))
        model.add(Dropout(0.5, name='fc1_dropout'))
        model.add(Dense(64, activation='elu', init='normal', name='fc2'))
        model.add(Dropout(0.5, name='fc2_dropout'))
        # model.add(Dense(16, activation='elu', init='normal', name='fc3'))
        # model.add(Dropout(0.5, name='fc3_dropout'))

        if typeof == 'class':
            model.add(Dense(NUMBER_OF_ZONES, name='output', init='zero'))
            model.add(Activation('softmax'))
        else:
            model.add(Dense(1, name='output', init='zero'))

    if weights is not None:
        import os.path
        if os.path.exists(weights):
            model.load_weights(weights, by_name=True)

    return model


def tiny(include_top=True, input_shape=None, weights=None, typeof='class'):
    model = Sequential()
    # normalize image values
    model.add(Lambda(lambda x: x / 127.5 - 1, input_shape=input_shape, name='Normalization'))
    # color space normalisation
    model.add(Convolution2D(16, 3, 3, border_mode='valid', name='conv_1', init='normal'))
    model.add(MaxPooling2D((4, 4), (4, 4), 'valid', name='max_pool'))
    model.add(Dropout(0.25))
    model.add(Flatten(name='Flatten'))
    model.add(Dense(1, name='output', init='zero'))

    if weights is not None:
        import os.path
        if os.path.exists(weights):
            model.load_weights(weights, by_name=True)

    return model


def nvidia(include_top=True, input_shape=None, weights=None, typeof='class'):
    model = Sequential()

    model.add(Lambda(lambda x: x / 127.5 - 1, input_shape=input_shape, name='Normalization'))
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2), border_mode="valid", init='he_normal', name='conv1', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2), border_mode="valid", init='he_normal', name='conv2', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2), border_mode="valid", init='he_normal', name='conv3', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Convolution2D(64, 3, 3, subsample=(1, 1), border_mode="valid", init='he_normal', name='conv4', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Convolution2D(64, 3, 3, subsample=(1, 1), border_mode="valid", init='he_normal', name='conv5', W_regularizer=l2(0.01)))
    model.add(Flatten(name='flatten1'))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Dense(1164, init='he_normal', name='dense1', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Dense(100, init='he_normal', name='dense2', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Dense(50, init='he_normal', name='dense3', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Dense(10, init='he_normal', name='dense4', W_regularizer=l2(0.01)))
    model.add(ELU())
    # model.add(Dropout(0.50))
    model.add(Dense(1, init='he_normal', name='dense5', W_regularizer=l2(0.01)))

    if weights is not None:
        import os.path
        if os.path.exists(weights):
            model.load_weights(weights, by_name=True)

    return model

from global_variables import *
from sample_models import *

import pickle
import json

from keras.models import model_from_json
from keras.callbacks import ModelCheckpoint
from keras.optimizers import Adam


def normalize(image):
    return image / 127.5 - 1


def get_model(include_top=True, input_shape=None, weights=None, typeof='class'):
    # return vgg_like(include_top, input_shape, weights, typeof)
    # return vgg_3blocks(include_top, input_shape, weights, typeof)
    # return simple_1(include_top, input_shape, weights, typeof)
    # return tiny(include_top, input_shape, weights, typeof)
    return nvidia(include_top, input_shape, weights, typeof)


def freeze_model(model):
    """
    Freeze all layers of the model (before adding new layers to stop backpropagation
    :param model:
    :return:
    """
    for layer in model:
        layer.trainable = False
    return model


def compile_model(model, typeof='class'):
    if typeof == 'class':
        model.compile(optimizer=Adam(LEARNING_RATE),
                      loss='sparse_categorical_crossentropy',
                      metrics=['accuracy'])
    else:
        model.compile(optimizer=Adam(LEARNING_RATE),
                      loss='mse')
    return model


def train_model(model, batch_gen, valid_gen, train_samples, validation_samples, filepath='best_lane.h5'):
    model.fit_generator(
        generator=batch_gen,
        samples_per_epoch=train_samples,
        nb_epoch=EPOCHS,
        validation_data=valid_gen,
        nb_val_samples=validation_samples,
        verbose=1,
        callbacks=[ModelCheckpoint(filepath=filepath, verbose=1, save_best_only=True)]
    )


def evaluate_model(model, batch_gen, test_samples):
    return model.evaluate_generator(generator=batch_gen,
                                    val_samples=test_samples
                                    )


def predict_lane(model, image):
    return int(model.predict_classes(image, batch_size=1, verbose=0))


def predict_angle(model, image):
    return model.predict(image, batch_size=1, verbose=0)[0][0]


def save_history(history, filename):
    with open(filename, 'wb') as f:
        pickle.dump(history, f)


def load_history(filename):
    with open(filename, 'rb') as f:
        history = pickle.load(f)
    return history


def save_model_to_json(model, filename):
    with open(filename, 'w') as f:
        json_string = model.to_json()
        json.dump(json_string, f)


def load_model(filepath=''):
    with open(filepath + 'model.json', 'r') as j_file:
        json_string = json.load(j_file)
        model = model_from_json(json_string)

    model = compile_model(model)
    model.load_weights(filepath + 'model.h5')
    return model

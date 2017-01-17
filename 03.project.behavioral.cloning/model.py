from data_handling import batch_generator_for_angle_training, create_training_angles_csv, create_training_sets
from global_variables import *
from model_api import *

import pandas as pd
import image_handling as imh

import matplotlib.pyplot as plt

#  we separate the left steering angles, the right steering angles and the zero-centered angles
# that way we can sample equally from those at every step of the training
create_training_angles_csv('training_angles', only_center=ONLY_CENTER_IMAGES)
data_list_left = pd.read_csv('training_angles_left.csv')
data_list_center = pd.read_csv('training_angles_center.csv')
data_list_right = pd.read_csv('training_angles_right.csv')

data_list = [data_list_left, data_list_center, data_list_right]
entries = [len(data_list_left), len(data_list_center), len(data_list_right)]
print(entries)

# we are creating 3 indexes that the generator will choose from in order to load images
tl, vl = create_training_sets(entries[0], test_set=False, train_size=0.85)
tc, vc = create_training_sets(entries[1], test_set=False, train_size=0.85)
tr, vr = create_training_sets(entries[2], test_set=False, train_size=0.85)

train_index, validation_index = [tl, tc, tr], [vl,vc, vr]


validation_samples = len(validation_index[0]) + len(validation_index[1]) + len(validation_index[2])


t = batch_generator_for_angle_training(data=data_list,
                                       index=train_index,
                                       batch_size=TRAIN_BATCH_SIZE,
                                       actions=[imh.crop_image_to_road, imh.resize_image],
                                       options={'new_x_size': IMAGE_SIZE_X, 'new_y_size': IMAGE_SIZE_Y}
                                       )

v = batch_generator_for_angle_training(data=data_list,
                                       index=validation_index,
                                       batch_size=VALIDATION_BATCH_SIZE,
                                       actions=[imh.crop_image_to_road, imh.resize_image],
                                       options={'new_x_size': IMAGE_SIZE_X, 'new_y_size': IMAGE_SIZE_Y}
                                       )

m = get_model(input_shape=IMAGE_SHAPE, typeof='regression', weights='model.h5')
m = compile_model(m, typeof='regression')
m.summary()

m.save('model_structure.h5')
save_model_to_json(m, 'model.json')

history = train_model(m, t, v, ITERATIONS_PER_EPOCH, validation_samples, 'model.h5')
save_history(history, 'model_history.p')

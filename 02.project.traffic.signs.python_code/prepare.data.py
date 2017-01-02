from fileio import read_data_from_files, save_object_to_file
from plotting import plot_random_images, plot_histogram
from image_handling import pre_process_dataset, generate_images, pre_process
from various import signs_distribution

configuration = {'files': {'training': './project.data/train.p',
                           'testing': './project.data/test.p',
                           'sign_labels': './project.data/signnames.csv',
                           'normalized_training': './project.data/norm.train.p',
                           'normalized_testing': './project.data/norm.test.p'
                           }
                 }

# read data from file
X_train, y_train, X_test, y_test, sign_labels = read_data_from_files(configuration, normalized=False)
# plot same random images
plot_random_images(X_train, 3, 3, y_train, sign_labels)
# preprocess the training set


normalized_X_train = pre_process_dataset(X_train)
normalized_X_test = pre_process_dataset(X_test)
# find how many images are per class category
code_labels, quantities_per_label = signs_distribution(y_train)
# decide how many you want to generate in order to have an even distribution
target_quantities_per_label = 1.0 * max(quantities_per_label)
# how many images i want per sign class
# if we use 1.0 , the traffic sign class with max number of images will not generate new content
# create more data so each label category has the same data size
# plotting the distribution BEFORE data generation
plot_histogram(code_labels, quantities_per_label)
# generate new images and append them to the original dataset
normalized_X_train, normalized_y_train = generate_images(normalized_X_train,
                                                         y_train,
                                                         code_labels,
                                                         target_quantities_per_label,
                                                         erase_percent=0.3
                                                         )
# just view a random set of images to verify everything is fine
plot_random_images(images=normalized_X_train,
                   rows=3, columns=3,
                   labels=normalized_y_train,
                   captions=sign_labels,
                   cmap='Greys_r'
                   )

# NEW plot to see the new distribution
# first we need to recalculate the sign class distribution
code_labels, quantities_per_label = signs_distribution(normalized_y_train)
# the new uniform distribution
plot_histogram(code_labels, quantities_per_label)
# save new data to file
normalized_data = {'features': normalized_X_train, 'labels': normalized_y_train}
normalized_test = {'features': normalized_X_test, 'labels': y_test}
save_object_to_file(normalized_data, configuration['files']['normalized_training'])
save_object_to_file(normalized_test, configuration['files']['normalized_testing'])

import numpy as np
import matplotlib.pyplot as plt

from conv_network import certain_in_classes


def plot_random_images(images, rows, columns, labels, captions, predicted_label=None, cmap=None):
    """
    Creates a figure grid with images from the provided dataset with labels as titles
    :param images: an ndarray with images
    :param rows: no of grid rows
    :param columns: no of column rows
    :param labels: an ndarray proving the corresponding image labels as a number encoding
    :param captions: the textual interpretation of the image
    :param predicted_label: if this is given then we show the predicted label
    :param cmap: the colormap used to draw the images
    :return: None
    """
    # no check for edge cases where images are much less than rows and/or rows*columns
    no_images = min(len(images), rows * columns)
    images_to_plot = np.random.choice(len(images), no_images, replace=False)

    figure = plt.figure(figsize=(8, 8), dpi=80)
    figure.subplots_adjust(wspace=0.7, hspace=0.2)
    for index in range(0, no_images):
        image_index = images_to_plot[index]
        image = images[image_index]
        image_label = labels[images_to_plot[index]]

        if predicted_label is None:
            image_caption = "Label: {0}".format(captions[image_label])
        else:
            image_prediction = predicted_label[images_to_plot[index]]
            image_caption = "Label: {0}, Pred: {1}".format(captions[image_label], captions[image_prediction])

        subplot = figure.add_subplot(rows, columns, index + 1)
        # subplot.set_title(image_caption, fontsize=10)
        subplot.set_xlabel(image_caption, fontsize=10)
        subplot.set_xticks([])
        subplot.set_yticks([])
        # subplot.get_xaxis().set_visible(False)
        # subplot.get_yaxis().set_visible(False)
        plt.imshow(image, cmap=cmap)
    plt.show()


def plot_histogram(labels, quantities):
    """
    Create a bar histogram to visualize the distribution of number of images
    per sign class
    :param labels: an ordinal list with all the sign classes
    :param quantities: the number of images per sign class
    :return: None
    """
    indexes = np.arange(len(labels))
    width = 0.8

    # plotting the histogram
    plt.bar(indexes, quantities, width)
    plt.rc('xtick', labelsize=10)
    plt.xticks(indexes + width * 0.5, labels)
    plt.show()


def plot_certainty_per_class(sess, data, graph, n, width=0.35):
    bars = certain_in_classes(sess, data, graph, n)  # 5 bars
    ind = np.arange(n)

    pl = []  # the plots
    colors = ['r', 'y', 'm', 'b', 'g']  # the colors per bar stack

    for i in range(len(bars)):
        if i == 0:
            pl.append(plt.bar(ind, bars[i], width, color=colors[i]))
            bottom = np.asarray(bars[i])  # save the previous bar so it can sit on the bottom of the next one
        else:
            pl.append(plt.bar(ind, bars[i], width, bottom=bottom, color=colors[i]))
            bottom += np.asarray(bars[i])
    plt.legend((pl[0][0], pl[1][0], pl[2][0], pl[3][0], pl[4][0]), ('1%', '5%', '10%', '15%', '20%'))
    plt.figure(figsize=(10, 8), dpi=80)
    plt.show()


def plot_confidence_per_image(sess, graph, data, image_number):
    x = graph['input']
    y_ = graph['label']
    pr = graph["keep_prob"]
    dx = data['x']
    dy = data['y']
    a = sess.run(tf.nn.top_k(graph['soft_prediction'], k=3),
                 feed_dict={x: dx[image_number:image_number + 1],
                            y_: dy[image_number:image_number + 1],
                            pr: 1.0}
                 )

    possible_signs = a[1][0]
    labels = [sign_labels[sign] for sign in possible_signs]
    possible_signs_softmax_probabilities = a[0][0]
    # print(possible_signs_softmax_probabilities)
    explode = (0.1, 0.0, 0.0)

    plt.figure(1, figsize=(10, 10))
    ax = plt.axes([0.1, 0.1, 0.8, 0.8])

    patches, texts, autotexts = plt.pie(possible_signs_softmax_probabilities,
                                        explode=explode,
                                        labels=labels,
                                        autopct='%1.1f%%',
                                        shadow=True,
                                        startangle=90,
                                        labeldistance=1.1,
                                        pctdistance=0.6)

    for t in autotexts:
        t.set_color('w')

    plt.legend(patches, labels)
    plt.show()
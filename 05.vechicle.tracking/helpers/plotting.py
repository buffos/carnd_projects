import matplotlib.pyplot as plt
import numpy as np


# noinspection PyPep8Naming
def plotChannelHistograms(histograms, histogramCenters, titles=None):
    n_histograms = len(histograms)
    if titles is None:
        titles = [''] * n_histograms
    fig = plt.figure(figsize=(12, 3))
    for i in range(n_histograms):
        plt.subplot(1, n_histograms, i + 1)
        plt.bar(histogramCenters, histograms[i])
        plt.xlim(0, 256)
        plt.title(titles[i])
        fig.tight_layout()
    plt.show()


def show_images(images, per_row=2, titles=None, main_title=None):
    figure = plt.figure(1)

    for n, img in enumerate(images):
        ax = figure.add_subplot(np.ceil(len(images) / per_row), per_row, n + 1)

        ax.grid(False)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_xticks([])
        ax.set_yticks([])

        if (titles is not None and len(titles) >= n):
            ax.set_title(titles[n])

        plt.imshow(img)
    if main_title is not None:
        plt.suptitle(main_title)

    plt.show()

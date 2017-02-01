# noinspection PyPep8Naming
def plotImages(*images, filename=None):
    import matplotlib.pyplot as plt
    from math import ceil
    noImages = len(images)
    noColumns = 2
    noRows = ceil(noImages / noColumns)
    currentFigure = 1
    for image in images:
        plt.subplot(noRows, noColumns, currentFigure)
        plt.imshow(image)
        currentFigure += 1
    plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0, hspace=0)
    plt.tight_layout()
    if filename is None:
        plt.show()
    else:
        plt.savefig(filename)


# noinspection PyPep8Naming
def plotData(data):
    import matplotlib.pyplot as plt
    plt.plot(data)
    plt.show()


def showImages(*images, title=None):
    import matplotlib.pyplot as plt
    f, axes = plt.subplots(1, len(images), squeeze=False)
    f.set_size_inches((6 * len(images), 8))
    if title and len(images) == 1 and type(title) == str:
        title = [title]
    for i in range(len(images)):
        img = images[i]
        ax = axes[0][i]
        if title is not None:
            assert len(title) == len(images)
            t = title[i]
            ax.text(0.5, 1.05, t, transform=ax.transAxes, fontsize=14, verticalalignment='bottom',
                    horizontalalignment='center')
        if len(img.shape) == 3:
            ax.imshow(img)
        else:
            ax.imshow(img, cmap="gray")

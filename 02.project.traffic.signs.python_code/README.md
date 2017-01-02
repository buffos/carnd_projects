## Exploring what parameters after network accuracy

#### Network 1:

each line in the list describing the network is:
["type of layer" , "number of filters" , "filter size", Max Pool? , Drop Out?]


    network = [
        ['conv', 16, 5, True, False],
        ['conv', 32, 5, True, False],
        ['conv', 64, 5, True, False],
        ['conv', 128, 1, False, True],
        ['conv', 256, 1, False, False],
        ['fc', 1024, True, True],
        ['fc', 512, True, False],
        ['fc', 43, False, False],
    ]

Used that one with and without dropout and it see is that the network reaches it limits because
at training I am seeing training accuracies** around 99%+** which means that the network almost
 perfectly represents initial data but its **pattern recognition capabilities** have reached its limits


`Num examples:    12630  Correct Predictions:    11605  Precision: 91.88% Top5 Precision: 98.11%`

Before going for a larger network , 2 things have to be tried.

 *  Smaller filter sizes
 *  Removing Max Pooling Layers

The first will probably find more patterns, within the same structure of the network, and the second
will add more detail to the data that has been removed from pooling.

_first try_ is watch if filter size affect the results

#### Network 2:

    network = [
        ['conv', 16, 3, True, False],
        ['conv', 32, 3, True, False],
        ['conv', 64, 3, True, False],
        ['conv', 128, 1, False, True],
        ['conv', 256, 1, False, False],
        ['fc', 1024, True, True],
        ['fc', 512, True, False],
        ['fc', 43, False, False],
    ]

`Num examples:    12630  Correct Predictions:    11499  Precision: 91.05% Top5 Precision: 98.15%`

No real improvement and a rather slower converge

let not _reduce_ the max pooling layers to two and place the second one just before the fully connected layer.

#### Network 3:

    network = [
        ['conv', 16, 3, False, False],
        ['conv', 32, 3, True, False],
        ['conv', 64, 3, False, False],
        ['conv', 128, 1, False, True],
        ['conv', 256, 1, True, False],
        ['fc', 1024, True, True],
        ['fc', 512, True, False],
        ['fc', 43, False, False],
    ]

this is much slower in my cpu per iteration , rising from 2 minutes per iteration to 12 minutes
After 12+ hours

`Num examples:    12630  Correct Predictions:    12217  Precision: 96.73% Top5 Precision: 99.25%`

A big jump as expected, since there was so much more information. That makes me wonder what will
happen if we max pool aggressively after the first few layers (2 for starters)


Going back to a "smaller" network trying now the impact of erasing randomly pixels from the image.
The main difference from the first network is that max pooling is not done on the first layer (and instead its done on the 4h)


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


I am always stopping on 40 iterations although with more data missing maybe more iterations would give better results.

* Lets see how it changes things if I **erase 10% of the pixels** from the training data.

`Num examples:    12630  Correct Predictions:    11651  Precision: 92.25% Top5 Precision: 98.52%`

* Lets see how it changes things if I **erase 20% of the pixels** from the training data.

`Num examples:    12630  Correct Predictions:    11686  Precision: 92.53% Top5 Precision: 98.08%`

That is a **small** improvement just on that small modification of the data. lets try to push it more and see the results.

* Lets see how it changes things if I **erase 30% of the pixels** from the training data.

`Num examples:    12630  Correct Predictions:    11750  Precision: 93.03% Top5 Precision: 98.23%`

and it 'feels' like more iterations will give better results, although architecture improvements get better results, this could be useful tool.

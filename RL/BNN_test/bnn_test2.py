import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
sns.set_style('white')
sns.set_context('talk')

import pymc3 as pm
import theano.tensor as T
import theano

from scipy.stats import mode, chisquare

from sklearn.metrics import confusion_matrix, accuracy_score

import lasagne
import sys, os
import itertools

def load_dataset():
    # We first define a download function, supporting both Python 2 and 3.
    if sys.version_info[0] == 2:
        from urllib import urlretrieve
    else:
        from urllib.request import urlretrieve
        
    def download(filename, source='http://yann.lecun.com/exdb/mnist/'):
        print("Downloading %s" % filename)
        urlretrieve(source + filename, filename)
        
    # We then define functions for loading MNIST images and labels.
    # For convenience, they also download the requested files if needed.
    import gzip
        
    def load_mnist_images(filename):
        if not os.path.exists(filename):
            download(filename)
            # Read the inputs in Yann LeCun's binary format.
        with gzip.open(filename, 'rb') as f:
            data = np.frombuffer(f.read(), np.uint8, offset=16)
        # The inputs are vectors now, we reshape them to monochrome 2D images,
        # following the shape convention: (examples, channels, rows, columns)
        data = data.reshape(-1, 1, 28, 28)
        # The inputs come as bytes, we convert them to float32 in range [0,1].
        # (Actually to range [0, 255/256], for compatibility to the version
        # provided at http://deeplearning.net/data/mnist/mnist.pkl.gz.)
        return data / np.float32(256)
                    
    def load_mnist_labels(filename):
        if not os.path.exists(filename):
            download(filename)
        # Read the labels in Yann LeCun's binary format.
        with gzip.open(filename, 'rb') as f:
            data = np.frombuffer(f.read(), np.uint8, offset=8)
        # The labels are vectors of integers now, that's exactly what we want.
        return data
                
    # We can now download and read the training and test set images and labels.
    X_train = load_mnist_images('train-images-idx3-ubyte.gz')
    y_train = load_mnist_labels('train-labels-idx1-ubyte.gz')
    X_test = load_mnist_images('t10k-images-idx3-ubyte.gz')
    y_test = load_mnist_labels('t10k-labels-idx1-ubyte.gz')
    
    # We reserve the last 10000 training examples for validation.
    X_train, X_val = X_train[:-10000], X_train[-10000:]
    y_train, y_val = y_train[:-10000], y_train[-10000:]
    
    # We just return all the arrays in order, as expected in main().
    # (It doesn't matter how we do this as long as we can read them again.)
    return X_train, y_train, X_val, y_val, X_test, y_test

print("Loading data...")
X_train, y_train, X_val, y_val, X_test, y_test = load_dataset()

# Building a theano.shared variable with a subset of the data to make construction of the model faster.
# We will later switch that out, this is just a placeholder to get the dimensionality right.
input_var = theano.shared(X_train[:500, ...].astype(np.float64))
target_var = theano.shared(y_train[:500, ...].astype(np.float64))

def build_ann(init):
    l_in = lasagne.layers.InputLayer(shape=(None, 1, 28, 28),
                                     input_var=input_var)

    # Add a fully-connected layer of 800 units, using the linear rectifier, and
    # initializing weights with Glorot's scheme (which is the default anyway):
    n_hid1 = 800
    l_hid1 = lasagne.layers.DenseLayer(
        l_in, num_units=n_hid1,
        nonlinearity=lasagne.nonlinearities.tanh,
        b=init,
        W=init
    )
    
    n_hid2 = 800
    # Another 800-unit layer:
    l_hid2 = lasagne.layers.DenseLayer(
        l_hid1, num_units=n_hid2,
        nonlinearity=lasagne.nonlinearities.tanh,
        b=init,
        W=init
    )
    
    # Finally, we'll add the fully-connected output layer, of 10 softmax units:
    l_out = lasagne.layers.DenseLayer(
        l_hid2, num_units=10,
        nonlinearity=lasagne.nonlinearities.softmax,
        b=init,
        W=init
    )
    
    prediction = lasagne.layers.get_output(l_out)
    
    # 10 discrete output classes -> pymc3 categorical distribution
    out = pm.Categorical('out',
                         prediction,
                         observed=target_var)
    return out
    
class GaussWeights(object):
    def __init__(self):
        self.count = 0
    def __call__(self, shape):
        self.count += 1
        return pm.Normal('w%d' % self.count, mu=0, sd=.1,
                         testval=np.random.normal(size=shape).astype(np.float64),
                         shape=shape)

# Tensors and RV that will be using mini-batches
minibatch_tensors = [input_var, target_var]

# Generator that returns mini-batches in each iteration
def create_minibatch(data, batchsize=500):
    global count
    rng = np.random.RandomState(0)
    start_idx = 0
    while True:
        # Return random data samples of set size batchsize each iteration
        ixs = rng.randint(data.shape[0], size=batchsize)
        yield  data[ixs]
        
minibatches = itertools.izip(
    create_minibatch(X_train, 500),
    create_minibatch(y_train, 500),
)

total_size = len(y_train)

def run_advi(likelihood, advi_iters=50):
    # Train on train data
    input_var.set_value(X_train[:500, ...])
    target_var.set_value(y_train[:500, ...])
    
    v_params = pm.variational.advi_minibatch(
        n=advi_iters, minibatch_tensors=minibatch_tensors,
        minibatch_RVs=[likelihood], minibatches=minibatches,
        total_size=total_size, learning_rate=1e-2, epsilon=1.0
    )
    trace = pm.variational.sample_vp(v_params, draws=500)
    
    # Predict on test data
    input_var.set_value(X_test)
    target_var.set_value(y_test)

    # Generate posterior predictable samples
    ppc = pm.sample_ppc(trace, samples=100)
    y_pred = mode(ppc['out'], axis=0)[0][0]
    
    return v_params, trace, ppc, y_pred

with pm.Model() as neural_network:
    likelihood = build_ann(GaussWeights())
    v_params, trace, ppc, y_pred = run_advi(likelihood)

plt.plot(v_params.elbo_vals[0:])
plt.show()
sns.despine()
print(len(y_test), len(y_pred))
sns.heatmap(confusion_matrix(y_test[0:len(y_pred)], y_pred))
plt.show()

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns
sns.set_style('white')
sns.set_context('talk')

import pymc3 as pm
import theano.tensor as T
import theano
from pylearn.rl_lib.kld import *

# bayes_by_backprop.py import
'''
from theano.tensor.shared_randomstreams import RandomStreams
from theano.sandbox.rng_mrg import MRG_RandomStreams
from lasagne.updates import adam
from lasagne.utils import collect_shared_vars

from sklearn.datasets import fetch_mldata
from sklearn.cross_validation import train_test_split
'''
from sklearn import preprocessing

# end import

from scipy.stats import mode, chisquare

from sklearn.metrics import confusion_matrix, accuracy_score

import lasagne
import sys, os
import itertools

from data_lib import *         # my library
#from sandbox.vime.dynamics import bnn
from bayes_by_backprop import *

if __name__ == '__main__':

    n_train = 1000
    n_test  = 100
    n_state = 0
    DATA = 0
    
    # This part is necessary
    theano.config.compute_test_value = 'off'
    
    if DATA == 1:
        log_file = '../../log/state_action_1dof.log'
        output_file = 'out1'
        n_state = 1
        X_train, y_train, X_test, y_test = load_file(log_file, n_state=n_state, n_train=n_train, n_test=n_test, sampling='curiosity')
        
    elif DATA == 2:
        log_file = '../../log/state_action_2dof.log'
        output_file = 'log2'
        n_state = 2
        X_train, y_train, X_test, y_test = load_file(log_file, n_state=n_state, n_train=n_train, n_test=n_test, sampling='curiosity')

    elif DATA == 0:
        output_file = "out_sin"
        X_train, y_train, X_test, y_test = get_sin_dataset(n_train, n_test)
    
    input_size = X_train.shape[1]
    output_size = y_train.shape[1]
    batch_size = 100
    n_batches = int(n_train / batch_size)

    '''
    bnn_dynamics = bnn.BNN(
        n_in=input_size,
        n_hidden=[32],
        n_out=output_size,
        n_batches=n_batches,
        layers_type=[1,1],
        trans_func=lasagne.nonlinearities.rectify,
        out_func=lasagne.nonlinearities.linear,
        batch_size=batch_size,
        n_samples=10,
        prior_sd=0.5,
        use_reverse_kl_reg=False,
        reverse_kl_reg_factor=1e-3,
        second_order_update=False,
        learning_rate=0.001,
        compression=False,
        information_gain=True
        )
    '''
    bnn_dynamics = BNN(input_size, output_size, batch_size, learning_task="regression")
    bnn_dynamics.set_train_data(X_train, y_train)
    bnn_params = bnn_dynamics.get_all_params_for_kld()
    print bnn_params[1]
    
    n_epochs = 5000
    
    X_batches = [ X_train[i * batch_size: (i + 1) * batch_size] for i in xrange(n_batches) ]
    y_batches = [ y_train[i * batch_size: (i + 1) * batch_size] for i in xrange(n_batches) ]
    
    # and finally, training loop
    with open(output_file, 'w') as write_file:
        with open("kld_log", 'w') as kld_file:
            with open("bnn_result", 'w') as result_file:
            
                for e in xrange(n_epochs):
                    # learning by batches
                    errs = []
                    kld = [e]
                    for b in xrange(n_batches):
                        batch_err = bnn_dynamics.train_fn(b)
                        errs.append(batch_err)
                    
                        # logging kld
                        old_bnn_params = bnn_params
                        bnn_params = bnn_dynamics.get_all_params_for_kld()
                        kld.append(kld_logsigma_multi_dim(bnn_params, old_bnn_params))
                    

                    kld = map(str, kld)
                    kld_file.write(' '.join(kld) + '\n')
                    out = bnn_dynamics.pred_fn(X_test)
                    acc = np.count_nonzero(abs(np.absolute(np.array(out.ravel()) - np.array(y_test.ravel()))) < 0.1) / float(y_test.shape[0])
                    print 'epoch', e, 'cost', np.mean(errs), 'Accuracy', acc

                    # logging result
                    result_file.write(str(e) + ' ' + str(np.mean(errs)+10**9) + ' ' + str(acc) + ' ' + str(kld[1]) +  '\n')
                                  
                    # writing final prediction of tactile
                    if e == n_epochs-1:
                                      
                        for val ,pred, target in zip(X_test, out, y_test):
                            tmp = map(str, list(val) + list(pred) + list(target))
                            write_file.write(' '.join(tmp) +'\n')
                            

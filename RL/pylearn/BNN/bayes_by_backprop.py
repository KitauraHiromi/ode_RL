import theano
import theano.tensor as T
from theano.tensor.shared_randomstreams import RandomStreams
from theano.sandbox.rng_mrg import MRG_RandomStreams
from lasagne.updates import adam
from lasagne.utils import collect_shared_vars

from sklearn.datasets import fetch_mldata
from sklearn.cross_validation import train_test_split
from sklearn import preprocessing

import numpy as np


rnd = RandomStreams(seed=123)
gpu_rnd = MRG_RandomStreams(seed=123)


def activation(x):
    return T.nnet.relu(x)

def activation_old(x):
    return T.tanh(x)


def log_gaussian(x, mu, sigma):
    return -0.5 * np.log(2 * np.pi) - T.log(T.abs_(sigma)) - (x - mu) ** 2 / (2 * sigma ** 2)


def log_gaussian_logsigma(x, mu, logsigma):
    #return -0.5 * np.log(2 * np.pi) - logsigma / 2. - (x - mu) ** 2 / (2. * T.exp(logsigma))
    return -0.5 * np.log(2 * np.pi) - logsigma / 2. - (x - mu) ** 2 / (2. * T.exp(logsigma) ** 2)


def _shared_dataset(data_xy, borrow=True):
    data_x, data_y = data_xy
    shared_x = theano.shared(np.asarray(data_x, dtype=theano.config.floatX), borrow=borrow)
    shared_y = theano.shared(np.asarray(data_y, dtype=theano.config.floatX), borrow=borrow)
    return shared_x, shared_y


def init(shape):
    return np.asarray(
        np.random.normal(0, 0.05, size=shape),
        dtype=theano.config.floatX
    )


def get_random(shape, avg, std):
    return gpu_rnd.normal(shape, avg=avg, std=std)


def get_mnist_data(N=5000):
    
    '''
    preparing mnist data.
    
    arguments
    N: the number of data

    return
    train_data, test_data, train_target, test_target
    
    '''
    mnist = fetch_mldata('MNIST original')
    data = np.float32(mnist.data[:]) / 255.
    idx = np.random.choice(data.shape[0], N)
    data = data[idx]
    target = np.int32(mnist.target[idx]).reshape(N, 1)
    
    train_idx, test_idx = train_test_split(np.array(range(N)), test_size=0.05)
    train_data, test_data = data[train_idx], data[test_idx]
    train_target, test_target = target[train_idx], target[test_idx]
    
    train_target = np.float32(preprocessing.OneHotEncoder(sparse=False).fit_transform(train_target))
    return train_data, test_data, train_target, test_target
    

class BNN:
    
    def __init__(self, input_size, output_size, batch_size, learning_rate=0.001, learning_task="classification"):
        # inputs
        self.x = T.matrix('x')
        self.y = T.matrix('y')
        
        # initial variance 
        sigma_prior = T.exp(3)
        self.learning_task = learning_task
    
        # declaring weights and bias
        # L1
        self.n_input = input_size
        self.n_hidden_1 = 20
        self.W1_mu = theano.shared(value=init((self.n_input, self.n_hidden_1)))
        self.W1_logsigma = theano.shared(value=init((self.n_input, self.n_hidden_1)))
        self.b1_mu = theano.shared(value=init((self.n_hidden_1,)))
        self.b1_logsigma = theano.shared(value=init((self.n_hidden_1,)))

        # L2
        self.n_hidden_2 = 20
        self.W2_mu = theano.shared(value=init((self.n_hidden_1, self.n_hidden_2)))
        self.W2_logsigma = theano.shared(value=init((self.n_hidden_1, self.n_hidden_2)))
        self.b2_mu = theano.shared(value=init((self.n_hidden_2,)))
        self.b2_logsigma = theano.shared(value=init((self.n_hidden_2,)))

        # L3
        self.n_output = output_size
        self.W3_mu = theano.shared(value=init((self.n_hidden_2, self.n_output)))
        self.W3_logsigma = theano.shared(value=init((self.n_hidden_2, self.n_output)))
        self.b3_mu = theano.shared(value=init((self.n_output,)))
        self.b3_logsigma = theano.shared(value=init((self.n_output,)))

        self.all_params = [
            self.W1_mu, self.W1_logsigma, self.b1_mu, self.b1_logsigma,
            self.W2_mu, self.W2_logsigma, self.b2_mu, self.b2_logsigma,
            self.W3_mu, self.W3_logsigma, self.b3_mu, self.b3_logsigma
        ]
        self.all_params = collect_shared_vars(self.all_params)

                        
        if learning_task == "classification":
            a1_mu = activation(T.dot(self.x, self.W1_mu) + self.b1_mu)
            a2_mu = activation(T.dot(a1_mu, self.W2_mu) + self.b2_mu)
            h_mu = T.nnet.softmax(activation(T.dot(a2_mu, self.W3_mu) + self.b3_mu))
            self.output_function = theano.function([self.x], T.argmax(h_mu, axis=1))
            
        elif learning_task == "regression":
            a1_mu = activation(T.dot(self.x, self.W1_mu) + self.b1_mu)
            a2_mu = activation(T.dot(a1_mu, self.W2_mu) + self.b2_mu)
            h_mu = T.dot(a2_mu, self.W3_mu) + self.b3_mu
            self.output_function = theano.function([self.x], h_mu)


    def set_train_data(self, train_data, train_target):
        self.train_data = theano.shared(np.asarray(train_data, dtype=theano.config.floatX))
        self.train_target = theano.shared(np.asarray(train_target, dtype=theano.config.floatX))

        i = T.iscalar()
        sigma_prior = T.exp(-3)
        learning_rate=0.001
        batch_size = 100

        if self.learning_task == "classification":
            objective = self.cross_entropy(batch_size=batch_size, sigma_prior=sigma_prior)
        elif self.learning_task == "regression":
            objective = self.mean_square_loss(batch_size=batch_size, sigma_prior=sigma_prior)
            
        # train function setting
        updates = adam(objective, self.all_params, learning_rate=learning_rate)

        
        self.train_function = theano.function(
            inputs=[i],
            outputs=objective,
            updates=updates,
            givens={
                self.x: self.train_data[i * batch_size: (i + 1) * batch_size],
                self.y: self.train_target[i * batch_size: (i + 1) * batch_size]
            }
        )
        

        self.n_train_batches = int(self.train_data.get_value().shape[0] / float(batch_size))
        
        
    def train_fn(self, n_mini_batch):

        '''
        train bnn network by train_function

        arguments
        the number of mini batch

        return
        batch error
        '''
        return self.train_function(n_mini_batch)
        

    def pred_fn(self, x):
        return self.output_function(x)


    def get_all_params(self):
        return [shared.get_value() for shared in self.all_params]


    def get_all_params_for_kld(self):
        all_params = [shared.get_value() for shared in self.all_params]
        m_s_list = [ np.array(matrix).flatten() for matrix in  all_params]
        params = [ [ m_s_list[2*i][j], m_s_list[2*i+1][j] ] for i in xrange(len(m_s_list)/2) for j in xrange(len(m_s_list[2*i])) ]
        return params
        

    def mean_square_loss(self, n_samples=3, batch_size=100, sigma_prior=T.exp(-3)):

        # building the objective
        # remember, we're evaluating by samples
        log_pw, log_qw, log_likelihood = 0., 0., 0.

        for _ in xrange(n_samples):

            # initializing weights
            # Is avg=0 OK ? 
            epsilon_w1 = get_random((self.n_input, self.n_hidden_1), avg=0., std=sigma_prior)
            epsilon_b1 = get_random((self.n_hidden_1,), avg=0., std=sigma_prior)
            
            W1 = self.W1_mu + T.log(1. + T.exp(self.W1_logsigma)) * epsilon_w1
            b1 = self.b1_mu + T.log(1. + T.exp(self.b1_logsigma)) * epsilon_b1

            epsilon_w2 = get_random((self.n_hidden_1, self.n_hidden_2), avg=0., std=sigma_prior)
            epsilon_b2 = get_random((self.n_hidden_2,), avg=0., std=sigma_prior)
            
            W2 = self.W2_mu + T.log(1. + T.exp(self.W2_logsigma)) * epsilon_w2
            b2 = self.b2_mu + T.log(1. + T.exp(self.b2_logsigma)) * epsilon_b2
            
            epsilon_w3 = get_random((self.n_hidden_2, self.n_output), avg=0., std=sigma_prior)
            epsilon_b3 = get_random((self.n_output,), avg=0., std=sigma_prior)
            
            W3 = self.W3_mu + T.log(1. + T.exp(self.W3_logsigma)) * epsilon_w3
            b3 = self.b3_mu + T.log(1. + T.exp(self.b3_logsigma)) * epsilon_b3
            
            a1 = activation(T.dot(self.x, W1) + b1)
            a2 = activation(T.dot(a1, W2) + b2)
            h = T.dot(a2, W3) + b3


            sample_log_pw, sample_log_qw, sample_log_likelihood = 0., 0., 0.
            
            for W, b, W_mu, W_logsigma, b_mu, b_logsigma in [(W1, b1, self.W1_mu, self.W1_logsigma, self.b1_mu, self.b1_logsigma),
                                                             (W2, b2, self.W2_mu, self.W2_logsigma, self.b2_mu, self.b2_logsigma),
                                                             (W3, b3, self.W3_mu, self.W3_logsigma, self.b3_mu, self.b3_logsigma)]:

                # first weight prior
                sample_log_pw += log_gaussian(W, 0., sigma_prior).sum()
                sample_log_pw += log_gaussian(b, 0., sigma_prior).sum()
                
                # then approximation
                sample_log_qw += log_gaussian_logsigma(W, W_mu, W_logsigma * 2).sum()
                sample_log_qw += log_gaussian_logsigma(b, b_mu, b_logsigma * 2).sum()

            # then the likelihood
            sample_log_likelihood = log_gaussian(self.y, h, sigma_prior).sum()
            
            log_pw += sample_log_pw
            log_qw += sample_log_qw
            log_likelihood += sample_log_likelihood

        log_qw /= n_samples
        log_pw /= n_samples
        log_likelihood /= n_samples
        
        n_batches = self.train_data.shape[0] / float(batch_size)

        return ((1. / n_batches) * (log_qw - log_pw) - log_likelihood).sum() / float(batch_size)
        
        
    def cross_entropy(self, n_samples=3, batch_size=100, sigma_prior=T.exp(-3)):
        
        '''
        cross entropy function?
        '''

        # building the objective
        # remember, we're evaluating by samples
        log_pw, log_qw, log_likelihood = 0., 0., 0.

        for _ in xrange(n_samples):

            # initializing weights
            # Is avg=0 OK ? 
            epsilon_w1 = get_random((self.n_input, self.n_hidden_1), avg=0., std=sigma_prior)
            epsilon_b1 = get_random((self.n_hidden_1,), avg=0., std=sigma_prior)
            
            W1 = self.W1_mu + T.log(1. + T.exp(self.W1_logsigma)) * epsilon_w1
            b1 = self.b1_mu + T.log(1. + T.exp(self.b1_logsigma)) * epsilon_b1

            epsilon_w2 = get_random((self.n_hidden_1, self.n_hidden_2), avg=0., std=sigma_prior)
            epsilon_b2 = get_random((self.n_hidden_2,), avg=0., std=sigma_prior)
            
            W2 = self.W2_mu + T.log(1. + T.exp(self.W2_logsigma)) * epsilon_w2
            b2 = self.b2_mu + T.log(1. + T.exp(self.b2_logsigma)) * epsilon_b2
            
            epsilon_w3 = get_random((self.n_hidden_2, self.n_output), avg=0., std=sigma_prior)
            epsilon_b3 = get_random((self.n_output,), avg=0., std=sigma_prior)
            
            W3 = self.W3_mu + T.log(1. + T.exp(self.W3_logsigma)) * epsilon_w3
            b3 = self.b3_mu + T.log(1. + T.exp(self.b3_logsigma)) * epsilon_b3
            
            a1 = activation(T.dot(self.x, W1) + b1)
            a2 = activation(T.dot(a1, W2) + b2)
            h = T.nnet.softmax(activation(T.dot(a2, W3) + b3))


            sample_log_pw, sample_log_qw, sample_log_likelihood = 0., 0., 0.
            
            for W, b, W_mu, W_logsigma, b_mu, b_logsigma in [(W1, b1, self.W1_mu, self.W1_logsigma, self.b1_mu, self.b1_logsigma),
                                                             (W2, b2, self.W2_mu, self.W2_logsigma, self.b2_mu, self.b2_logsigma),
                                                             (W3, b3, self.W3_mu, self.W3_logsigma, self.b3_mu, self.b3_logsigma)]:

                # first weight prior
                sample_log_pw += log_gaussian(W, 0., sigma_prior).sum()
                sample_log_pw += log_gaussian(b, 0., sigma_prior).sum()
                
                # then approximation
                sample_log_qw += log_gaussian_logsigma(W, W_mu, W_logsigma * 2).sum()
                sample_log_qw += log_gaussian_logsigma(b, b_mu, b_logsigma * 2).sum()

            # then the likelihood
            sample_log_likelihood = log_gaussian(self.y, h, sigma_prior).sum()
            
            log_pw += sample_log_pw
            log_qw += sample_log_qw
            log_likelihood += sample_log_likelihood

        log_qw /= n_samples
        log_pw /= n_samples
        log_likelihood /= n_samples
        
        n_batches = self.train_data.shape[0] / float(batch_size)

        return ((1. / n_batches) * (log_qw - log_pw) - log_likelihood).sum() / float(batch_size)
        

if __name__ == '__main__':

    train_data, test_data, train_target, test_target = get_mnist_data()
    
    input_size = train_data.shape[1]
    output_size = train_target.shape[1]
    batch_size = 100
    
    bnn = BNN(input_size, output_size, batch_size)
    bnn.set_train_data(train_data, train_target)
    
    n_epochs = 100
    # training loop shoule be method
    for e in xrange(n_epochs):
        errs = []
        for b in xrange(bnn.n_train_batches):
            batch_err = bnn.train_fn(b)
            errs.append(batch_err)
        out = bnn.pred_fn(test_data)
        acc = np.count_nonzero(out == np.int32(test_target.ravel())) / float(test_data.shape[0])
        print 'epoch', e, 'cost', np.mean(errs), 'Accuracy', acc

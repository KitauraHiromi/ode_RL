import sklearn
from sklearn.neural_network import MLPRegressor
from random import random
from math import sin
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from load_file import * #my liblary
n_train = 100000
n_test  = 10000
n_state = 0
DATA = 1

if DATA == 1:
    log_file = '../../log/state_action_1dof.log'
    n_state = 1
    X_train, y_train, X_test, y_test = load_file(log_file, n_state=n_state, n_train=n_train, n_test=n_test, sampling="curiosity")

elif DATA == 2:
    log_file = '../../log/state_action_2dof.log'
    n_state = 2
    X_train, y_train, X_test, y_test = load_file(log_file, n_state=n_state, n_train=n_train, n_test=n_test, sampling="curiosity")
    
elif DATA == 0:
    X_train, X_test = 10*(np.random.random(n_train).reshape((-1, 1))-0.5), 10*(np.random.random(n_test).reshape((-1, 1))-0.5)
    y_train, y_test = np.sin(X_train), np.sin(X_test)
    X_train = np.append(X_train, [[1]]*len(X_train), axis=1)
    X_test = np.append(X_test, [[1]]*len(X_test), axis=1)    

NN = MLPRegressor(hidden_layer_sizes=(50, 50), max_iter=10000)
NN.fit(X_train, y_train)
print NN.score(X_test, y_test)

y_pred = NN.predict(X_test)

if DATA is 1:
    plt.plot(zip(*X_test)[0], y_test, 'o', color='red')
    plt.plot(zip(*X_test)[0], y_pred, 'o', color='blue')    
elif DATA is 2:
    fig = plt.figure()
    ax  = Axes3D(fig)
    ax.scatter3D(zip(*X_test)[0], zip(*X_test)[1], y_test, color='red')
    ax.scatter3D(zip(*X_test)[0], zip(*X_test)[1], y_pred, color='blue')
plt.show()

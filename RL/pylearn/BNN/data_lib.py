import numpy as np
from random import randint
from random import gauss

def load_file(filename, n_state=2, n_action=1, n_train=10000, n_test=100, sampling='random'):
    bias = 1.0
    all_lines = []
    # n for data, 1 for bias
    x_train = np.empty((0, n_state+1))
    y_train = np.empty((0, n_action))
    x_test  = np.empty((0, n_state+1))
    y_test  = np.empty((0, n_action))
    with open(filename, 'r') as read_file:
        for line in read_file:
            tmp = line.split(' ')
            state_tmp = tmp[0:n_state]
            state_tmp.append(bias)
            action_tmp = tmp[n_state:n_state+n_action]
            all_lines.append([state_tmp, action_tmp])

    # all_lines_tmp = []
    if sampling is 'curiosity':
        all_lines = sorted(all_lines, key=lambda x: float(x[1][0]), reverse=True)
    
    '''
            # roughly sorting
            idx_tmp = []
            for i in len(all_lines)-1:
                if(all_lines[i][n_state] > 0.01):
                    all_lines_tmp.append(all_lines[i])
                    idx_tmp.append(i)
                    
            for i in len(all_lines)-1:
                if not i in  idx_tmp:
                    all_lines_tmp.append(all_lines[i])
            all_lines = all_lines_tmp
    '''
    
    for i in range(n_train):
        if sampling is 'random':
            rand = randint(0, len(all_lines)-1)
            x_train = np.append(x_train, np.array([all_lines[rand][0]], dtype=np.float32), axis=0)
            y_train = np.append(y_train, np.array([all_lines[rand][1]], dtype=np.float32), axis=0)
        elif sampling is 'curiosity':
            rand = randint(0, 10)
            if rand > 2:
                idx = abs(int(gauss(0, 5)))
                if idx > len(all_lines) - 1:
                    idx = 0
            else:
                idx = randint(0, len(all_lines)-1)
            x_train = np.append(x_train, np.array([all_lines[idx][0]], dtype=np.float32), axis=0)
            y_train = np.append(y_train, np.array([all_lines[idx][1]], dtype=np.float32), axis=0)

    for i in range(n_test):
        rand = randint(0, len(all_lines)-1)
        x_test = np.append(x_test, np.array([all_lines[rand][0]], dtype=np.float32), axis=0)
        y_test = np.append(y_test, np.array([all_lines[rand][1]], dtype=np.float32), axis=0)

    return x_train, y_train, x_test, y_test

def get_sin_dataset(n_train, n_test):
    X_train, X_test = 10*(np.random.random(n_train).reshape((-1, 1))-0.5), 10*(np.random.random(n_test).reshape((-1, 1))-0.5)
    y_train, y_test = np.sin(X_train), np.sin(X_test)
    X_train = np.append(X_train, [[1]]*len(X_train), axis=1)
    X_test = np.append(X_test, [[1]]*len(X_test), axis=1)
    return X_train, y_train, X_test, y_test

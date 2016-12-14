from policy import policy_from_state_density
from probability_distribution import C_gaussian, C_multi_gaussian
from scipy.stats import multivariate_normal
from copy import deepcopy
from matplotlib import pyplot as plt
import numpy as np


filename1 = "test1.log"
filename2 = "test2.log"

def test1(): 

    gaus1 = C_gaussian(20, 1)
    gaus2 = C_gaussian(30, 10)

    print gaus1.mu, gaus2.mu

    state1 = np.array([10])
    state2 = np.array([10])
    state3 = np.array([10])
    action_list = np.array([[-10], [0], [10]])

    with open(filename1, 'w') as write_file:
        write_file.write(' '.join(map(str, state1)) + ' '
                         + ' '.join(map(str, state2)) + ' '
                         + ' '.join(map(str, state3)) + ' '
                         + '\n')
        for i in xrange(100000):
            action1 = policy_from_state_density(state1, gaus1.gaussian, action_list)
            state1 += action1
            action2 = policy_from_state_density(state2, gaus1.gaussian, action_list)
            state2 += action2
            action3 = policy_from_state_density(state3, gaus1.gaussian, action_list)
            state3 += action3
            
            write_file.write(' '.join(map(str, state1)) + ' '
                             + ' '.join(map(str, state2)) + ' '
                             + ' '.join(map(str, state3)) + ' '
                             + '\n')
            
            
def test2():
    
    mu = [0., 0.]
    sigma = [[10000., -5000.],
             [-5000., 10000.]]
    gaus1 = C_multi_gaussian(mu, sigma)

    state = np.array([0, 0])
    action_list = np.array([[-10, -10],
                            [-10, 0],
                            [-10, 10],
                            [0, -10],
                            [0, 0],
                            [0, 10],
                            [10, -10],
                            [10, 0],
                            [10, 10]])

    def print_gaussian(mu, sigma):
        x, y = np.mgrid[-200:200:1, -200:200:1]
        pos = np.empty(x.shape + (2,))
        pos[:, :, 0] = x; pos[:, :, 1] = y
        rv = multivariate_normal(mu, sigma)
        plt.contourf(x, y, rv.pdf(pos))
        plt.show()
        
    print_gaussian(mu, sigma)
    
    with open(filename2, 'w') as write_file:
        write_file.write(' '.join(map(str, state)) + '\n')
        for i in xrange(10000):
            action = policy_from_state_density(state, gaus1.multi_gaussian, action_list)
            state += action

            write_file.write(' '.join(map(str, state)) + '\n')

            
if __name__ == "__main__":
    test2()

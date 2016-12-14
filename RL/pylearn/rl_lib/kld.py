# -*- coding:utf-8 -*-

import math
#from pylearn.BNN.bayes_by_backprop import *
from overload import *


@overload
def kld_1dim(m1, s1, m2, s2):
    '''
    Gaussian distribution
    p(x) = N(x|m1, s1)
    q(x) = N(x|m2, s2)
    
    D_KL(p||q) = ∫ p *ln(p/q) dx
               = E_p[ ln(s1 / s2) ] - 1/( 2 * s1^2 ) * E_p[ ( x - m1 )^2 ] + 1/( 2 * s2^2 ) * E_p[ (x - m2)^2 ]
               = 1/2 * { (s1/s2)^2 + 2log(s2) - 2log(s1) + (m1 - m2)^2 / s2~2 } - 1/2
    '''
    
    return 1/2. * ( (s1/s2)**2 + ((m1-m2) / s2)**2 ) + math.log(s2) - math.log(s1) - 1/2.

    
@overload
def kld_1dim(theta1, theta2):
    
    return kld_1dim(theta1[0], theta1[1], theta2[0], theta2[1])

    
def kld_multi_dim(theta1, theta2):
    
    return sum([ kld_1dim(th1, th2) for th1, th2 in zip(theta1, theta2) ])
    
    
@overload
def kld_logsigma_1dim(m1, logs1, m2, logs2):
    '''
    Gaussian distribution
    p(x) = N(x|m1, logs1)
    q(x) = N(x|m2, logs2)
    
    D_KL(p||q) = ∫ p *ln(p/q) dx
               = E_p[ logs1 - logs2 ] - 1/( 2 * s1^2 ) * E_p[ ( x - m1 )^2 ] + 1/( 2 * s2^2 ) * E_p[ (x - m2)^2 ]
               = 1/2 * { exp{2*(logs1 - logs2)} + 2*logs2 - 2*logs1 + (m1 - m2)^2 / exp{2*logs2} } - 1/2
    '''

    return 1/2. * ( math.exp(2*logs1 - 2*logs2) + ((m1-m2) / math.exp(logs2))**2 ) + logs2 - logs1  - 1/2.

    
@overload
def kld_logsigma_1dim(theta1, theta2):
    
    return kld_logsigma_1dim(theta1[0], theta1[1], theta2[0], theta2[1])

    
def kld_logsigma_multi_dim(theta1, theta2):
    
    return sum([ kld_logsigma_1dim(th1, th2) for th1, th2 in zip(theta1, theta2) ])
    

    
if __name__ == "__main__":
    print kld_logsigma_multi_dim( [[1, 0.1], [3, 0.4], [5, 0.6]], [[2, 0.4], [5, 0.2], [7, 0.9]] )
    print kld_logsigma_multi_dim( [[2, 0.4], [5, 0.2], [7, 0.9]], [[2, 0.4], [5, 0.2], [7, 0.9]] )

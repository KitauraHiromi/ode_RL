from math import exp, sqrt, pi
from scipy.stats import multivariate_normal


def F_gaussian(x):
    return exp( (-1/2./gaussian.sigma**2) * (x-gaussian.mu)**2 ) / sqrt(2*pi) / gaussian.sigma

    
class C_gaussian:
    def __init__(self, mu, sigma):
        self.mu = mu
        self.sigma = sigma

    def gaussian(self, x):
        return exp( (-1/2./self.sigma**2) * (x-self.mu)**2 ) / sqrt(2*pi) / self.sigma

        
class C_multi_gaussian:
    def __init__(self, mu_matrix, sigma_matrix):
        self.mu = mu_matrix
        self.sigma = sigma_matrix

    def multi_gaussian(self, vec_x):
        return multivariate_normal.pdf(vec_x, self.mu, self.sigma)

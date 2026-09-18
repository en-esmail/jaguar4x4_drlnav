import numpy as np

class OUNoise:
    def __init__(self, size, mu=0.0, theta=0.15, sigma=0.2):
        self.size = size
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.reset()

    def reset(self):
        self.state = np.ones(self.size) * self.mu

    def sample(self):
        dx = self.theta * (self.mu - self.state) + self.sigma * np.random.randn(self.size)
        self.state = self.state + dx
        return self.state

class GaussianNoise:
    def __init__(self, size, mu=0.0, sigma=0.1):
        self.size = size
        self.mu = mu
        self.sigma = sigma

    def sample(self):
        return np.random.normal(self.mu, self.sigma, size=self.size)

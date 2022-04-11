
import torch
import matplotlib.pyplot as plt
import numpy as np
import os
import time
from termcolor import cprint
from utils_numpy_filter import NUMPYIEKF
from utils import prepare_data



class InitProcessCovNet(torch.nn.Module):

    def __init__(self):
        super(InitProcessCovNet, self).__init__()

        self.beta_process = 3 * torch.ones(2).double()
        self.beta_initialization = 3 * torch.ones(2).double()

        self.factor_initial_covariance = torch.nn.Linear(1, 6, bias=False).double()
        """parameters for initializing covariance"""
        self.factor_initial_covariance.weight.data[:] /= 10

        self.factor_process_covariance = torch.nn.Linear(1, 6, bias=False).double()
        """parameters for process noise covariance"""
        self.factor_process_covariance.weight.data[:] /= 10
        self.tanh = torch.nn.Tanh()

    def forward(self, iekf):
        return

    def init_cov(self, iekf):
        alpha = self.factor_initial_covariance(torch.ones(1).double()).squeeze()
        beta = 10 ** (self.tanh(alpha))
        return beta

    def init_processcov(self, iekf):
        alpha = self.factor_process_covariance(torch.ones(1).double())
        beta = 10 ** (self.tanh(alpha))
        return beta


class MesNet(torch.nn.Module):
    def __init__(self):
        super(MesNet, self).__init__()
        self.beta_measurement = 3 * torch.ones(2).double()
        self.tanh = torch.nn.Tanh()

        self.cov_net = torch.nn.Sequential(torch.nn.Conv1d(6, 32, 5),
                                           torch.nn.ReplicationPad1d(4),
                                           torch.nn.ReLU(),
                                           torch.nn.Dropout(p=0.5),
                                           torch.nn.Conv1d(32, 32, 5, dilation=3),
                                           torch.nn.ReplicationPad1d(4),
                                           torch.nn.ReLU(),
                                           torch.nn.Dropout(p=0.5),
                                           ).double()
        "CNN for measurement covariance"
        self.cov_lin = torch.nn.Sequential(torch.nn.Linear(32, 2),
                                           torch.nn.Tanh(),
                                           ).double()
        self.cov_lin[0].bias.data[:] /= 100
        self.cov_lin[0].weight.data[:] /= 100

    def forward(self, u, iekf):
        y_cov = self.cov_net(u).transpose(0, 2).squeeze()
        z_cov = self.cov_lin(y_cov)
        z_cov_net = self.beta_measurement.unsqueeze(0) * z_cov
        measurements_covs = (iekf.cov0_measurement.unsqueeze(0) * (10 ** z_cov_net))
        return measurements_covs
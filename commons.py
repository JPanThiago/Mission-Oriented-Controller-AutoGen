"""
This is a class for computing objective function
===========================================================================
Functions:
    compute: Compute loss
    write_control_to_file: Write the hyperparameters of the controller to the file "parameter/parameter_controller.txt"
    write_list_to_file: Write the hyperparameters of the Koopman model to the file "parameter/parameter_lifting.txt"
    get_configspace: Construct the hyperparameter search space config_space
"""

import numpy as np
import math
import time
import ConfigSpace as CS
from hpbandster.core.worker import Worker
import matlab.engine
eng = matlab.engine.start_matlab()

class MyWorker(Worker):
    def __init__(self, *args, sleep_interval = 0, degree, delay, Nx, Nu, matlab_engine = 'Yes', **kwargs):
        super().__init__(*args, **kwargs)

        self.sleep_interval = sleep_interval
        self.degree = degree # the degree shows the maximum dimension of the lifting function
        self.delay = delay   # the delay indicates the number of past states used in the lifting function construction
        self.Nx = Nx         # the dimension of the system state
        self.Nu = Nu         # the dimension of the system input
        self.matlab_engine = matlab_engine

    def compute(self, config, budget, **kwargs):
        # Determine the optimization matrix D
        Parameter_lifting = []
        N = int(np.prod(range((self.Nx * (self.delay + 1) + 1), (self.Nx * (self.delay + 1) + self.degree + 1))) / math.factorial(self.degree))
        for i in range(N-1-self.Nx):
            Parameter_lifting.append(config['x{}'.format(i+1)])
        self.write_list_to_file('parameter/parameter_lifting.txt', config)
        self.write_control_to_file(self.run_id, 'parameter/parameter_controller.txt', config)

        res = eng.model(budget)
        print('loss:', res, ' ID:', kwargs['config_id'], ' Fidelity:', budget)

        time.sleep(self.sleep_interval)

        return({
                    'loss': float(res),
                    'info': res
                })

    @staticmethod
    def write_control_to_file(robot, filename, hyperparameter):
        if robot == 'DP':
            num_list = [hyperparameter['kp'] / 10, hyperparameter['ki'] / 20, hyperparameter['kd'] / 1000, hyperparameter['Np'], hyperparameter['Q'] * 100, hyperparameter['R'] * 100, hyperparameter['P']]
        elif robot == 'TDVPT':
            num_list = [hyperparameter['kp'] / 2000, hyperparameter['ki'] / 1000, hyperparameter['kd'] / 2000, hyperparameter['Np'], hyperparameter['Q'] * 100, hyperparameter['R'] * 100, hyperparameter['P']]
        elif robot == 'SoftRobot':
            num_list = [hyperparameter['kp'] / 200, hyperparameter['ki'] / 100, hyperparameter['kd'] / 1000, hyperparameter['Np'], hyperparameter['Q'] * 100, hyperparameter['R'] * 100, hyperparameter['P']]
        elif robot == 'AUV':
            num_list = [hyperparameter['kp'] / 2, hyperparameter['ki'] / 200, hyperparameter['kd'], hyperparameter['Np'], hyperparameter['Q'] * 100, hyperparameter['R'] * 100, hyperparameter['P']]
        else:
            num_list = [hyperparameter['kp'], hyperparameter['ki'] / 10, hyperparameter['kd'] * 15, hyperparameter['Np'], hyperparameter['Q'] * 100, hyperparameter['R'] * 100, hyperparameter['P']]
        with open(filename, 'w') as file:
            file.write(' '.join(str(num) for num in num_list))
        file.close()

    def write_list_to_file(self, filename, hyperparameter):
        Parameter_lifting = []
        N = int(np.prod(range((self.Nx * (self.delay + 1) + 1), (self.Nx * (self.delay + 1) + self.degree + 1))) / math.factorial(self.degree))
        for i in range(N - 1 - self.Nx):
            Parameter_lifting.append(hyperparameter['x{}'.format(i + 1)])
        with open(filename, 'w') as file:
            file.write(' '.join(str(num) for num in Parameter_lifting))
        file.close()

    @staticmethod
    def get_configspace(degree, delay, Nx, Nu):
        config_space = CS.ConfigurationSpace()

        # Hyperparameters of the Koopman model
        N = int(np.prod(range((Nx * (delay + 1) + 1), (Nx * (delay + 1) + degree + 1))) / math.factorial(degree))
        for i in range(N-1-Nx):
            config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('x{}'.format(i+1), lower=0, upper=1))

        # Hyperparameters of the controller
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('Np', lower=4, upper=15))
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('kp', lower=0, upper=15))
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('ki', lower=0, upper=15))
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('kd', lower=0, upper=15))
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('Q', lower=1, upper=20))
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('P', lower=1, upper=20))
        config_space.add_hyperparameter(CS.UniformIntegerHyperparameter('R', lower=1, upper=20))
        return(config_space)


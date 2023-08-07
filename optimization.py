"""
The main program for optimization
===========================================================================
Parameters:
    eta: The proportion of successive halving, is employed to calculate the proportion of configurations left from low-fidelity to high-fidelity
    min_budget: Minimum budget used during the optimization
    max_budget: Maximum budget used during the optimization
    random_fraction: The proportion of random sampling, is employed to balance exploitation and exploration
    n_iterations: Times of iterations performed by the optimizer w.r.t. max_SH_iter
    min_points_in_model: Minimum number of points required for TPE-based model
"""

import logging
logging.basicConfig(level=logging.WARNING)
import argparse
import hpbandster.core.nameserver as hpns
from hpbandster.optimizers import BOHB as BOHB
from commons import MyWorker
from bayes_opt.util import Colours
import robot_type

parser = argparse.ArgumentParser(description='The mission-oriented controller auto-generation framework')
parser.add_argument('--eta', type=float, default=3)
parser.add_argument('--min_budget', type=float, default=1)
parser.add_argument('--max_budget', type=float, default=27)
parser.add_argument('--random_fraction', type=float, default=0.1)
parser.add_argument('--n_iterations', type=int, default=50)
parser.add_argument('--min_points_in_model', type=int, default=None)
args = parser.parse_args()

robot = robot_type.show_custom_dialog() # choose the robot type for optimization
if robot == 'DP':
    degree = 3
    delay = 2
    Nx = 2
    Nu = 1
elif robot == 'TDVPT':
    degree = 3
    delay = 2
    Nx = 2
    Nu = 2
elif robot == 'SoftRobot':
    degree = 3
    delay = 2
    Nx = 2
    Nu = 3
elif robot == 'AUV':
    degree = 2
    delay = 1
    Nx = 6
    Nu = 2
elif robot == 'AUV_3D':
    degree = 2
    delay = 1
    Nx = 12
    Nu = 3
elif robot == 'RoboticPenguin':
    degree = 2
    delay = 2
    Nx = 3
    Nu = 1
else:
    degree = 2
    delay = 1
    Nx = 6
    Nu = 1

# Step 1: Start a nameserver
NS = hpns.NameServer(run_id=robot, host='127.0.1.1', port=None)
NS.start()

# Step 2: Start a worker
w = MyWorker(sleep_interval=0, nameserver='127.0.1.1', run_id=robot, degree=degree, delay=delay, Nx=Nx, Nu=Nu)
w.run(background=True)
f1 = open('parameter/parameter_robot.txt', "w")
f1.write(robot)
f1.close()
f2 = open('parameter/parameter_koopman.txt', "w")
f2.write(str(degree) + ' ' + str(delay) + ' ' + str(Nx) + ' ' + str(Nu))
f2.close()

# Step 3: Run an optimizer
space = w.get_configspace(degree, delay, Nx, Nu)
print(Colours.yellow("--- Optimizing ---"))
bohb = BOHB(configspace=space,
            run_id=robot, nameserver='127.0.1.1',
            min_budget=args.min_budget, max_budget=args.max_budget,
            eta=args.eta, min_points_in_model=args.min_points_in_model,
            random_fraction=args.random_fraction)
res = bohb.run(n_iterations=args.n_iterations*bohb.max_SH_iter)

# Step 4: Shutdown
bohb.shutdown(shutdown_workers=True)
NS.shutdown()

# Step 5: Analysis
id2config = res.get_id2config_mapping()
incumbent = res.get_incumbent_id()

MAE = res.data[incumbent]
MAE1 = MAE.results[args.max_budget]

print('Best found configuration:', id2config[incumbent]['config'])
print('Num:', incumbent)
print('MAE:', MAE1['loss'])
print('A total of %i unique configurations where sampled.' % len(id2config.keys()))
print('A total of %i runs where executed.' % len(res.get_all_runs()))
print('Total budget corresponds to %.1f full function evaluations.' % (sum([r.budget for r in res.get_all_runs()]) / args.max_budget))

# Step 6: Save hyperparameter
hyperparameter = id2config[incumbent]['config']
w.write_list_to_file('parameter/parameter_lifting.txt', hyperparameter)
w.write_control_to_file(robot, 'parameter/parameter_controller.txt', hyperparameter)




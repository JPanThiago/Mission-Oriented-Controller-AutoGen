"""
Collect data from robot dynamics

"""

import numpy as np
import argparse
from Utility import data_collecter

def main():
    data_collect = data_collecter(args.env)
    
    # Save data
    for i in range(60):
        file_path = "data/" + args.env + "/" + '%s.txt' % i
        f1 = open(file_path, 'w')
        data = data_collect.collect_koopman_data(args.traj_num)
        np.savetxt(f1, data, fmt='%f')
        f1.close()

if __name__ == "__main__":
    # Collect data for damping pendulum
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=str, default="DP")
    parser.add_argument("--traj_num", type=int, default=1000)
    args = parser.parse_args()
    main()
    
    # Collect data for autonomous underwater vehicles
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=str, default="AUV")
    parser.add_argument("--traj_num", type=int, default=1000)
    args = parser.parse_args()
    main()
    
    # Collect data for the 3D task of autonomous underwater vehicles
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", type=str, default="AUV_3D")
    parser.add_argument("--traj_num", type=int, default=1000)
    args = parser.parse_args()
    main()


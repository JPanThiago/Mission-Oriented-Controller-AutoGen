# Auto-Generation of Mission-Oriented Robot Controllers Using Bayesian-Based Koopman Operator
This code can be employed to generate controllers for various robots with diverse data sources. Meanwhile, example scripts including the damping pendulum (DP), tendon-driven visual pan-tilt (TDVPT), soft robot, autonomous underwater vehicles (AUV), and penguin-inspired robot are provided for recreating these models and controllers on your machine.

![](https://github.com/JPanThiago/mission_oriented_controller_autogeneration_framework/blob/main/fig.alg.jpg)

## Environment Configuration
Notably, the optimization program is implemented in Python, while the control program considers the sampled data from [Bruder *et al*.](https://github.com/ramvasudevan/soft-robot-koopman) and the constructed robotic penguin, both implemented in MATLAB. Therefore, environment configuration requires considering the integration of Python with MATLAB.

Firstly, it is essential to verify the corresponding [versions](https://ww2.mathworks.cn/support/requirements/python-compatibility.html) of Python and MATLAB. Then, locate the `extern>engines>python` folder in the MATLAB directory. Finally, open the command prompt (cmd) at this path and run

```
python setup.py install
```

In particular, the relevant information can be found in [mathworks](https://ww2.mathworks.cn/help/matlab/matlab-engine-for-python.html?lang=en). Besides, our specific version number is 

![Static Badge](https://img.shields.io/badge/Python-3.8-blue)
![Static Badge](https://img.shields.io/badge/MATLAB-R2021a-blue)

Meanwhile, the Python packages that need to be followed include

![Static Badge](https://img.shields.io/badge/hpbandster-0.7.4-blue)
![Static Badge](https://img.shields.io/badge/gym-0.26.2-blue)
![Static Badge](https://img.shields.io/badge/scipy-1.9.0-blue)
![Static Badge](https://img.shields.io/badge/bayesian--optimization-1.2.0-blue)
![Static Badge](https://img.shields.io/badge/numpy-1.23.1-blue)

## Optimization Steps
**Note**: When analyzing the results of the paper, you can skip the optimization phase and directly run the subsequent examples. Meanwhile, you can modify the specific parts related to robot dynamics and control according to your individual needs.

* **Step 1**: Data Collection

  Collect robot state-input pairs from dynamics including DP and AUV.

  ```
  run data_collect.py
  ```

  Integrate robot state-input pairs for subsequent work. The dataset is stored in the subfolder ''[data](/data/)''.

  ```
  run data_integrate.m
  ```
  
* **Step 2**: Controller Optimization

  Start optimization after selecting the robot to be optimized.

  ```
  run optimization.py
  ```

  During the optimization process, the objective function will be calculated by calling `model.m`, and the model used in optimization will also be stored in the subfolder ''[model_koopman_tem](/model_koopman_tem/)''. In particular, the parameters in the Koopman-based model file are described in the following table.

  <div align="center">
   
  Parameter  | Description
  | :--- | :---
  type  | The type of Koopman model.
  degree  | The maximum dimension of the lifting function.
  delay  | The number of past states used in the lifting function construction.
  horizon  | The length of the prediction horizon.
  name  | The type of robot to be optimized.
  optimization  | Determine whether the current stage is during controller optimization. (`Yes`, `No`).
  matrixD  | The matrix ***D*** delivered by BOHB for Koopman operator optimization.
  datanoise  | Choose whether to identify the Koopman-based model with data sampling noise (`Yes`, `No`).
  
  </div>

  Besides, the parameters in the control file are also provided.
  
  <div align="center">
   
  Parameter  | Description
  | :--- | :---
  budget  | Determine the budget needed for the current optimization.
  underopt  | Determine whether to optimize the controller, if not, run the saved results (`Yes`, `No`).
  datanoise  | Choose whether to run the cases with data sampling noise (`Yes`, `No`).
  noise  | Choose whether to run the cases with environmental perturbation (`Yes`, `No`).
  failurecase  | Choose whether to run the failure cases (`Yes`, `No`).
  
  </div>

## Running Example
**Note**: Most cases are solved online, so it may take a few minutes to obtain the results.
* Simulation: `run example_simulation.m`, `run example_simulation_datanoise.m`

  This visualization illustrates the simulation results of path tracking for various robots, including the DP, TDVPT, soft robot, and AUV. The primary focus is on examining the impact of the un-optimized Koopman-based Model Predictive Control (MPC) and the proposed controller generation algorithm. Additionally, simulation results under data sampling noise are also presented.
  
* Penguin-Inspired Robot: `run example_RoboticPenguin.m`, `run example_RoboticPenguin_full.m`

  This illustration provides the controller auto-generation case for the penguin-inspired robot using both the three-state dynamics after dimensionality reduction and the to-be-identified six-state model.

* AUV 3D Task: `run example_AUV_3Dtask.m`

  This study emphasizes the application of the proposed algorithm in AUV 3D path control, showcasing its capacity to tackle intricate cross-coupling scenarios in multiple directions.

* Failure Case: `run example_failurecase.m`

  This case introduces Gaussian noise to simulate environmental uncertainty in real-world scenarios. As the noise increases, the proposed method cannot complete the task.

* Comparative Case: `run example_SoftRobot_com.m`

  This case depicts simulation results for the soft robot, where the Koopman-based control model is identical to the interaction model, similar to the work of [Bruder *et al*.](https://github.com/ramvasudevan/soft-robot-koopman) for comparison.

* Prediction Error `run example_prediction_error.m`

  This case is employed to calculate the prediction error.

## Description of Folders

<div align="center">
 
Folder  | Description
| :--- | :---
control_test  | The control code includes the dynamics and MPC for various robots. Specifically, the subfolder ''[Robotic Penguin](/control_test/RoboticPenguin/)'' contains the dynamics of the penguin-inspired robot.
data  | The pre-sampling dataset for training models.
model_environment  | The un-optimized Koopman-based models for comparative experiments and environment construction of the TDVPT and soft robot.
model_koopman  | The optimized Koopman-based models.
model_koopman_tem  | Koopman-based models stored during optimization.
parameter  | Detailed parameters stored during optimization.
ref_trajectories  | Soft robot trajectories provided by [Bruder *et al*](https://github.com/ramvasudevan/soft-robot-koopman).
results  | Several supplementary results include the Koopman-based modeling errors for various robots, the prediction error curve for the soft robot, and the control error for the soft robot.

</div>

## References
https://github.com/ramvasudevan/soft-robot-koopman

https://github.com/automl/HpBandSter

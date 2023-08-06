%-----------------------------------------%
%--- Calculate the objective function ----%
%-----------------------------------------%
function loss = model(budget)
    addpath(genpath('.\control_test'));
    addpath (genpath('.\parameter'));
    
    %% Determine the Basic Parameters
    Parameter_robot = textread('parameter_robot.txt', '%s');
    robot = Parameter_robot{1, 1};
    Parameter_Koopman = load('parameter_koopman.txt');
    Parameter_lifting = load('parameter_lifting.txt');
    
    degree = Parameter_Koopman(1, 1);
    delay = Parameter_Koopman(1, 2);
    Nx = Parameter_Koopman(1, 3);
    Nu = Parameter_Koopman(1, 4);
    D = matrix_D(Parameter_lifting, Nx);

    %% Identify Koopman Model from Data
    % import data from file
    data_name = strcat(robot, '.mat');
    data_path = 'data/';
    data = load([data_path, data_name]);
    
    %{
    type:          the type of Koopman model
    degree:        the maximum dimension of the lifting function
    delay:         the number of past states used in the lifting function construction
    horizon:       the length of the prediction horizon     
    name:          the type of robot to be optimized
    optimization:  determine whether the current stage is during controller optimization
    matrixD:       the matrix D delivered by BOHB for Koopman operator optimization
    datanoise:     choose whether to identify the Koopman-based model with data sampling noise
    %}
    koopman_model = koopman(data, ...
                    'type', 'poly', ...    
                    'degree', degree, ... 
                    'delay', delay, ...
                    'horizon', 10, ...
                    'name', robot, ...
                    'optimization', 'Yes', ...
                    'matrixD', D, ...
                    'datanoise', 'No');
    koopman_model = koopman_model.train_model;
    koopman_model.save_model_tem(koopman_model);

    %% Train Controller
    %{
    budget:       determine the budget needed for the current optimization
    underopt:     determine whether to optimize the controller, if not, run the saved results
    datanoise:    choose whether to run the cases with data sampling noise
    noise:        choose whether to run the cases with environmental perturbation
    failurecase:  choose whether to run the failure cases
    %}
    underopt = 'Yes'; 
    datanoise = 'No'; 
    noise = 'No'; 
    failurecase = 'No'; 
    
    if strcmp(robot, 'DP')
        [~, DP_loss_avg, ~, ~] = control_DP(budget, underopt, datanoise, noise, failurecase);
        loss = DP_loss_avg;
    elseif strcmp(robot, 'TDVPT')
        [~, TDVPT_loss_avg, ~, ~] = control_TDVPT(3, budget, underopt, datanoise, noise, failurecase);
        loss = TDVPT_loss_avg;
    elseif strcmp(robot, 'SoftRobot')
        [~, SoftRobot_loss_avg, ~, ~] = control_SoftRobot(budget, underopt, datanoise, noise, failurecase);
        loss = SoftRobot_loss_avg;
    elseif strcmp(robot, 'AUV')   
        [~, AUV_loss_avg, ~, ~] = control_AUV(budget, underopt, datanoise, noise, failurecase);
        loss = AUV_loss_avg;
    elseif strcmp(robot, 'AUV_3D')   
        [~, AUV_3D_loss_avg, ~] = control_AUV_3D(budget, underopt);
        loss = AUV_3D_loss_avg;
    elseif strcmp(robot, 'RoboticPenguin') 
        [~, RoboticPenguin_loss_avg] = control_RoboticPenguin(budget, underopt);
        loss = RoboticPenguin_loss_avg;
    else
        [~, RoboticPenguin_full_loss_avg] = control_RoboticPenguin_full(budget, underopt);
        loss = RoboticPenguin_full_loss_avg;
    end
end

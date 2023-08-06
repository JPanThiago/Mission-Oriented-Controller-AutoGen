%----------------------------------------------------%
%--- The case of calculating the prediction error ---%
%----------------------------------------------------%
close all;
clear;
addpath (genpath('control_test'))
addpath (genpath('parameter'))

optimization = questdlg('Whether the model is an optimization model?', ...
         'Model', 'Yes', 'No', 'Yes');
     
%% Determine Basic Parameters    
if strcmp(optimization, 'Yes') == 1
    Parameter_controller = load('parameter_controller.txt');
    Parameter_Koopman = load('parameter_koopman.txt');
    Parameter_lifting = load('parameter_lifting.txt');
    Parameter_robot = textread('parameter_robot.txt', '%s');
    robot = Parameter_robot{1, 1};

    degree = Parameter_Koopman(1, 1);
    delay = Parameter_Koopman(1, 2);
    Nx = Parameter_Koopman(1, 3);
    Nu = Parameter_Koopman(1, 4);
    D = matrix_D(Parameter_lifting, Nx);
else
    degree = 2;
    delay = 3;
    D = [];
end

%% Identify Koopman Model from Data
% import data from file
if strcmp(optimization, 'Yes') == 1
    data_name = strcat(robot, '.mat');
    data_path = 'data/';
else
    [data_name, data_path] = uigetfile('data/*.mat', 'Choose data file for system identification...');
end
data = load([data_path, data_name]);

koopman_model = koopman(data, ...
                'type', 'poly', ...    
                'degree', degree, ... 
                'delay', delay, ...
                'horizon', 10, ...
                'name', data_name(1 : find('.'==data_name) - 1), ...
                'optimization', optimization, ...
                'matrixD', D, ...
                'datanoise', 'No');

%% Train Koopman Model
koopman_model = koopman_model.train_model;
koopman_model.save_model(koopman_model);

%% Validate Koopman Model
% calculate the modeling error as shown in fig.2
[results, loss] = koopman_model.val;

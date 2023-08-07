%------------------------------------------------------------------------------%
%--- Supplementary results for the prediction error curve of the soft robot ---%
%------------------------------------------------------------------------------%
close all;
clear;
addpath (genpath('..\'));

%{
type:          the type of Koopman model
degree:        the maximum dimension of the lifting function
delay:         the number of past states used in the lifting function construction
horizon:       the length of the prediction horizon     
name:          the type of the robot to be optimized
optimization:  determine whether the current stage is during controller optimization
matrixD:       the matrix D delivered by BOHB for Koopman operator optimization
datanoise:     choose whether to identify the Koopman-based model with data sampling noise
%}
koopman_model = koopman(load(['data/', 'SoftRobot.mat']),...
                'type', 'poly',...    
                'degree', 2,... 
                'delay', 3,...
                'horizon', 10,...
                'name', 'SoftRobot',...
                'optimization', 'No',...
                'matrixD', [],...
                'datanoise', 'No');
            
koopman_model = koopman_model.train_model;
[results, loss] = koopman_model.val;
    
prey = results{1, 1}.sim.prey;
real = results{1, 1}.real.y;

figureUnits = 'centimeters';
fig = figure;
set(gcf, 'unit', 'centimeters', 'position', [20, 14, 8, 4.5])
set(gca, 'Fontsize', 8, 'FontName', 'Times New Roman', 'position', [0.105, 0.15, 0.89, 0.85]);

p1 = plot(real(1 : 75, 1), real(1 : 75, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
p2 = plot(prey(1 : 70, 11), prey(1 : 70, 12), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [1 0.6 0.46]);
hold on
p3 = plot(prey(1 : 65, 21), prey(1 : 65, 22), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.86 0.39 0.43]);
hold on

for i = 17 : 24
    j = i * 3 - 2;
    prey1 = [prey(j, 11); real(j + 5, 1)];
    prey2 = [prey(j, 12); real(j + 5, 2)];
    p4 = plot(prey1, prey2, '--', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.6 0.6 0.6]);
    hold on
end

for i = 1 : 10
    j = i * 5 - 4;
    prey1 = [prey(j, 11); real(j + 5, 1)];
    prey2 = [prey(j, 12); real(j + 5, 2)];
    p4 = plot(prey1, prey2, '--', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.6 0.6 0.6]);
    hold on
end

for i = 15 : 22
    j = i * 3 - 1;
    prey1 = [prey(j, 21); real(j + 10, 1)];
    prey2 = [prey(j, 22); real(j + 10, 2)];
    p5 = plot(prey1, prey2, '--', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.6 0.6 0.6]);
    hold on
end

for i = 1 : 9
    j = i * 5 - 4;
    prey1 = [prey(j, 21); real(j + 10, 1)];
    prey2 = [prey(j, 22); real(j + 10, 2)];
    p5 = plot(prey1, prey2, '--', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.6 0.6 0.6]);
    hold on
end

h = legend([p1 p2 p3 p4], {'Real', '5th step prediction', '10th step prediction', 'Deviation'}, 'FontName', 'Times New Roman', 'Fontsize', 8);
set(h,...
    'Position', [0.562511593242304 0.183422971633699 0.406181007064468 0.326470579469905],...
    'Orientation', 'horizontal',...
    'NumColumns', 1,...
    'FontSize', 8,...
    'EdgeColor','none');
h.ItemTokenSize = [20, 100];

axis([-5.95 -3.9 -2.05 -0.55]);
set(gca, 'XTick', [-5.5 -5 -4.5 -4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-2 -1.5 -1 -0.5], 'FontName', 'Times New Roman', 'Fontsize', 8);

ylabel('{\it{y}} (cm)', 'Fontsize', 8, 'FontName', 'Times New Roman')
xlabel('{\it{x}} (cm)', 'Fontsize', 8, 'FontName', 'Times New Roman')
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

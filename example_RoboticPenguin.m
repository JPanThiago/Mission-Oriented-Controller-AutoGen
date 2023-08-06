%------------------------------------------------------------------------------------------------------------%
%--- The case of the penguin-inspired robot based on the three-state model after dimensionality reduction ---%
%------------------------------------------------------------------------------------------------------------%
close all
clear
addpath(genpath('.\control_test'));
addpath(genpath('.\parameter'));

[sim, RoboticPenguin_loss_avg] = control_RoboticPenguin();

%% Plot the Results of the Penguin-Inspired Robot based on the Dimensionality Reduction Model
figure
set(gcf, 'unit', 'centimeters', 'position', [15, 12, 8.5, 4.5])
set(gca, 'Fontsize', 8, 'FontName', 'Times New Roman', 'position', [0.135, 0.16, 0.8, 0.71]);

p1 = plot(sim.t1.Xt(:, 1), sim.t1.Xt(:, 2), '-', 'LineWidth', 5, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
p2 = plot(sim.t1.Xt(:, 1), sim.t1.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
p3 = plot(sim.t1.X(:, 1), sim.t1.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on
axis equal
axis([-3.1 3.1 -1.4 1.7]);
set(gca, 'XTick', [-3 -2 -1 0 1 2 3], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-1.5 -1 -0.5 0 0.5 1 1.5], 'FontName', 'Times New Roman', 'Fontsize', 8);

h = legend([p1 p3], {'Target', 'Proposed'}, 'FontName', 'Times New Roman', 'Fontsize', 8);
set(h,...
    'Position', [0.0693146493343918 0.891998787984169 0.861370701331216 0.0970588213380646],...
    'Orientation', 'horizontal',...
    'NumColumns', 3,...
    'FontSize', 8,...
    'EdgeColor', 'none');
h.ItemTokenSize = [20, 100];
annotation('line', [0.290064215686275 0.372536764705883], [0.94034060846561 0.94034060846561], 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1])

ylabel('{\it{y}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
xlabel('{\it{x}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

figure
set(gcf, 'unit', 'centimeters', 'position', [24, 12, 8.5, 4.5])
set(gca, 'Fontsize', 8, 'FontName', 'Times New Roman', 'position', [0.135, 0.16, 0.8, 0.71]);
p1 = plot(sim.t2.Xt(:, 1), sim.t2.Xt(:, 2), '-', 'LineWidth', 5, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
p2 = plot(sim.t2.Xt(:, 1), sim.t2.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
p3 = plot(sim.t2.X(:, 1), sim.t2.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on
axis equal

axis([-3.1 3.1 -1.4 1.7]);
set(gca, 'XTick', [-3 -2 -1 0 1 2 3], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-1.5 -1 -0.5 0 0.5 1 1.5], 'FontName', 'Times New Roman', 'Fontsize', 8);

h = legend([p1 p3], {'Target', 'Proposed'}, 'FontName', 'Times New Roman', 'Fontsize', 8);
set(h,...
    'Position', [0.0693146493343918 0.891998787984169 0.861370701331216 0.0970588213380646],...
    'Orientation', 'horizontal',...
    'NumColumns', 3,...
    'FontSize', 8,...
    'EdgeColor', 'none');
h.ItemTokenSize = [20, 100];
annotation('line', [0.290064215686275 0.372536764705883], [0.94034060846561 0.94034060846561], 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1])

ylabel('{\it{y}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
xlabel('{\it{x}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

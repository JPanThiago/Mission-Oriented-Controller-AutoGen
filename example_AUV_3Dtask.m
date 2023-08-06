%-------------------------------------------------------------------------%
%------------------ The case of 3D path control for AUV ------------------%
%-------------------------------------------------------------------------%
close all
clear 
addpath(genpath('.\control_test'));
addpath (genpath('.\parameter'));

[sim, AUV_loss_avg, AUV_loss_avg_un] = control_AUV_3D();

%% Illustration of the 3D Path Tracking for AUV
figure
set(gcf, 'unit', 'centimeters', 'position', [17.7, 12, 12.6, 7])
set(gca, 'Fontsize', 10, 'FontName', 'Times New Roman', 'position', [0.13, 0.08, 0.8, 0.82]);

p1 = plot3(sim.Xt(:, 1), sim.Xt(:, 2), sim.Xt(:, 3), '-', 'LineWidth', 6, 'Color', [0.8, 0.8, 0.8]);
hold on
p2 = plot3(sim.Xt(:, 1), sim.Xt(:, 2), sim.Xt(:, 3), '-', 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1]);
hold on
p3 = plot3(sim.X_un(:, 1), sim.X_un(:, 2), sim.X_un(:, 3), '-', 'LineWidth', 1, 'Color', [0.3, 0.75, 0.7]);
hold on
p4 = plot3(sim.X(:, 1), sim.X(:, 2), sim.X(:, 3), '-', 'LineWidth', 1, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2 2 -4 0 -0.1 7]);
set(gca, 'XTick', [-2 -1 0 1 2], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-4 -3 -2 -1 0], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'ZTick', [0 1.5 3 4.5 6], 'FontName', 'Times New Roman', 'Fontsize', 8);

zlabel('{\it{z}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
ylabel('{\it{y}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman', 'position', [-2.7, -2.5])
xlabel('{\it{x}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman', 'position', [-0.4, -4.8])

h = legend([p1 p3 p4], {'Target', 'Koopman-based MPC', 'Proposed'}, 'FontName', 'Times New Roman', 'Fontsize', 8);
set(h, ...
  'Position', [0.125667021465298 0.934853094444609 0.750715383786251 0.0336048872249918], ...
  'Orientation', 'horizontal', ...
  'NumColumns', 4, ...
  'FontSize', 8, ...
  'EdgeColor', 'none');
h.ItemTokenSize = [20, 100];
annotation('line', [0.218943660519762 0.27421620953937], [0.952259920634922 0.952259920634922], 'LineWidth', 1,'Color', [0.1, 0.1, 0.1])

box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5);
set(gca, 'xgrid', 'on');
set(gca, 'ygrid', 'on');
set(gca, 'zgrid', 'on');

%% The Path Tracking Comparison in the ogxgyg Plane
figure
set(gcf, 'unit', 'centimeters', 'position', [17.7, 5.45, 6.2, 4.5])
set(gca, 'Fontsize', 10, 'FontName', 'Times New Roman', 'position', [0.12, 0.16, 0.84, 0.83]);

plot(sim.Xt(:, 1), sim.Xt(:, 2), '-', 'LineWidth', 6, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(sim.Xt(:, 1), sim.Xt(:, 2), '-', 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(sim.X_un(:, 1), sim.X_un(:, 2), '-', 'LineWidth', 1, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(sim.X(:, 1), sim.X(:, 2), '-', 'LineWidth', 1, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2 2 -3.2 0.2]);
set(gca, 'XTick', [-2 -1 0 1 2], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-3 -2 -1 0], 'FontName', 'Times New Roman', 'Fontsize', 8);
ylabel('{\it{y}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
xlabel('{\it{x}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')

box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5);
set(gca, 'xgrid', 'on');
set(gca, 'ygrid', 'on');
set(gca, 'zgrid', 'on');

%% The Path Tracking Comparison in the ogxgzg Plane
figure
set(gcf, 'unit', 'centimeters', 'position', [24.1, 5.45, 6.2, 4.5])
set(gca, 'Fontsize', 10, 'FontName', 'Times New Roman', 'position', [0.13, 0.16, 0.84, 0.83]);

plot(sim.Xt(:, 1), sim.Xt(:, 3), '-', 'LineWidth', 6, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(sim.Xt(:, 1), sim.Xt(:, 3), '-', 'LineWidth', 1, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(sim.X_un(:, 1), sim.X_un(:, 3), '-', 'LineWidth', 1, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(sim.X(:, 1), sim.X(:, 3), '-', 'LineWidth', 1, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2 2 -0.1 7]);
set(gca, 'XTick', [-2 -1 0 1 2], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [0 1.5 3 4.5 6], 'FontName', 'Times New Roman', 'Fontsize', 8);
ylabel('{\it{z}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')
xlabel('{\it{x}} (m)', 'Fontsize', 8, 'FontName', 'Times New Roman')

box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5);
set(gca, 'xgrid', 'on');
set(gca, 'ygrid', 'on');
set(gca, 'zgrid', 'on');

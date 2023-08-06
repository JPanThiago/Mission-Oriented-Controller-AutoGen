close all;
clear;
set(gcf,'unit', 'centimeters','Position', [17, 2, 17, 5])
addpath(genpath('.\control_test'));

[softrobot, softrobot_loss_avg_un] = control_SoftRobot_com();

%% Plot SoftRobot
subplot(1,3,1)
p1 = plot([softrobot.pacman.Xt(:, 1); softrobot.pacman.Xt(:, 1)], [softrobot.pacman.Xt(:, 2); softrobot.pacman.Xt(:, 2)], '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.72, 0.72, 0.72]);
hold on
p2 = plot([softrobot.pacman.Xt(:, 1); softrobot.pacman.Xt(:, 1)], [softrobot.pacman.Xt(:, 2); softrobot.pacman.Xt(:, 2)], '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.15, 0.15, 0.15]);
hold on
p3 = plot(softrobot.pacman.X_un(:, 1), softrobot.pacman.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on

axis([-2.7 4.7 -2.7 4.7]);
ylabel('{\it{y}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.05, 0.165, 0.27, 0.72]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(1,3,2)
plot([softrobot.star.Xt(:, 1); softrobot.star.Xt(:, 1)], [softrobot.star.Xt(:, 2); softrobot.star.Xt(:, 2)], '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.72, 0.72, 0.72]);
hold on
plot([softrobot.star.Xt(:, 1); softrobot.star.Xt(:, 1)], [softrobot.star.Xt(:, 2); softrobot.star.Xt(:, 2)], '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.15, 0.15, 0.15]);
hold on
plot(softrobot.star.X_un(:, 1), softrobot.star.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(softrobot.star.X_noise(:, 1), softrobot.star.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(softrobot.star.X(:, 1), softrobot.star.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2.6 7.2 -4.7 4.7]);
set(gca,'XTick',[-2 0 2 4 6], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-4 -2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
ylabel('{\it{y}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.385, 0.165, 0.27, 0.72]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(1,3,3)
plot([softrobot.blockM.Xt(:, 1); softrobot.blockM.Xt(:, 1)], [softrobot.blockM.Xt(:, 2); softrobot.blockM.Xt(:, 2)], '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.72, 0.72, 0.72]);
hold on
plot([softrobot.blockM.Xt(:, 1); softrobot.blockM.Xt(:, 1)], [softrobot.blockM.Xt(:, 2); softrobot.blockM.Xt(:, 2)], '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.15, 0.15, 0.15]);
hold on
plot(softrobot.blockM.X_un(:, 1), softrobot.blockM.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(softrobot.blockM.X_noise(:, 1), softrobot.blockM.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(softrobot.blockM.X(:, 1), softrobot.blockM.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2.7 4.7 -3.6 4.1]);
ylabel('{\it{y}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca, 'XTick', [-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.72, 0.165, 0.27, 0.72]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

h = legend([p1 p3], {'Target', 'Koopman-based MPC'}, 'FontName', 'Times New Roman', 'Fontsize', 8);
set(h, ...
  'Position', [0.1525667021465298 0.934853094444609 0.750715383786251 0.0336048872249918], ...
  'Orientation', 'horizontal', ...
  'NumColumns', 4, ...
  'FontSize', 8, ...
  'EdgeColor', 'none');
h.ItemTokenSize = [20, 100];
annotation('line', [0.380155931372549 0.42108774509804], [0.950048015873017 0.950048015873017], 'LineWidth', 1, 'Color', [0.15, 0.15, 0.15])


%---------------------------------------------------------------------------------------------%
%--- The failure case that introduces Gaussian noise to simulate environmental uncertainty ---%
%---------------------------------------------------------------------------------------------%
close all;
clear;
set(gcf,'unit', 'centimeters','Position', [17 , 2, 17, 21])
addpath(genpath('.\control_test'));

underopt = 'No';
datanoise = 'No';
noise = 'Yes';
failurecase = 'Yes';

[DP, DP_loss_avg, DP_loss_avg_un, DP_loss_avg_noise] = control_DP(27, underopt, datanoise, noise, failurecase);
[TDVPT, TDVPT_loss_avg, TDVPT_loss_avg_un, TDVPT_loss_avg_noise] = control_TDVPT(3, 27, underopt, datanoise, noise, failurecase);
[softrobot, softrobot_loss_avg, softrobot_loss_avg_un, softrobot_loss_avg_noise] = control_SoftRobot(27, underopt, datanoise, noise, failurecase);
[AUV, AUV_loss_avg, AUV_loss_avg_un, AUV_loss_avg_noise] = control_AUV(27, underopt, datanoise, noise, failurecase);

%% Plot DP
subplot(4,3,1)
p1 = plot(DP.t1.Xt(:, 2) * 180 / pi, DP.t1.Xt(:, 1) * 180 / pi, '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
p2 = plot(DP.t1.Xt(:, 2) * 180 / pi, DP.t1.Xt(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
p3 = plot(DP.t1.X_un(:, 2) * 180 / pi, DP.t1.X_un(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.6]);
hold on
p5 = plot(DP.t1.X_noise(:, 2) * 180 / pi, DP.t1.X_noise(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
p4 = plot(DP.t1.X(:, 2) * 180 / pi, DP.t1.X(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-23.5 23.5 -27 3]);
text('FontName', 'Times New Roman', 'Fontsize', 8, 'String', '.', 'Position', [-2.86263634569905 -29.6772725841493 -1.4210854715202e-14]);
ylabel('\theta (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('\theta (°/s)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-20 -10 0 10 20], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-25 -20 -15 -10 -5 0], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.05, 0.79, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

annotation('textbox', ...
  [0.500469977874419 0.7404 0.03 0.01],'String','(a)', ...
  'FontSize', 8, ...
  'FontName', 'Times New Roman', ...
  'FitBoxToText', 'off', ...
  'EdgeColor', 'none');
annotation('textbox', ...
  [0.500469977874419 0.497 0.03 0.01],'String','(b)', ...
  'FontSize', 8, ...
  'FontName', 'Times New Roman', ...
  'FitBoxToText', 'off', ...
  'EdgeColor', 'none');
annotation('textbox', ...
  [0.500469977874419 0.2537 0.03 0.01],'String','(c)', ...
  'FontSize', 8, ...
  'FontName', 'Times New Roman', ...
  'FitBoxToText', 'off', ...
  'EdgeColor', 'none');
annotation('textbox', ...
  [0.500469977874419 0.0104 0.03 0.01],'String','(d)', ...
  'FontSize', 8, ...
  'FontName', 'Times New Roman', ...
  'FitBoxToText', 'off', ...
  'EdgeColor', 'none');

h = legend([p1 p3 p4 p5], {'Target', 'Koopman-based MPC', 'Proposed', 'Proposed under noise'}, 'FontName', 'Times New Roman', 'Fontsize', 8);
set(h, ...
  'Position', [0.125667021465298 0.964853094444609 0.750715383786251 0.0336048872249918], ...
  'Orientation', 'horizontal', ...
  'NumColumns', 4, ...
  'FontSize', 8, ...
  'EdgeColor', 'none');
h.ItemTokenSize = [20,100];
annotation('line', [0.191510294117647 0.232782843137255], [0.981498015873017, 0.981498015873017], 'LineWidth',1,'Color', [0.1, 0.1, 0.1])

subplot(4,3,2)

plot(DP.t2.Xt(:, 2) * 180 / pi, DP.t2.Xt(:, 1) * 180 / pi, '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(DP.t2.Xt(:, 2) * 180 / pi, DP.t2.Xt(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(DP.t2.X_un(:, 2) * 180 / pi, DP.t2.X_un(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(DP.t2.X_noise(:, 2) * 180 / pi, DP.t2.X_noise(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(DP.t2.X(:, 2) * 180 / pi, DP.t2.X(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-45 45 -46 4]);
set(gca, 'XTick', [-40 -20 0 20 40], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'YTick', [-40 -30 -20 -10 0], 'FontName', 'Times New Roman', 'Fontsize', 8);
ylabel('\theta (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('\theta (°/s)', 'FontName', 'Times New Roman', 'Fontsize', 8)
text('FontName', 'Times New Roman', 'Fontsize', 8, 'String', '.', 'Position', [-5.58108014973365 -50.4391773460541 -1.4210854715202e-14]);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.385, 0.79, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,3)

plot(DP.t3.Xt(:, 2) * 180 / pi, DP.t3.Xt(:, 1) * 180 / pi, '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(DP.t3.Xt(:, 2) * 180 / pi, DP.t3.Xt(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(DP.t3.X_un(:, 2) * 180 / pi, DP.t3.X_un(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(DP.t3.X_noise(:, 2) * 180 / pi, DP.t3.X_noise(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(DP.t3.X(:, 2) * 180 / pi, DP.t3.X(:, 1) * 180 / pi, '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-29.5 29.5 -42.5 4.5]);
ylabel('\theta (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('\theta (°/s)', 'FontName', 'Times New Roman', 'Fontsize', 8)
text('FontName', 'Times New Roman', 'Fontsize', 8, 'String', '.', 'Position', [-3.68482654742815 -46.6516315585082 -1.4210854715202e-14]);
set(gca,'XTick',[-20 -10 0 10 20 30], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-40 -30 -20 -10 0], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.72, 0.79, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

%% Plot TDVPT
subplot(4,3,4)
plot(TDVPT.t1.Xt(:, 1), TDVPT.t1.Xt(:, 2), '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(TDVPT.t1.Xt(:, 1), TDVPT.t1.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(TDVPT.t1.X_un(:, 1), TDVPT.t1.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(TDVPT.t1.X_noise(:, 1), TDVPT.t1.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(TDVPT.t1.X(:, 1), TDVPT.t1.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-23 6 -16 16]);
ylabel('\theta (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('\psi (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-20 -15 -10 -5 0 5], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-15 -10 -5 0 5 10 15], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.05, 0.5466, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,5)
plot(TDVPT.t2.Xt(:, 1), TDVPT.t2.Xt(:, 2), '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(TDVPT.t2.Xt(:, 1), TDVPT.t2.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(TDVPT.t2.X_un(:, 1), TDVPT.t2.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(TDVPT.t2.X_noise(:, 1), TDVPT.t2.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(TDVPT.t2.X(:, 1), TDVPT.t2.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-25 21 -23 23]);
set(gca,'XTick',[-20 -10 0 10 20], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-20 -10 0 10 20], 'FontName', 'Times New Roman', 'Fontsize', 8);
ylabel('\theta (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('\psi (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.385, 0.5466, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,6)
plot(TDVPT.t3.Xt(:, 1), TDVPT.t3.Xt(:, 2), '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(TDVPT.t3.Xt(:, 1), TDVPT.t3.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(TDVPT.t3.X_un(:, 1), TDVPT.t3.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(TDVPT.t3.X_noise(:, 1), TDVPT.t3.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(TDVPT.t3.X(:, 1), TDVPT.t3.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-28 21 -25 25]);
ylabel('\theta (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('\psi (°)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-20 -10 0 10 20], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-20 -10 0 10 20], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.72, 0.5466, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

%% Plot SoftRobot
subplot(4,3,7)
plot([softrobot.pacman.Xt(:, 1); softrobot.pacman.Xt(:, 1)], [softrobot.pacman.Xt(:, 2); softrobot.pacman.Xt(:, 2)], '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot([softrobot.pacman.Xt(:, 1); softrobot.pacman.Xt(:, 1)], [softrobot.pacman.Xt(:, 2); softrobot.pacman.Xt(:, 2)], '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(softrobot.pacman.X_un(:, 1), softrobot.pacman.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(softrobot.pacman.X_noise(:, 1), softrobot.pacman.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(softrobot.pacman.X(:, 1), softrobot.pacman.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2.7 4.7 -2.7 4.7]);
ylabel('{\it{y}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (cm)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.05, 0.3033, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,8)
plot([softrobot.star.Xt(:, 1); softrobot.star.Xt(:, 1)], [softrobot.star.Xt(:, 2); softrobot.star.Xt(:, 2)], '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot([softrobot.star.Xt(:, 1); softrobot.star.Xt(:, 1)], [softrobot.star.Xt(:, 2); softrobot.star.Xt(:, 2)], '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
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
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.385, 0.3033, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,9)
plot([softrobot.blockM.Xt(:, 1); softrobot.blockM.Xt(:, 1)], [softrobot.blockM.Xt(:, 2); softrobot.blockM.Xt(:, 2)], '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot([softrobot.blockM.Xt(:, 1); softrobot.blockM.Xt(:, 1)], [softrobot.blockM.Xt(:, 2); softrobot.blockM.Xt(:, 2)], '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
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
set(gca,'XTick',[-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-2 0 2 4], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.72, 0.3033, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

%% Plot AUV
subplot(4,3,10)
plot(AUV.t1.Xt(:, 1), AUV.t1.Xt(:, 2), '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(AUV.t1.Xt(:, 1), AUV.t1.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(AUV.t1.X_un(:, 1),AUV.t1.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(AUV.t1.X_noise(:, 1),AUV.t1.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(AUV.t1.X(:, 1), AUV.t1.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-1.25 1.25 -2.25 0.25]);
ylabel('{\it{y}} (m)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (m)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-1 -0.5 0 0.5 1], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-2 -1.5 -1 -0.5 0], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.05, 0.06, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,11)
plot(AUV.t2.Xt(:, 1), AUV.t2.Xt(:, 2), '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(AUV.t2.Xt(:, 1), AUV.t2.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(AUV.t2.X_un(:, 1), AUV.t2.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(AUV.t2.X_noise(:, 1), AUV.t2.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(AUV.t2.X(:, 1), AUV.t2.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-3.5 3.5 -1.65 1.65]);
set(gca,'XTick',[-3 -2 -1 0 1 2 3], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-1.5 -1 -0.5 0 0.5 1 1.5], 'FontName', 'Times New Roman', 'Fontsize', 8);
ylabel('{\it{y}} (m)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (m)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.385, 0.06, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

subplot(4,3,12)
plot(AUV.t3.Xt(:, 1), AUV.t3.Xt(:, 2), '-', 'LineWidth', 6, 'MarkerSize', 4, 'Color', [0.8, 0.8, 0.8]);
hold on
plot(AUV.t3.Xt(:, 1), AUV.t3.Xt(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.1, 0.1, 0.1]);
hold on
plot(AUV.t3.X_un(:, 1), AUV.t3.X_un(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.3, 0.75, 0.7]);
hold on
plot(AUV.t3.X_noise(:, 1), AUV.t3.X_noise(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.66, 0.60, 0.65]);
hold on
plot(AUV.t3.X(:, 1), AUV.t3.X(:, 2), '-', 'LineWidth', 1, 'MarkerSize', 4, 'Color', [0.94, 0.5, 0.45]);
hold on

axis([-2.1 3.5 -2.85 2.85]);
ylabel('{\it{y}} (m)', 'FontName', 'Times New Roman', 'Fontsize', 8)
xlabel('{\it{x}} (m)', 'FontName', 'Times New Roman', 'Fontsize', 8)
set(gca,'XTick',[-2 -1 0 1 2 3], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca,'YTick',[-2 -1 0 1 2], 'FontName', 'Times New Roman', 'Fontsize', 8);
set(gca, 'FontName', 'Times New Roman', 'Fontsize', 8, 'Position', [0.72, 0.06, 0.27, 0.1714]);
box on;
set(gca, 'gridlinestyle', '-', 'linewidth', 0.5)
set(gca, 'xgrid', 'on')
set(gca, 'ygrid', 'on')

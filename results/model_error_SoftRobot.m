yy = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3];
xx = [1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5];
sz = [0.4, 0.2, 0.21, 0.2, 0.2, 0.4, 0.2, 0.2, 1, 0.8, 0.4, 0.19, 0.2, 0.6, 0.4, 0.21, 0.18, 0.44, 0.43, 0.2];
ssz = [0.0413, 0.0369, 0.0375, 0.0367, 0.0364, 0.0405, 0.0365, 0.0366, 0.7206, 0.2782,...
    0.0399, 0.0359, 0.0364, 0.0916, 0.0407, 0.0379, 0.0344, 0.0483, 0.0451, 0.0364];
szz = num2str(ssz', '%.4f\n');
sz =  150 * sz;

figureUnits = 'centimeters';
fig = figure;
set(gcf, 'unit', 'centimeters', 'position', [8, 6, 8, 3])
set(gca, 'Fontsize', 8, 'FontName', 'Times New Roman', 'position', [0.07, 0.25, 0.92, 0.7]);
hold on

map = [255 186 163
       255 154 118
       249 109 128
       220 100 110
       187  89 107]/255;
colormap(map);

scatter(xx, yy, sz, sz, 'filled', 'MarkerFaceAlpha', 0.7)
hXLabel = xlabel('Degree');
hYLabel = ylabel('Delay');
text(xx + 0.16, yy , cellstr(szz), 'FontSize', 7, 'FontName', 'Times New Roman');

set(gca, 'Box', 'on', ...                                        
         'XGrid', 'off', 'YGrid', 'off', ...                     
         'TickDir', 'in', 'TickLength', [.01 .01], ...           
         'XMinorTick', 'off', 'YMinorTick', 'off', ...           
         'XColor', [.1 .1 .1],  'YColor', [.1 .1 .1],...         
         'XTick', 1 : 1 : 5,...                                     
         'XLim', [0.7 5.7],...
         'YTick', 0 : 1 : 3,...
         'YLim', [-0.8 3.8])

set(gca, 'FontName', 'Times New Roman')
set([hXLabel, hYLabel], 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8, 'FontName', 'Times New Roman')
set([hXLabel, hYLabel], 'FontSize', 8, 'FontName', 'Times New Roman')
set(gcf,'Color',[1 1 1])
%------------------------------------------------------------------------------%
%--- Supplementary results for the Koopman-based modeling errors of the AUV ---%
%------------------------------------------------------------------------------%
yy = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3];
xx = [1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 3, 4, 1, 2, 3];
sz = [0.45, 0.3, 0.2, 0.2, 0.2, 0.35, 0.3, 1, 0.3, 0.4, 0.3, 0.8, 0.25, 0.3, 0.6, 0.5];
ssz = [0.5883, 0.4682, 0.3589, 0.3793, 0.3873, 0.4885, 0.4400, 8.5506, 0.4657, 0.5208,...
    0.4582, 4.1751, 0.4175, 0.4592, 1.7149, 1.2752];
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

scatter(xx, yy, sz, sz, 'filled', 'MarkerFaceAlpha',0.7)
hXLabel = xlabel('Degree');
hYLabel = ylabel('Delay');
text(xx + 0.16, yy , cellstr(szz), 'FontSize', 7,'FontName','Times New Roman');

scatter(2, 2, 134, 'x', 'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', [0.8500 0.3250 0.0980])
scatter(5, 2, 58, '_', 'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', [0.8500 0.3250 0.0980])
scatter(5, 3, 58, '_', 'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', [0.8500 0.3250 0.0980])
scatter(4, 3, 58, '_', 'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', [0.8500 0.3250 0.0980])

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

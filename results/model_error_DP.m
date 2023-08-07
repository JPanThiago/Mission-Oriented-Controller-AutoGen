%-------------------------------------------------------------------------------------------%
%--- Supplementary results for the Koopman-based modeling errors of the damping pendulum ---%
%-------------------------------------------------------------------------------------------%
yy = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3];
xx = [1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5];
sz = [0.4, 0.4, 0.3, 0.3, 0.3, 0.4, 0.4, 0.3, 1, 0.6, 0.6, 0.6, 0.8, 0.2, 0.2, 0.8, 0.5, 0.25, 0.2, 0.2];
ssz = [0.0120, 0.0123, 0.0104, 0.0103, 0.0098, 0.0117, 0.0117, 0.0089, 0.3482, 0.0549,...
    0.0538, 0.0530, 0.0995,  0.0042, 0.0040, 0.0992, 0.0173, 0.0063, 0.0043, 0.0040];
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
text(xx + 0.16, yy, cellstr(szz), 'FontSize', 7, 'FontName', 'Times New Roman');

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

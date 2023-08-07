%---------------------------------------------------------------------%
%--- Supplementary results for the control error of the soft robot ---%
%---------------------------------------------------------------------%
yy = [0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3];
xx = [1, 2, 3, 4, 5, 1, 2, 3, 4, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5];
sz =[0.62, 0.8, 0.81, 0.82, 1, 0.42, 0.6, 0.6, 0.83, 0.4, 0.42, 0.41, 0.42, 0.61, 0.2, 0.23, 0.21, 0.22, 0.4];
ssz = [0.6935, 0.7082, 0.7192, 0.7242, 0.7421, 0.6382, 0.6596, 0.6721, 0.7430, 0.6018, 0.6354, 0.6232,...
    0.6368, 0.6812, 0.5071, 0.5447, 0.5323, 0.533, 0.6000];
szz = num2str(ssz', '%.4f\n');
sz =  150 * sz;
yy1 = 1;
xx1 = 5;
sz1 = 300 * 0.446303279;
ssz1 = 0.446;
szz1 = num2str(ssz1');

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
aa = num2str([1; 2],'%.4f\n');

scatter(xx, yy, sz, sz, 'filled', 'MarkerFaceAlpha', 0.7)
hXLabel = xlabel('Degree');
hYLabel = ylabel('Delay');
scatter(xx1, yy1, sz1, 'x', 'MarkerFaceAlpha', 0.7)
text(xx + 0.16, yy , szz, 'FontSize', 7, 'FontName', 'Times New Roman');

set(gca, 'Box', 'on', ...                                        
         'XGrid', 'off', 'YGrid', 'off', ...                     
         'XMinorTick', 'off', 'YMinorTick', 'off', ...          
         'XTick', 1 : 1 : 5,...                                    
         'XLim', [0.7 5.7],...
         'YTick', 0 : 1: 3,...
         'YLim', [-0.8 3.8])

set(gca, 'FontName', 'Times New Roman')
set([hXLabel, hYLabel], 'FontName', 'Times New Roman')
set(gca, 'FontSize', 8, 'FontName', 'Times New Roman')
set([hXLabel, hYLabel], 'FontSize', 8, 'FontName', 'Times New Roman')
set(gcf, 'Color', [1 1 1])

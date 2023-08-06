yy = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3];
xx = [1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5];
sz = [0.7, 0.7, 0.65, 0.65, 0.7, 0.65, 0.6, 0.45, 0.45, 0.5, 0.5, 0.5, 0.3, 0.3, 1, 0.5, 0.5, 0.2, 0.3, 1];
ssz = [0.0825, 0.0823, 0.0807, 0.0807, 0.0827, 0.0809, 0.0806, 0.0741, 0.0739, 0.0794,...
    0.0789, 0.0784, 0.0693, 0.0692, 0.3710, 0.0787, 0.0782, 0.0662, 0.0697, 0.3543];
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
       187 89  107]/255;
colormap(map);

scatter(xx, yy, sz, sz, 'filled', 'MarkerFaceAlpha',0.7)
hXLabel = xlabel('Degree');
hYLabel = ylabel('Delay');
text(xx + 0.16, yy , cellstr(szz), 'FontSize', 7,'FontName','Times New Roman');

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
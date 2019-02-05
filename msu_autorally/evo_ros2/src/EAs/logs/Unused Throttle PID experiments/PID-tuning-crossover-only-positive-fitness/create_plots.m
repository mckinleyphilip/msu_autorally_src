%%
%
%
% GAS 2018-12-04

%% Parallel Coordinate Plot
figure

mat = comp_table_2{1:height(comp_table_2), { 'KP', 'KI', 'KD', 'IMax'}};
run_numbers = comp_table_2{1:height(comp_table_2), {'Run_Number'}};
labels = { 'KP', 'KI', 'KD', 'IMax'};

parallelcoords(mat,...
	'group', run_numbers,... 
	'labels', labels)

title_string = strcat(plot_sets{x}, ' - Parallel Coordinate Plot of Best PID Settings');
title(title_string)

file_name = strcat('plots/', title_string, '.png');
saveas(gcf, file_name)
close all



%% Stem Plot
figure

for i=1:max(comp_table_2.Run_Number)
	rows = comp_table_2.Run_Number==i;
	t = comp_table_2(rows, :);
	
	stem3(t.KD, t.KI, t.KP)
	hold on
end

%xlim([0,1])
%ylim([0,1])
%zlim([0,1])
xlabel('KD')
ylabel('KI')
zlabel('KP')
head = unique(comp_table_2(:, 1));
head2 = string(table2cell(head));
stem_legend = legend(head2,'Position', [0.82, 0.5 0.15 0.2]);
title(stem_legend, "Run Number:", 'FontSize',8);

title_string = strcat(plot_sets{x}, ' - Stem Plot of Best PID Settings');
title(title_string)

file_name = strcat('plots/', title_string, '.png');
saveas(gcf, file_name)
close all


%% Breakout plot
figure
for i=1:max(comp_table_2.Run_Number)
	rows = comp_table_2.Run_Number==i;
	t = comp_table_2(rows, :);
	
	plot(t.IMax, 0.*t.IMax+1, '*','MarkerSize',10)
	plot(t.KI, 0.*t.KI+2, '*','MarkerSize',10)
	plot(t.KD, 0.*t.KD+3, '*','MarkerSize',10)
	plot(t.KP, 0.*t.KP+4, '*','MarkerSize',10)
	
	hold on
end
ylim([0,5])
xbounds = xlim;
textx = (xbounds(1) + xbounds(2)) / 2;
text(textx, 1.3, 'IMax')
text(textx, 2.3, 'KI')
text(textx, 3.3, 'KD')
text(textx, 4.3, 'KP')
%plot_legend = legend(head2,'Position', [0.01, 0.4 0.15 0.2]);
plot_legend = legend(head2,'Location', 'Best');
title(plot_legend, "Run Number:", 'FontSize',8);

title_string = strcat(plot_sets{x}, ' - Breakout Plot of PID Settings');
title(title_string)

file_name = strcat('plots/', title_string, '.png');
saveas(gcf, file_name)
close all



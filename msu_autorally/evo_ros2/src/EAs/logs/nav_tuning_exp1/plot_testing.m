%%
%
%
% GAS 2018-11-19



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
title('Stem Plot of Best PID Settings')



%% KI vs IMAX
figure

for i=1:max(comp_table_2.Run_Number)
	rows = comp_table_2.Run_Number==i;
	t = comp_table_2(rows, :);
	
	scatter(t.IMax, t.KI)
	hold on
end

%xlim([0,1])
%ylim([0,1])
%zlim([0,1])
xlabel('IMax')
ylabel('KI')
lgd = legend(head2,'Position', [0.82, 0.5 0.15 0.2]);
title(lgd, "Run Number:", 'FontSize',8);
title('KI vs IMax')



%% 1D plot
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
title('IMax Settings')



%% Error Plot
figure

bar_mean = zeros(max(comp_table_2.Run_Number), 4);
bar_error = zeros(max(comp_table_2.Run_Number), 4);
for i=1:max(comp_table_2.Run_Number)
	rows = comp_table_2.Run_Number==i;
	t = comp_table_2(rows, :);
	
	run_mean = [mean(t.IMax), mean(t.KD), mean(t.KI), mean(t.KP)];
	run_error = [std(t.IMax), std(t.KD), std(t.KI), std(t.KP)];
	
	bar_mean(i, :) = run_mean;
	bar_error(i, :) = run_error;
end
bar(bar_mean)
hold on
%errorbar(bar_mean, bar_error, '.')
xlabel('Run Number')
ylabel('Value')
bar_legend = legend({'Imax', 'KD', 'KI', 'KP'}, 'Location', 'Best');
title(bar_legend, "Run Number:", 'FontSize',8);
title('Average PID Settings Over Runs')


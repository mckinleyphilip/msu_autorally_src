%%
%
%
% GAS 2018-11-05


t = comp_table(:,3:end)
t2 = comp_table_2(:,3:end)


% Stem Plot
figure
stem3(t.KD, t.KI, t.KP)
hold on
stem3(t2.KD, t2.KI, t2.KP)
%xlim([0,1])
%ylim([0,1])
%zlim([0,1])
xlabel('KD')
ylabel('KI')
zlabel('KP')
legend(['Incline', 'Flat'])
legend('Incline', 'Flat','location','NorthEast')
title('Stem Plot of Best PID Settings')


% Error Plot
figure
t_mean = [mean(t.IMax), mean(t.KD), mean(t.KI), mean(t.KP)];
t2_mean = [mean(t2.IMax), mean(t2.KD), mean(t2.KI), mean(t2.KP)];
bar_mean = [t_mean(1) t2_mean(1); t_mean(2) t2_mean(2); t_mean(3) t2_mean(3); t_mean(4) t2_mean(4)];
t_std = [std(t.IMax), std(t.KD), std(t.KI), std(t.KP)];
t2_std = [std(t2.IMax), std(t2.KD), std(t2.KI), std(t2.KP)];
bar_std = [t_std(1) t2_std(1); t_std(2) t2_std(2); t_std(3) t2_std(3); t_std(4) t2_std(4)];
bar(bar_mean)
hold on
errorbar(bar_mean, bar_std, '.')


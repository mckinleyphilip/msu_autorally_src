%% Import best individual from each run
%
%
% GAS 11-5-17

clear all;
number_of_runs = 10;

headers = {'Run_Number', 'Best_Fitness', 'IMax', 'KD', 'KI', 'KP'};
comp_table = cell2table(cell(0,6));
comp_table.Properties.VariableNames = headers;

run_directories = dir();

for i=1:length(run_directories)
	if run_directories(i).isdir
		dir_name = run_directories(i).name;
		if contains(dir_name, 'run')
			filename = strcat(dir_name, '/','log.json');
			table = jsondecode(fileread(filename));
			IMax = table.best_ind(1);
			KD = table.best_ind(2);
			KI = table.best_ind(3);
			KP = table.best_ind(4);
			comp_table = [comp_table; {table.run_number, table.best_ind_fitness, IMax, KD, KI, KP}];
		end
	end
end




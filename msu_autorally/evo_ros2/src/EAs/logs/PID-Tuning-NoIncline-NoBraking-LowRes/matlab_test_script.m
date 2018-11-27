%% Import best individual from each run
%
%
% GAS 11-5-17

clear all;

headers = {'Run_Number', 'Best_Fitness', 'IMax', 'KD', 'KI', 'KP'};
comp_table_2 = cell2table(cell(0,6));
comp_table_2.Properties.VariableNames = headers;

run_directories = dir();

for i=1:length(run_directories)
	if run_directories(i).isdir
		dir_name = run_directories(i).name;
		if contains(dir_name, 'run')
			filename = strcat(dir_name, '/','log.json');
			table = jsondecode(fileread(filename));
			
			for j=1%:length(table.hall_of_fame)
				IMax = table.hall_of_fame(j, 1);
				KD = table.hall_of_fame(j, 2);
				KI = table.hall_of_fame(j, 3);
				KP = table.hall_of_fame(j, 4);
				fitness = table.hall_of_fame_fitnesses(j);
				
				comp_table_2 = [comp_table_2; {table.run_number, fitness, IMax, KD, KI, KP}];
			end
		end
	end
end



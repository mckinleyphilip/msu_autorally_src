%% Import best individual from each run
%
%
% GAS 2018-12-04

clear all;

plot_sets = {'Best Inds Overall', 'Best in Last Gen', 'Hall of Fame'};

headers = {'Run_Number', 'Best_Fitness', 'IMax', 'KD', 'KI', 'KP'};
comp_table_2 = cell2table(cell(0,6));
comp_table_2.Properties.VariableNames = headers;

run_directories = dir();
mkdir plots


for x=1:length(plot_sets)
	for i=1:length(run_directories)
		if run_directories(i).isdir
			dir_name = run_directories(i).name;
			if contains(dir_name, 'run')
				if x==1 || x==3
					filename = strcat(dir_name, '/','log.json');
				else
					filename = strcat(dir_name, '/','best_from_last_gen.json');
				end
				
				table = jsondecode(fileread(filename));
				
				if x==3
					num_from_each_run = length(table.hall_of_fame);
				else
					num_from_each_run = 1;
				end
				
				for j=1:num_from_each_run
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
	create_plots
end




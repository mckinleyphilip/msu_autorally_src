%% Import best individual from each run
%
%
% GAS 11-5-17

clear all;

%%
headers = {'run_number', 'fitness', 'global_inflation_radius', 'local_inflation_radius',...
	'min_turning_radius', 'weight_max_vel_x', 'weight_max_vel_theta',...
	'weight_acc_lim_x', 'weight_kinematics_turning_radius', 'weight_obstacle',...
	'max_vel_theta', 'acc_lim_x', 'acc_lim_theta'};


inds = cell2table(cell(0,length(headers)));
inds.Properties.VariableNames = headers;

%%
run_directories = dir();

for i=1:length(run_directories)
	if run_directories(i).isdir
		dir_name = run_directories(i).name;
		if contains(dir_name, 'run')
			filename = strcat(dir_name, '/','log.json');
			table = jsondecode(fileread(filename));
			
			% Iterate over individuals in hall of fame
			for j=1:length(table.hall_of_fame(:, 1))
				
				% Add row in inds table
				inds(j, :) = [repmat({0},1,width(inds))];
				
				% write run number
				inds(j,1) = {table.run_number};
				
				% write fitness
				inds(j,2) = {table.hall_of_fame_fitnesses(j)};
				
				% Iterate over individual's genome
				for h=1:length(table.hall_of_fame(j, :))
					inds(j, h+2) = {table.hall_of_fame(j, h)};
				end
	
			end
		end
	end
end



%filename = '~/simulation/auto_rally_catkin_ws/src/autorally/autorally_description/urdf/autoRallyTrack.stl';
%coord = stlread(filename);

%pts = coord.vertices;

%plot(pts(:,1),pts(:,2))


% plot course taken
log = readtable('../../run_2/out.csv');
plot(log.PosX, log.PosY)

% plot obstacles
models = readtable('../models.csv');
hold on;
scatter(str2double(models.PosX), str2double(models.PosY))



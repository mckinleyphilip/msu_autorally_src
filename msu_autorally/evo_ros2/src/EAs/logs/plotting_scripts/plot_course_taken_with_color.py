import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import pprint

from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

run_path = '../nav_tuning_exp1/run1/'
file_name = 'best_ind_details.csv'
fig_file_name = 'course_taken.png'


# Prepare track boundary data
filename = 'gazebo_2018-10-18.npz'
trackBoundaries = np.load(filename)
  
# Prepare obstacle placement data
obstacles_df = pd.read_csv('models.csv')

# Prepare obstacle placement data
course_df = pd.read_csv(run_path + file_name)
df = course_df

#pp = pprint.PrettyPrinter(indent=4)
#pp.pprint(course_df)


fig, ax1 = plt.subplots()

# Plot track
ax1.plot((trackBoundaries['X_in'][:]+8.7)*-3.2, (trackBoundaries['Y_in'][:])*3.2,'-',color='black') # outer boundary
ax1.plot((trackBoundaries['X_out'][:]+8.7)*-2.8, (trackBoundaries['Y_out'][:])*2.8,'-',color='black') #inner boundary

dx = np.diff((trackBoundaries['X_in'][:]+8.7)*-3.2)
dy = np.diff((trackBoundaries['Y_in'][:])*3.2)
d = np.hypot(dx, dy)
d = np.insert(d,0,0)
total_inner_distance = np.sum(d)
print(total_inner_distance)

dx = np.diff((trackBoundaries['X_out'][:]+8.7)*-2.8)
dy = np.diff((trackBoundaries['Y_out'][:])*2.8)
d = np.hypot(dx, dy)
d = np.insert(d,0,0)
total_outter_distance = np.sum(d)
print(total_outter_distance)


# Plot obstacles
obstacles_df.plot(kind='scatter', x='PosX', y='PosY', color='red', ax=ax1)

# Plot driving course
#course_df.truncate(before=2).plot(kind='line', x='Pos X', y='Pos Y', color='blue', ax=ax1)

course_df = course_df.truncate(before=2)

x = course_df['Pos X'].values
y = course_df['Pos Y'].values
dydx = course_df['Actual Speed'].values

points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)



# Create a continuous norm to map from data points to colors
norm = plt.Normalize(-3, 7.0)
#norm = plt.Normalize(0, 8)
#lc = LineCollection(segments, cmap='viridis', norm=norm)
lc = LineCollection(segments, norm=norm)
# Set the values used for colormapping
lc.set_array(dydx)
lc.set_linewidth(2)
line = ax1.add_collection(lc)
fig.colorbar(line, ax=ax1)


ax1.grid(False)
ax1.set_xlabel('Position X (meters)')
ax1.set_ylabel('Position Y (meters)')
plt.title('Course and Speed of Autorally')
plt.text(55,0,'Speed (m/s)')




###
avg_speed = df['Actual Speed'].mean()
max_speed = df['Actual Speed'].max()
norm_avg_speed = avg_speed / 15
norm_max_speed = max_speed / 15

# Waypoints
waypoints_achieved = df['Goal Status'].max()
norm_wp = waypoints_achieved / 4.0

if norm_wp == 1.0:
    # Time
    time_elapsed = df['Time'].max()
    norm_time_elapsed =  15.2 / time_elapsed

    # Distance
    dx = np.diff(df['Pos X'])
    dy = np.diff(df['Pos Y'])
    d = np.hypot(dx, dy)
    d = np.insert(d,0,0)
    total_distance = np.sum(d)
    norm_distance = 152.67 / total_distance

else:
    # Time
    time_elapsed = df['Time'].max()
    norm_time_elapsed =  time_elapsed / 300.0

    # Distance
    dx = np.diff(df['Pos X'])
    dy = np.diff(df['Pos Y'])
    d = np.hypot(dx, dy)
    d = np.insert(d,0,0)
    total_distance = np.sum(d)
    norm_distance = (total_distance / 222) / 2

raw_fitness = [avg_speed, max_speed, waypoints_achieved, time_elapsed, total_distance]
norm_fitness = [norm_avg_speed * 2, norm_max_speed * 2, norm_wp * 3, norm_time_elapsed * 1, norm_distance * 1]
total_fitness = sum(norm_fitness)

print('Raw Fitness: {}'.format(raw_fitness))
print('Total Fitness: {}'.format(total_fitness))


plt.savefig(run_path + fig_file_name)
plt.show()



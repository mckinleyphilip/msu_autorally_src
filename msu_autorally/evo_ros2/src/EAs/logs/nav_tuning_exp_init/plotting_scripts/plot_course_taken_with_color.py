import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import pprint

from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm


# Prepare track boundary data
filename = 'gazebo_2018-10-18.npz'
trackBoundaries = np.load(filename)
  
# Prepare obstacle placement data
obstacles_df = pd.read_csv('models.csv')

# Prepare obstacle placement data
course_df = pd.read_csv('../run2/best_ind_details.csv')

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
norm = plt.Normalize(dydx.min(), dydx.max())
#norm = plt.Normalize(0, 8)
#lc = LineCollection(segments, cmap='viridis', norm=norm)
lc = LineCollection(segments, norm=norm)
# Set the values used for colormapping
lc.set_array(dydx)
lc.set_linewidth(2)
line = ax1.add_collection(lc)
fig.colorbar(line, ax=ax1)




plt.show()



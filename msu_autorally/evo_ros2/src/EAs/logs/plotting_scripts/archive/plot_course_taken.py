import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import pprint



# Prepare track boundary data
filename = 'gazebo_2018-10-18.npz'
trackBoundaries = np.load(filename)
  
# Prepare obstacle placement data
obstacles_df = pd.read_csv('models.csv')

# Prepare obstacle placement data
course_df = pd.read_csv('../run_2/out.csv')

#pp = pprint.PrettyPrinter(indent=4)
#pp.pprint(course_df)


fig, ax1 = plt.subplots()
# Plot track
ax1.plot((trackBoundaries['X_in'][:]+8.7)*-3.2, (trackBoundaries['Y_in'][:])*3.2,'-',color='black') # outer boundary
ax1.plot((trackBoundaries['X_out'][:]+8.7)*-2.8, (trackBoundaries['Y_out'][:])*2.8,'-',color='black') #inner boundary

# Plot obstacles
obstacles_df.plot(kind='scatter', x='PosX', y='PosY', color='red', ax=ax1)

# Plot driving course
course_df.truncate(before=2).plot(kind='line', x='Pos X', y='Pos Y', color='blue', ax=ax1)





#plt.gca().invert_yaxis()
plt.show()


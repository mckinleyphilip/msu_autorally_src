import sys

fname = '/home/simongle/simulation/auto_rally_catkin_ws/src/autorally/autorally_control/launch/waypoints'
out_fname = '/home/simongle/simulation/auto_rally_catkin_ws/src/autorally/autorally_control/launch/waypoints_scaled'


with open(fname) as f:
    content = f.readlines()
   
new_content = ""
for line in content:
	line = line.split(',')
	line[0] = str(float(line[0]) * 3)
	line[1] = str(float(line[1]) * 3)
	new_content += str(line[0]) + ',' + str(line[1]) + '\n'

content = "".join(new_content)

#print(content)
    
with open(out_fname, 'w') as o:
	o.write(content)


import xml.etree.ElementTree as ET
import csv

tree = ET.parse('/home/simongle/simulation/auto_rally_catkin_ws/src/autorally/autorally_description/urdf/obstacle_sky_world2.world')
root = tree.getroot()

models = list()
models.append(['ModelName', 'PosX', 'PosY', 'PosZ', 'OriX', 'OriY', 'OriZ']);

for model in root[0].findall('model'):
	#print model.tag, model.attrib
	name = model.get('name')
	pose = model.find('pose').text
	models.append([name]+pose.split())
	
print models



with open('models.csv', 'wb') as myfile:
    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
    wr.writerows(models)

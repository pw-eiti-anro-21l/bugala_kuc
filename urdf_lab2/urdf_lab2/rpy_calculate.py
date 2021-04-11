import csv
import json
from math import cos, sin, atan2, sqrt
import os
from matrix import matrix_multiplier
#import xml.etree.ElementTree as ET
from lxml import etree as ET
import transformations

def csv_reader(filename):
	dh = []
	with open(filename, 'r') as file:
		read_file = csv.reader(file, delimiter = ';')
		for row in read_file:
			dh.append(row)
	rows = len(dh)
	columns = len(dh[0])
	for i in range(1, rows):
		for j in range(1, columns-1):
			dh[i][j] = float(dh[i][j])
	dh.pop(0)
	return dh

def get_params(part):
    with open("parts_params.json", "r") as file:
        read_file = json.load(file)
    part_params = read_file[part]
    return part_params

# i a d alpha theta part
# 0 1 2 3     4     5
def find_rpy(filename):
	dh = csv_reader(filename)
	print(dh)
	rows = len(dh)
	columns = len(dh[0])
	rpy_table = []
	xyz_table = []
	file = open('urdf_params.yaml', 'w')
	for row in range(0, rows):
		rot_theta = transformations.rotation_matrix(dh[row][4], (0,0,1))
		rot_alpha = transformations.rotation_matrix(dh[row][3], (1,0,0))
		trans_a = transformations.translation_matrix((dh[row][1],0,0))
		trans_d = transformations.translation_matrix((0,0,dh[row][2]))
		h_matrix = trans_a @ rot_alpha @ trans_d @ rot_theta
		rpy_table.append(transformations.euler_from_matrix(h_matrix))
		xyz_table.append(transformations.translation_from_matrix(h_matrix))
	return rpy_table, xyz_table

		# h = [[cos(dh[row][4]), -sin(dh[row][4])*cos(dh[row][3]), sin(dh[row][4])*sin(dh[row][3]), dh[row][1]*cos(dh[row][4])],
		# 	   [sin(dh[row][4]), cos(dh[row][4])*cos(dh[row][3]), -cos(dh[row][4])*sin(dh[row][3]), dh[row][1]*sin(dh[row][4])],
		# 	   [0, sin(dh[row][3]), cos(dh[row][3]), dh[row][2]],
		# 	   [0, 0, 0, 1]]
		# [roll, pitch, yaw] = find_rpy(h)
		# print([roll, pitch, yaw])
		# file.write(dh[row][0] +":\n")
		# file.write(" joint_xyz: "+str(0)+" "+str(0)+" "+str(0)+"\n")
		# file.write(" joint_rpy: "+str(roll)+" "+str(pitch)+" "+str(yaw)+"\n")
		# file.write(" link_xyz: "+str(0)+" "+str(0)+" "+str(float(dh[row][2]*(-0.5)))+"\n")
		# file.write(" link_rpy: "+str(0)+" "+str(0)+" "+str(0)+"\n")
		# file.write(" link_len: "+str(dh[row][2])+"\n")


def link_xml_creator(link_name, length, rpy=[0, 0, 0], xyz=[0, 0, 0]):
	params = get_params(link_name)

	if link_name == "base" or link_name == "tool":
		xyz_z = str(float(0.5*float(length)))
	else:
		xyz_z = str(float(-0.5*float(length)))

	rpy = f'{rpy[0]} {rpy[1]} {rpy[2]}'
	xyz = f'{xyz[0]} {xyz[1]} {xyz_z}'

	link = ET.Element('link', name=params['link_name'])
	# inertial = ET.SubElement(link, 'inertial')
	# mass = ET.SubElement(inertial, 'mass', value = mass) 
	# inertial = ET.SubElement(inertial, 'inertia', ixx="100", ixy="0", ixz="0", iyy="100", iyz="0", izz="100")
	#origin = ET.SubElement(inertial, "origin")
	visual = ET.SubElement(link, "visual")
	visual_origin = ET.SubElement(visual, "origin", xyz=xyz, rpy=rpy)
	geometry = ET.SubElement(visual, "geometry")
	geometry_type = ET.SubElement(geometry, "box", size=params['size'])
	material = ET.SubElement(visual, "material", name=params['material'])
	color = ET.SubElement(material, "color", rgba=params['color'])
	# collision = ET.SubElement(link, "collision")
	# collision_origin = ET.SubElement(collision, "origin", xyz=xyz, rpy=rpy)
	# collision_geometry = ET.SubElement(collision, "geometry")
	# collision_cylinder = ET.SubElement(collision_geometry, "cylinder", radius=params['radius'], length=length)
	# contact_cofficients = ET.SubElement(collision, "contact_cofficients", mu="0", kp="1000.0", kd="1.0")
	# tree = ET.ElementTree(link)
	# tree.write('robot_lab2.urdf.xml', pretty_print=True)
	return link

# def joint_xml_creator(joint_name, parent_name, child_name, origin_rpy, axis = None, limit_att = None):
def joint_xml_creator(joint_name, rpy, xyz,):
	params = get_params(joint_name)
	rpy = f'{rpy[0]} {rpy[1]} {rpy[2]}'
	xyz = f'{xyz[0]} {xyz[1]} {xyz[2]}'
	joint = ET.Element('joint', name=params['joint_name'], type=params['joint_type'])
	parent = ET.SubElement(joint, 'parent', link=params['parent_name'])
	child = ET.SubElement(joint, 'child', link=params['child_name'])
	origin = ET.SubElement(joint, 'origin', xyz=xyz,rpy = rpy)
	if params['joint_type'] != 'fixed':
		axis = ET.SubElement(joint, 'axis', axis=params['axis'])
		limit = ET.SubElement(joint, 'limit', lower=params['lower_limit'], upper=params['upper_limit'], effort=params['effort'], velocity=params['velocity'])
	# tree = ET.ElementTree(joint)
	# tree.write('robot_lab2_joint.urdf.xml', pretty_print=True)
	return joint


def xacro_prop_xml():
	prop = ET.Element('xacro:property', name="params", value="${load_yaml('urdf_params.yaml')}")
	return prop

def urdf_xml_writer(filename):
	dh = csv_reader(filename)
	rows = len(dh)
	print('Number of rows: ' + str(rows))
	columns = len(dh[0])
	tree = ET.Element("robot", name="robot_bugkuc")
	base = link_xml_creator("base", "1")
	tool = link_xml_creator("tool", "0.01")
	parts_array = [base]
	rpy, xyz = find_rpy(filename)
	print(xyz)
	print(rpy)
	for i in range(rows):
		next_joint = joint_xml_creator(f'joint_{i}_{i+1}', rpy[i], xyz[i])			
		next_link = link_xml_creator(f'link_{i+1}', f'{dh[i][1]}')
		parts_array.extend((next_joint, next_link))

	parts_array.append(tool)
	tree.extend(parts_array)
	ET.ElementTree(tree).write('robot_lab2.urdf.xml', pretty_print=True)

	

if __name__ == '__main__':
	#print(csv_reader('dh_table.csv'))
	# r = rotation_matrix_calc('dh_table.csv')
	# print(find_rpy(r))
	urdf_xml_writer('dh_table.csv')
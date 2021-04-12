import csv
import json
from math import cos, sin, atan2, sqrt
import os
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
	for row in range(0, rows):
		rot_theta = transformations.rotation_matrix(dh[row][4], (0,0,1))
		rot_alpha = transformations.rotation_matrix(dh[row][3], (1,0,0))
		trans_a = transformations.translation_matrix((dh[row][1],0,0))
		trans_d = transformations.translation_matrix((0,0,dh[row][2]))
		h_matrix = trans_a @ rot_alpha @ trans_d @ rot_theta
		rpy_table.append(transformations.euler_from_matrix(h_matrix))
		xyz_table.append(transformations.translation_from_matrix(h_matrix))
	return rpy_table, xyz_table


def link_xml_creator(link_name, rpy="0 0 0"):
	params = get_params(link_name)

	link = ET.Element('link', name=params['link_name'])
	visual = ET.SubElement(link, "visual")
	visual_origin = ET.SubElement(visual, "origin", xyz=params['xyz'], rpy=rpy)
	geometry = ET.SubElement(visual, "geometry")
	if params['geometry_type'] == 'sphere':
		geometry_type = ET.SubElement(geometry, 'sphere', radius=params['radius'])
	elif params['geometry_type'] == 'box':
		geometry_type = ET.SubElement(geometry, 'box', size=params['size'])
	elif params['geometry_type'] == 'cylinder':
		geometry_type = ET.SubElement(geometry, 'cylinder', radius=params['radius'], length=params['length'])
	material = ET.SubElement(visual, "material", name=params['material'])
	color = ET.SubElement(material, "color", rgba=params['color'])
	return link

def joint_xml_creator(joint_name, rpy, xyz, fixed=False):
	params = get_params(joint_name)
	rpy = f'{rpy[0]} {rpy[1]} {rpy[2]}'
	xyz = f'{xyz[0]} {xyz[1]} {xyz[2]}'

	joint = ET.Element('joint', name=params['joint_name'], type=(params['joint_type'] if not fixed else "fixed"))
	parent = ET.SubElement(joint, 'parent', link=params['parent_name'])
	child = ET.SubElement(joint, 'child', link=params['child_name'])
	origin = ET.SubElement(joint, 'origin', xyz=xyz,rpy = rpy)
	if params['joint_type'] != 'fixed':
		axis = ET.SubElement(joint, 'axis', xyz=params['axis'])
		limit = ET.SubElement(joint, 'limit', lower=params['lower_limit'], upper=params['upper_limit'], effort=params['effort'], velocity=params['velocity'])
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
	base = link_xml_creator("link_0")
	parts_array = [base]
	rpy, xyz = find_rpy(filename)
	print(xyz)
	print(rpy)
	for i in range(rows):
		next_joint = joint_xml_creator(f'joint_{i}_{i+1}', rpy[i], xyz[i])			
		next_link = link_xml_creator(f'link_{i+1}')
		parts_array.extend((next_joint, next_link))

	tree.extend(parts_array)
	ET.ElementTree(tree).write('../urdf/robot_lab2.urdf.xml', pretty_print=True)

	
if __name__ == '__main__':
	#print(csv_reader('dh_table.csv'))
	# r = rotation_matrix_calc('dh_table.csv')
	# print(find_rpy(r))
	urdf_xml_writer('dh_table.csv')
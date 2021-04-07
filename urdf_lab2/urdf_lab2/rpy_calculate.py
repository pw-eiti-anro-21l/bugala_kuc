import csv
from math import cos, sin, atan2, sqrt
import os
from matrix import matrix_multiplier
#import xml.etree.ElementTree as ET
from lxml import etree as ET

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
	return dh

def generate_yaml_params(filename):
	dh = csv_reader(filename)
	rows = len(dh)
	columns = len(dh[0])
	file = open('urdf_params.yaml', 'w')
	for row in range(1, rows):
		h = [[cos(dh[row][4]), -sin(dh[row][4])*cos(dh[row][3]), sin(dh[row][4])*sin(dh[row][3]), dh[row][1]*cos(dh[row][4])],
			   [sin(dh[row][4]), cos(dh[row][4])*cos(dh[row][3]), -cos(dh[row][4])*sin(dh[row][3]), dh[row][1]*sin(dh[row][4])],
			   [0, sin(dh[row][3]), cos(dh[row][3]), dh[row][2]],
			   [0, 0, 0, 1]]
		[roll, pitch, yaw] = find_rpy(h)
		print([roll, pitch, yaw])
		file.write(dh[row][0] +":\n")
		file.write(" joint_xyz: "+str(0)+" "+str(0)+" "+str(0)+"\n")
		file.write(" joint_rpy: "+str(roll)+" "+str(pitch)+" "+str(yaw)+"\n")
		file.write(" link_xyz: "+str(0)+" "+str(0)+" "+str(float(dh[row][2]*(-0.5)))+"\n")
		file.write(" link_rpy: "+str(0)+" "+str(0)+" "+str(0)+"\n")
		file.write(" link_len: "+str(dh[row][2])+"\n")


def find_rpy(r):
	roll = atan2(r[2][1], r[2][2])
	pitch = atan2(-r[2][0], sqrt(r[2][1]*r[2][1] + r[2][2]*r[2][2]))
	yaw = atan2(r[1][0], r[0][0])
	return [roll, pitch, yaw]

def link_xml_creator():
	link_name = "dupa"
	link = ET.Element('link', name = link_name)
	inertial = ET.SubElement(link, 'inertial')
	mass = ET.SubElement(inertial, 'mass', value = "1") 
	inertial = ET.SubElement(inertial, 'inertia', ixx="100", ixy="0", ixz="0", iyy="100", iyz="0", izz="100")
	origin = ET.SubElement(inertial, "origin")
	visual = ET.SubElement(link, "visual")
	visual_origin = ET.SubElement(visual, "origin", xyz="0 0 0", rpy="1.57 0 0")
	geometry = ET.SubElement(visual, "geometry")
	cylinder = ET.SubElement(geometry, "cylinder", radius="0.01", length=".5")
	material = ET.SubElement(visual, "material", name="gray")
	color = ET.SubElement(material, "color", rgba=".2 .2 .2 1")
	collision = ET.SubElement(link, "collision")
	collision_origin = ET.SubElement(collision, "origin", xyz="0 0 0", rpy="1.57 0 0")
	collision_geometry = ET.SubElement(collision, "geometry")
	collision_cylinder = ET.SubElement(geometry, "cylinder", radius="0.01", length=".5")
	contact_cofficients = ET.SubElement(collision, "contact_cofficients", mu="0", kp="1000.0", kd="1.0")
	# tree = ET.ElementTree(link)
	# tree.write('robot_lab2.urdf.xml', pretty_print=True)
	return link


def joint_xml_creator():
    joint_name = "tilt"
    joint_type = "revolute"
    parent_link = "axis"
    child_link = "body"
    origin_xyz = "0 0 0"
    origin_rpy = "1.57 0 0"
    axis_xyz = "0 1 0"
    upper_limit = "0"
    lower_limit = "-0.5"
    effort_limit = "10"
    velocity_limit = "10"

    joint = ET.Element('joint', name = joint_name, type = joint_type)
    parent = ET.SubElement(joint, 'parent', link = parent_link)
    child = ET.SubElement(joint, 'child', link = child_link)
    origin = ET.SubElement(joint, 'origin', xyz = origin_xyz, rpy = origin_rpy)
    axis = ET.SubElement(joint, 'axis', xyz = axis_xyz)
    limit = ET.SubElement(joint, 'limit', upper = upper_limit, lower = lower_limit, effort = effort_limit, velocity = velocity_limit)
    # tree = ET.ElementTree(joint)
    # tree.write('robot_lab2.urdf.xml', pretty_print=True)
    return joint

def xacro_prop_xml():
	prop = ET.Element('xacro:property', name="params", value="${load_yaml('urdf_params.yaml')}")
	return prop

def urdf_xml_writer():
	dh = csv_reader(filename)
	rows = len(dh)
	columns = len(dh[0])
	tree = ET.Element("robot", name="robot_bugkuc", xacro="http://www.ros.org/wiki/xacro")
	prop = xacro_prop_xml()
	base = link_xml_creator()
	tree.extend([link, joint])
	ET.ElementTree(tree).write('robot_lab2.urdf.xml', pretty_print=True)

	

if __name__ == '__main__':
	#print(csv_reader('dh_table.csv'))
	# r = rotation_matrix_calc('dh_table.csv')
	# print(find_rpy(r))
	#urdf_xml_writer()
	generate_yaml_params('dh_table.csv')
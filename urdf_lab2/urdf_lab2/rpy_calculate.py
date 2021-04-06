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
	for i in range(1, 4):
		for j in range(1, 5):
			dh[i][j] = float(dh[i][j])
	return dh

def rotation_matrix_calc(filename):
	dh = csv_reader(filename)
	for i in range(1, 3):
		for j in range(1, 4):
			dh[i][j] = float(dh[i][j])

	h01 = [[cos(dh[1][4]), -sin(dh[1][4])*cos(dh[1][3]), sin(dh[1][4])*sin(dh[1][3]), dh[1][1]*cos(dh[1][4])],
		   [sin(dh[1][4]), cos(dh[1][4])*cos(dh[1][3]), -cos(dh[1][4])*sin(dh[1][3]), dh[1][1]*sin(dh[1][4])],
		   [0, sin(dh[1][3]), cos(dh[1][3]), dh[1][2]],
		   [0, 0, 0, 1]]
	h02 = [[cos(dh[2][4]), -sin(dh[2][4])*cos(dh[2][3]), sin(dh[2][4])*sin(dh[2][3]), dh[2][1]*cos(dh[2][4])],
		   [sin(dh[2][4]), cos(dh[2][4])*cos(dh[2][3]), -cos(dh[2][4])*sin(dh[2][3]), dh[2][1]*sin(dh[2][4])],
		   [0, sin(dh[2][3]), cos(dh[2][3]), dh[2][2]],
		   [0, 0, 0, 1]]  
	h03 = [[cos(dh[3][4]), -sin(dh[3][4])*cos(dh[3][3]), sin(dh[3][4])*sin(dh[3][3]), dh[3][1]*cos(dh[3][4])],
		   [sin(dh[3][4]), cos(dh[3][4])*cos(dh[3][3]), -cos(dh[3][4])*sin(dh[3][3]), dh[3][1]*sin(dh[3][4])],
		   [0, sin(dh[3][3]), cos(dh[3][3]), dh[3][2]],
		   [0, 0, 0, 1]]
	h12 = matrix_multiplier(h01, h02)
	h123 = matrix_multiplier(h12, h03)
	#print(h123)
	return h123

def find_rpy(r):
	roll = atan2(r[2][1], r[2][2])
	pitch = atan2(-r[2][0], sqrt(r[2][1]*r[2][1] + r[2][2]*r[2][2]))
	yaw = atan2(r[1][0], r[0][0])
	return [roll, pitch, yaw]

def link_xml_creator(r, p, y):
	link_name = "dupa"
	link = ET.Element('link', name = link_name)
	inertial = ET.SubElement(link, 'inertial')
	mass = ET.SubElement(inertial, 'mass', value = "1") 
	inertia = ET.SubElement(inertial, 'inertia', ixx="100", ixy="0", ixz="0", iyy="100", iyz="0", izz="100")
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
	tree = ET.ElementTree(link)
	tree.write('robot_lab2.urdf.xml', pretty_print=True)


if __name__ == '__main__':
	#print(csv_reader('dh_table.csv'))
	r = rotation_matrix_calc('dh_table.csv')
	print(find_rpy(r))
	link_xml_creator(1,2,3)
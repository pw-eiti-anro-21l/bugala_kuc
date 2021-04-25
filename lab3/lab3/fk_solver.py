import csv
import json
from math import cos, sin, atan2, sqrt
import os
from lxml import etree as ET
import mathutils

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

def solve(position):
	dh = csv_reader('/home/jan/dev_ws/src/bugala_kuc/lab3/lab3dh_table.csv')
	T_matrix = []
	x_axis, z_axis = (1,0,0), (0,0,1)
	for joint in range (0,len(dh)-1):
		rot_alpha = mathutils.Matrix.Rotation(dh[joint][3], 4, 'X')
		rot_theta = mathutils.Matrix.Rotation(dh[joint][4], 4, 'Z')
		trans_a = mathutils.Matrix.Translation((dh[joint][1],0,0))
		trans_d = mathutils.Matrix.Translation((0,0,dh[joint][2]+position[joint]))
		T_joint = rot_alpha @ trans_a @  rot_theta @ trans_d
		if (len(T_matrix) != 0):
			T_matrix = T_matrix @ T_joint
		else:
			T_matrix = T_joint
	return T_matrix

# def get_params(part, filename):
#     with open(filename, "r") as file:
#         read_file = json.load(file)
#     part_params = read_file[part]
#     return part_params


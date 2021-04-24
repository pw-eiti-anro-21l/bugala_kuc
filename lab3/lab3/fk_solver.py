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

def solve():
	dh = csv_reader('dh_table.csv')
	T_matrix = []
	x_axis, z_axis = (1,0,0), (0,0,1)
	for joint in dh:
		rot_alpha = transformations.rotation_matrix(joint[3], x_axis)
		rot_theta = transformations.rotation_matrix(joint[4], z_axis)
		trans_a = transformations.translation_matrix((joint[1],0,0))
		trans_d = transformations.translation_matrix((0,0,joint[2]))
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

if __name__ == '__main__':
	T = solve()
	print(T)
import csv
from math import cos, sin, atan2, sqrt
import os
from matrix import matrix_multiplier

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
	return h123

def find_rpy(r):
	roll = atan2(r[3][2], r[3][3])
	pitch = atan2(-r[3][1], sqrt(r[3][2]*r[3][2] + r[3][3]*r[3][3]))
	yaw = atan2(r[2][1], r[1][1])
	return [roll, pitch, yaw]

if __name__ == '__main__':
	#print(csv_reader('dh_table.csv'))
	r = rotation_matrix_calc('dh_table.csv')
	print(find_rpy(r))
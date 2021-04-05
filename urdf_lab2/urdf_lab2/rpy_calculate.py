import csv
import math
import os

def csv_reader(filename):
	dh = []
	with open(filename, 'r') as file:
		read_file = csv.reader(file)
		for row in read_file:
			dh.append(row)
	return dh

def rotation_matrix_calc(dh, filename):
	dh = csv_reader(filename)
	h01 = [[cos(dh[1][4]), -sin(dh[1][4])*cos(dh[1][3]), sin(dh[1][4])*sin(dh[1][3]), dh[1][1]*cos(dh[1][4])],
		   [sin(dh[1][4]), cos(dh[1][4])*cos(dh[1][3]), -cos(dh[1][4])*sin(dh[1][3]), dh[1][1]*sin(dh[1][4])],
		   []]
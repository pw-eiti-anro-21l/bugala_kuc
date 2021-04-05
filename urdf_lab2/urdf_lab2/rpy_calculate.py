import csv
import math
import os

def csv_reader(filename):
	dh = []
	with open(filename, 'r') as file:
		read_file = csv.reader(file)
		for row in read_file:
			DH.append(row)
	return dh

def rotation_matrix_calc(dh):
	rot1 = []
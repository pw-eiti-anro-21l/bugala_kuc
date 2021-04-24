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

# def get_params(part, filename):
#     with open(filename, "r") as file:
#         read_file = json.load(file)
#     part_params = read_file[part]
#     return part_params

if __name__ == '__main__':
	dh = csv_reader('dh_table.csv')
	print(dh)
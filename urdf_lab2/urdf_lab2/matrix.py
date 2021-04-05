def matrix_multiplier(a, b):
	
	rows_a = len(a)
	columns_a = len(a[0])

	rows_b = len(b)
	columns_b = len(b[0])
	
	if columns_a != rows_b:
		raise Exception("\n\nThe number of columns mismatch the number of rows. Cannot multiply.")
	
	c = [[0 for i in range(rows_a)] for j in range(columns_b)]
	
	for row in range(rows_a):
		for column  in range(columns_b):
			for i in range(rows_b):
				c[row][column] += a[row][i]*b[i][column] 
	return c

# def merge_matrices(R,T):
# 	H = R
# 	for i in range(len(R)):
# 		H[i].append(T[i])
# 	last_row = [0 for i in range(len(R[0])-1)]
# 	last_row.append(1)
# 	H.append(last_row)

# 	return H
	

if __name__ == '__main__':
	print('a')
	a = [[1,2,4],
	[3,4,6]]

	b = [1,4]
	
	c = merge_matrices(a,b)
	print(c)
	print('a')

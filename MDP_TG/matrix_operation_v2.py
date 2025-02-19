import random
import pickle

import math
from networkx import single_source_shortest_path

from networkx.classes.digraph import DiGraph
import time
import itertools
from itertools import product
import sys



#---compute square matrix with row sum and column sum equal to 1
def is_valid(matrix, row, col):
    n = len(matrix)
    return sum(matrix[row]) <= 1 and sum(matrix[i][col] for i in range(n)) <= 1

def backtrack(matrix, row, col):
    n = len(matrix)
    
    # If we have filled the entire matrix, add it to the list
    if row == n:
        return [row[:] for row in matrix]

    # Try placing a 1 in the current position
    if is_valid(matrix, row, col):
        matrix[row][col] = 1
        next_row, next_col = divmod(col + 1, n)

        # Temporarily increase the recursion limit
        sys.setrecursionlimit(max(sys.getrecursionlimit(), row + 1))

        result = backtrack(matrix, row + 1 if next_col == 0 else row, next_col)

        # Reset the recursion limit
        sys.setrecursionlimit(max(sys.getrecursionlimit(), 1000))

        matrix[row][col] = 0  # Backtrack

        # Try placing a 0 in the current position
        result += backtrack(matrix, row, next_col)
        return result

    # If placing a 1 is not valid, move to the next column
    return backtrack(matrix, row, col + 1)

def generate_square_matrices(n):
    matrix = [[0] * n for _ in range(n)]
    return backtrack(matrix, 0, 0)

#---compute n*m matrix with row sum equal to 1
def generate_matrices(n, m):
    matrices = []

    # Generate all possible matrices with row sum equals 1
    for perm in product(range(2), repeat=n * m):
        matrix = [list(perm[i * m : (i + 1) * m]) for i in range(n)]
        row_sums = [sum(row) for row in matrix]
        column_sums = compute_column_sums(matrix)
        if all(sum == 1 for sum in row_sums) and all_elements_small_equal_one(column_sums[1:]):
            matrices.append(matrix)

    return matrices

def enumerate_matrices(rows, cols):
    # Generate all possible matrices
    matrices = []
    for i in range(2 ** (rows * cols)):
        matrix = [[(i >> j) & 1 for j in range(rows * cols)][k * cols:(k + 1) * cols] for k in range(rows)]
        matrices.append(matrix)
    return matrices

def compute_row_sums(matrix):
    # Initialize an empty list to store row sums
    row_sums = []

    # Iterate through each row and compute the sum
    for row in matrix:
        row_sum = sum(row)
        row_sums.append(row_sum)

    return row_sums

def compute_column_sums(matrix):
    # Get the number of columns
    num_columns = len(matrix[0]) if matrix else 0

    # Initialize an empty list to store column sums
    column_sums = [0] * num_columns

    # Iterate through each column and compute the sum
    for row in matrix:
        for col_index, value in enumerate(row):
            column_sums[col_index] += value

    return column_sums

def all_elements_small_equal_one(row):
    for element in row:
        if element > 1:
            return False
    return True

#---------------------------------
#---------------------------------
def human_robot_co_assembly_scenario(obj_num, human_action_allowed):
    #all_matrices = generate_matrices(obj_num, obj_num+1)
    if obj_num == 2:
        with open('pickle_data/human_robot_2.pkl', 'rb') as pickle_file:
            all_matrices = pickle.load(pickle_file)
        valid_matrix = []
        for matrix in all_matrices:
            column_sums = compute_column_sums(matrix)
            if 1:
                valid_matrix.append(matrix)
    elif obj_num == 3:
        with open('pickle_data/human_robot_3.pkl', 'rb') as pickle_file:
            all_matrices = pickle.load(pickle_file)
        valid_matrix = []
        for matrix in all_matrices:
            column_sums = compute_column_sums(matrix)
            if 1:
                if column_sums[-1]<1:
                    valid_matrix.append(matrix)
                else: #column_sums[-1]=1
                    if column_sums[1] == 1 and column_sums[2] == 1:
                        valid_matrix.append(matrix)
    elif obj_num == 4:
        with open('pickle_data/human_robot_4.pkl', 'rb') as pickle_file:
            all_matrices = pickle.load(pickle_file)
        valid_matrix = []
        for matrix in all_matrices:
            column_sums = compute_column_sums(matrix)
            if 1:
                if column_sums[3] < 1 and column_sums[4] < 1:
                    valid_matrix.append(matrix)
                elif column_sums[3] == 1 and column_sums[4] < 1:
                    if column_sums[1] == 1:
                        valid_matrix.append(matrix)
                elif column_sums[3] < 1 and column_sums[4] == 1:
                    if column_sums[2] == 1:
                        valid_matrix.append(matrix)
                else:
                    if column_sums[1] == 1 and column_sums[2] == 1: 
                        valid_matrix.append(matrix)
    elif obj_num == 5:
        with open('pickle_data/human_robot_5.pkl', 'rb') as pickle_file:
            all_matrices = pickle.load(pickle_file)
        valid_matrix = []
        for matrix in all_matrices:
            column_sums = compute_column_sums(matrix)
            if all_elements_small_equal_one(column_sums[1:]):
                if column_sums[5]<1:
                    if column_sums[3] < 1 and column_sums[4] < 1:
                        valid_matrix.append(matrix)
                    elif column_sums[3] == 1 and column_sums[4] < 1:
                        if column_sums[1] == 1:
                            valid_matrix.append(matrix)
                    elif column_sums[3] < 1 and column_sums[4] == 1:
                        if column_sums[2] == 1:
                            valid_matrix.append(matrix)
                    else:
                        if column_sums[1] == 1 and column_sums[2] == 1: 
                            valid_matrix.append(matrix)
                else: #column_sums[-1]=1
                    if column_sums[1] == 1 and column_sums[2] == 1 and column_sums[3] == 1 and column_sums[4] == 1:
                        valid_matrix.append(matrix)
    elif obj_num == 6:
        with open('pickle_data/human_robot_6.pkl', 'rb') as pickle_file:
            all_matrices = pickle.load(pickle_file)
        valid_matrix = []
        for matrix in all_matrices:
            column_sums = compute_column_sums(matrix)
            if all_elements_small_equal_one(column_sums[1:]):
                if column_sums[5] < 1 and column_sums[6] < 1:
                    if column_sums[3] < 1 and column_sums[4] < 1:
                        valid_matrix.append(matrix)
                    elif column_sums[3] == 1 and column_sums[4] < 1:
                        if column_sums[1] == 1:
                            valid_matrix.append(matrix)
                    elif column_sums[3] < 1 and column_sums[4] == 1:
                        if column_sums[2] == 1:
                            valid_matrix.append(matrix)
                    else:
                        if column_sums[1] == 1 and column_sums[2] == 1: 
                            valid_matrix.append(matrix)
                elif column_sums[5] == 1 and column_sums[6] < 1:
                    if column_sums[1] == 1 and column_sums[3] == 1:
                        if column_sums[4] < 1:
                            valid_matrix.append(matrix)
                        else:
                            if column_sums[2] == 1:
                                valid_matrix.append(matrix)
                elif column_sums[5] < 1 and column_sums[6] == 1:
                    if column_sums[2] == 1 and column_sums[4] == 1:
                        if column_sums[3] < 1:
                            valid_matrix.append(matrix)
                        else:
                            if column_sums[1] == 1:
                                valid_matrix.append(matrix)                  
                else:
                    if column_sums[3] == 1 and column_sums[4] == 1 and column_sums[2] == 1 and column_sums[1] == 1:
                        valid_matrix.append(matrix)
    elif obj_num == 7:
        valid_matrix = []
        for matrix in all_matrices:
            column_sums = compute_column_sums(matrix)
            if all_elements_small_equal_one(column_sums[1:]):
                if column_sums[-1]<1:
                    if column_sums[5] < 1 and column_sums[6] < 1:
                        if column_sums[3] < 1 and column_sums[4] < 1:
                            valid_matrix.append(matrix)
                        elif column_sums[3] == 1 and column_sums[4] < 1:
                            if column_sums[1] == 1:
                                valid_matrix.append(matrix)
                        elif column_sums[3] < 1 and column_sums[4] == 1:
                            if column_sums[2] == 1:
                                valid_matrix.append(matrix)
                        else:
                            valid_matrix.append(matrix) 
                    elif column_sums[5] == 1 and column_sums[6] < 1:
                        if column_sums[3] == 1 and column_sums[1] == 1:
                            if column_sums[4] == 1:
                                if column_sums[2] == 1:
                                    valid_matrix.append(matrix)
                            else:
                                valid_matrix.append(matrix)
                    elif column_sums[5] < 1 and column_sums[6] == 1:
                        if column_sums[4] == 1 and column_sums[2] == 1:
                            if column_sums[3] == 1:
                                if column_sums[1] == 1:
                                    valid_matrix.append(matrix)
                            else:
                                valid_matrix.append(matrix)
                    else:
                        if column_sums[1] == 1 and column_sums[2] == 1 and column_sums[3] == 1 and column_sums[4] == 1:
                            valid_matrix.append(matrix)
                else: #column_sums[-1]=1
                    if column_sums[1] == 1 and column_sums[2] == 1 and column_sums[3] == 1 and column_sums[4] == 1 and column_sums[5] == 1 and column_sums[6] == 1:
                        valid_matrix.append(matrix)
    print('Number of valid matrices: %s' %len(valid_matrix))

    #----
    U = []
    U.append(tuple('ST'))
    for obj in range(obj_num):
        for loc in range(obj_num+1):
            u = (obj, loc)
            U.append(u)

    U_h = []
    U_h.append(tuple('ST'))
    for obj in range(obj_num):
        for loc in range(1, obj_num+1):
            u = (obj, loc)
            U_h.append(u)

    #----
    obj_matrix = [[0] * (obj_num+1) for _ in range(obj_num)]
    for id in range(obj_num):
        obj_matrix[id][id+1] = 1
    target = tuple(tuple(row) for row in obj_matrix)

    obstacle = set()
    if obj_num == 2 or obj_num == 3:
        for matrix in valid_matrix:
            if matrix[0][2] == 1 and matrix[1][1] == 1:
                obs = tuple(tuple(row) for row in matrix)
                obstacle.add(obs)
    # if obj_num == 5:
    #     for matrix in valid_matrix:
    #         if matrix[0][2] == 1 and matrix[1][1] == 1 and matrix[2][4] == 1 and matrix[3][3] == 1:
    #             obs = tuple(tuple(row) for row in matrix)
    #             obstacle.add(obs)

    if obj_num == 4 or obj_num == 5 or obj_num == 6:
        for matrix in valid_matrix:
            if matrix[0][2] == 1 and matrix[1][1] == 1:
                obs = tuple(tuple(row) for row in matrix)
                obstacle.add(obs)
            if matrix[2][4] == 1 and matrix[3][3] == 1:
                obs = tuple(tuple(row) for row in matrix)
                obstacle.add(obs)



    init_matrix = [[0] * (obj_num+1) for _ in range(obj_num)]
    for id in range(obj_num):
        init_matrix[id][0] = 1
    init_node_robot = tuple(tuple(row) for row in init_matrix)

    return valid_matrix, U, U_h, target, obstacle, init_node_robot, human_action_allowed

def valid_prefix_node(prod_mdp, T):
    for init_node in prod_mdp.graph['initial']:
        path_init = single_source_shortest_path(prod_mdp, init_node)
        print('Reachable from init size: %s' % len(list(path_init.keys())))
        if not set(path_init.keys()).intersection(T):
            print("Initial node can not reach sf")
        Sn = set(path_init.keys()).difference(T)
        # ----find bad states that can not reach MEC
        simple_digraph = DiGraph()
        simple_digraph.add_edges_from(((v, u) for u, v in prod_mdp.edges()))
        path = single_source_shortest_path(
            simple_digraph, random.sample(sorted(T), 1)[0])
        reachable_set = set(path.keys())
        print('States that can reach sf, size: %s' % str(len(reachable_set)))
        Sd = Sn.difference(reachable_set)
        Sr = Sn.intersection(reachable_set)
        # #--------------
        print('Sn size: %s; Sd inside size: %s; Sr inside size: %s' %
                (len(Sn), len(Sd), len(Sr)))
    return Sr

#----
# all_matrix = generate_matrices(6, 7)
# print(len(all_matrix))

# with open('pickle_data/human_robot_6.pkl', 'wb') as pickle_file:
#     pickle.dump(all_matrix, pickle_file)

# with open('pickle_data/human_robot_6.pkl', 'rb') as pickle_file:
#     all_matrices = pickle.load(pickle_file)

# print(len(all_matrices))
# # #print(all_matrices)

# #----
# object = 6
# with open('pickle_data/human_robot_5.pkl', 'rb') as pickle_file:
#     all_matrices = pickle.load(pickle_file)
# print(len(all_matrices))

# # Generate all combinations of length 1 with elements [0, 1]
# all_combinations = [0, 1]

# # Generate all possible 1x7 matrices
# valid_matrices = list(itertools.product(*[all_combinations] * (object+1)))

# # Filter matrices with a row sum of 1
# valid_rows = [list(matrix) for matrix in valid_matrices if sum(list(matrix)) == 1]

# expand_matrix_set_set = []
# for matrix in all_matrices:
#     row_dict = dict()
#     expand_matrix_set = []
#     #print(matrix)
#     for i, row in enumerate(matrix):
#         row_dict[i] = (row + [0], [0, 0, 0, 0, 0, 0, 1])
#     #print(row_dict)

#     for row0 in row_dict[0]:
#         for row1 in row_dict[1]:
#             for row2 in row_dict[2]:
#                 for row3 in row_dict[3]:
#                     for row4 in row_dict[4]:
#                         for row5 in valid_rows:
#                             expand_matrix = [row0, row1, row2, row3, row4, row5]
#                             column_sums = compute_column_sums(expand_matrix)
#                             if all_elements_small_equal_one(column_sums[1:]):
#                                 expand_matrix_set.append(expand_matrix)
#                                 if expand_matrix not in expand_matrix_set_set:
#                                     expand_matrix_set_set.append(expand_matrix)
#     print(len(expand_matrix_set))
            
# print(len(expand_matrix_set_set))

# with open('pickle_data/human_robot_6.pkl', 'wb') as pickle_file:
#     pickle.dump(expand_matrix_set_set, pickle_file)


            

        



from math import sqrt
import math
import time

def create_world_honeycomb(num_x, num_y, square_root, Obstacle, U, C):
    x_seq1 = [3*i + 1 for i in range(num_x)]
    x_seq2 = [3*i + 5/2 for i in range(num_x)]
    x_seq = [x_seq1, x_seq2]
    y_seq1 = [square_root*(i + 1) for i in range(num_x)]
    y_seq2 = [square_root*(i + 1/2) for i in range(num_x)]
    y_seq = [y_seq1, y_seq2]
    Base1 = [(3*num_x - 1/2, square_root/2), (3*num_x - 1/2, 3*square_root/2)]
    Base2 = [(3*num_x - 1/2, square_root*(num_y-1/2))]
    Base3 = [(1, square_root*num_y), (1, square_root*(num_y-1))]

    WS_node_dict = dict()
    for idx, x_sq in enumerate(x_seq):
        y_sq = y_seq[idx]
        for x in x_sq:
            for y in y_sq:
                node = (x, y)
                if node in Base1:
                    WS_node_dict[node] = {frozenset(['base1']): 1.0, }
                elif node in Base2:
                    WS_node_dict[node] = {frozenset(['base2']): 1.0, }
                elif node in Base3:
                    WS_node_dict[node] = {frozenset(['base3']): 1.0, }
                elif node in Obstacle:
                    WS_node_dict[node] = {frozenset(['obstacle']): 1.0, }
                else:
                    WS_node_dict[node] = {frozenset(): 1.0, }
    
    # ------------------------------------
    robot_nodes = dict()
    for loc, prop in WS_node_dict.items():
        for d in [1, 2, 3, 4]: # ['N', 'S', 'E', 'W']
            node = (loc[0], loc[1], d)
            robot_nodes[tuple(node)] = prop
    print('Number of states: %s' % len(robot_nodes))

    #------------
    C_dict = dict()
    C_dict[U[0]] = 2
    C_dict[U[1]] = 4
    C_dict[U[2]] = 3
    C_dict[U[3]] = 3
    P_FR = [0.1, 0.8, 0.1]
    P_BK = [0.15, 0.7, 0.15]
    P_TR = [0.05, 0.9, 0.05]
    P_TL = [0.05, 0.9, 0.05]
    P_dict = dict()
    P_dict[U[0]] = P_FR
    P_dict[U[1]] = P_BK
    P_dict[U[2]] = P_TR
    P_dict[U[3]] = P_TL
    # -------------

    set_nodes = ()
    robot_edges = dict()
    robot_state_action = dict()
    for fnode in robot_nodes.keys():
        fx = fnode[0]
        fy = fnode[1]
        fd = fnode[2]
        # action FR
        u = U[0]
        c = C[0]
        if fd == 1:
            t_nodes = [((fx+3/2, fy+square_root/2, fd),), ((fx, fy+square_root, fd),), ((fx-3/2, fy+square_root/2, fd),)]
        if fd == 2:
            t_nodes = [((fx+3/2, fy-square_root/2, fd),), ((fx, fy-square_root, fd),),  ((fx-3/2, fy-square_root/2, fd),)]
        if fd == 3:
            t_nodes = [((fx, fy+square_root, fd),), ((fx+3/2, fy+square_root/2, fd), (fx+3/2, fy-square_root/2, fd)), ((fx, fy-square_root, fd),)]
        if fd == 4:
            t_nodes = [((fx, fy+square_root, fd),), ((fx-3/2, fy+square_root/2, fd), (fx-3/2, fy-square_root/2, fd)), ((fx, fy-square_root, fd),)]
        tnode_valid_set = []
        if fd == 1 or fd == 2:
            if t_nodes[1][0] in robot_nodes.keys():
                for k, tnode_set in enumerate(t_nodes):
                    tnode_valid = ()
                    for tnode in tnode_set:
                        if tnode in list(robot_nodes.keys()):
                            tnode_valid += (tuple(tnode),)
                    if tnode_valid:
                        robot_edges[(fnode, u, tnode_valid)] = (P_FR[k], c)
                        tnode_valid_set.append(tnode_valid)
                        if len(tnode_valid)>1:
                            set_nodes += (tuple(tnode_valid),)
        else:
            if t_nodes[1][0] in robot_nodes.keys() or t_nodes[1][1] in robot_nodes.keys():
                for k, tnode_set in enumerate(t_nodes):
                    tnode_valid = ()
                    for tnode in tnode_set:
                        if tnode in list(robot_nodes.keys()):
                            tnode_valid += (tuple(tnode),)
                    if tnode_valid:
                        robot_edges[(fnode, u, tnode_valid)] = (P_FR[k], c)
                        tnode_valid_set.append(tnode_valid)
                        if len(tnode_valid)>1:
                            set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set
        # action BK
        u = U[1]
        c = C[1]
        if fd == 1:
            t_nodes = [((fx+3/2, fy-square_root/2, fd),), ((fx, fy-square_root, fd),),  ((fx-3/2, fy-square_root/2, fd),)]
        if fd == 2:
            t_nodes = [((fx+3/2, fy+square_root/2, fd),), ((fx, fy+square_root, fd),),  ((fx-3/2, fy+square_root/2, fd),)]
        if fd == 3:
            t_nodes = [((fx, fy+square_root, fd),), ((fx-3/2, fy+square_root/2, fd), (fx-3/2, fy-square_root/2, fd)), ((fx, fy-square_root, fd),)]
        if fd == 4:
            t_nodes = [((fx, fy+square_root, fd),), ((fx+3/2, fy+square_root/2, fd), (fx+3/2, fy-square_root/2, fd)), ((fx, fy-square_root, fd),)]
        tnode_valid_set = []
        if fd == 1 or fd == 2:
            if t_nodes[1][0] in robot_nodes.keys():
                for k, tnode_set in enumerate(t_nodes):
                    tnode_valid = ()
                    for tnode in tnode_set:
                        if tnode in list(robot_nodes.keys()):
                            tnode_valid += (tuple(tnode),)
                    if tnode_valid:
                        robot_edges[(fnode, u, tnode_valid)] = (P_BK[k], c)
                        tnode_valid_set.append(tnode_valid)
                        if len(tnode_valid)>1:
                            set_nodes += (tuple(tnode_valid),)
        else:
            if t_nodes[1][0] in robot_nodes.keys() or t_nodes[1][1] in robot_nodes.keys():
                for k, tnode_set in enumerate(t_nodes):
                    tnode_valid = ()
                    for tnode in tnode_set:
                        if tnode in list(robot_nodes.keys()):
                            tnode_valid += (tuple(tnode),)
                    if tnode_valid:
                        robot_edges[(fnode, u, tnode_valid)] = (P_BK[k], c)
                        tnode_valid_set.append(tnode_valid)
                        if len(tnode_valid)>1:
                            set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set
        # action TR
        u = U[2]
        c = C[2]
        if fd == 1:
            t_nodes = [((fx, fy, 1),), ((fx, fy, 3),), ((fx, fy, 2),)]
        if fd == 2:
            t_nodes = [((fx, fy, 2),), ((fx, fy, 4),), ((fx, fy, 1),)]
        if fd == 3:
            t_nodes = [((fx, fy, 3),), ((fx, fy, 2),), ((fx, fy, 4),)]
        if fd == 4:
            t_nodes = [((fx, fy, 4),), ((fx, fy, 1),), ((fx, fy, 3),)]
        tnode_valid_set = []
        for k, tnode_set in enumerate(t_nodes):
            tnode_valid = ()
            for tnode in tnode_set:
                if tnode in list(robot_nodes.keys()):
                    tnode_valid += (tuple(tnode),)
            if tnode_valid:
                robot_edges[(fnode, u, tnode_valid)] = (P_TR[k], c)
                tnode_valid_set.append(tnode_valid)
                if len(tnode_valid)>1:
                    set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set
        # action TL
        u = U[3]
        c = C[3]
        if fd == 2:
            t_nodes = [((fx, fy, 2),), ((fx, fy, 3),), ((fx, fy, 1),)]
        if fd == 1:
            t_nodes = [((fx, fy, 1),), ((fx, fy, 4),), ((fx, fy, 2),)]
        if fd == 4:
            t_nodes = [((fx, fy, 4),), ((fx, fy, 2),), ((fx, fy, 3),)]
        if fd == 3:
            t_nodes = [((fx, fy, 3),), ((fx, fy, 1),), ((fx, fy, 4),)]
        tnode_valid_set = []
        for k, tnode_set in enumerate(t_nodes):
            tnode_valid = ()
            for tnode in tnode_set:
                if tnode in list(robot_nodes.keys()):
                    tnode_valid += (tuple(tnode),)
            if tnode_valid:
                robot_edges[(fnode, u, tnode_valid)] = (P_TL[k], c)
                tnode_valid_set.append(tnode_valid)
                if len(tnode_valid)>1:
                    set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set

    return WS_node_dict, robot_nodes, robot_edges, set_nodes, robot_state_action, C_dict, P_dict


#-----------------------------------------------------
def create_world_grid(num_x, num_y, Obstacle, U, C):
    x_seq = [2*i + 1 for i in range(num_x)]
    y_seq = [2*i + 1 for i in range(num_y)]
    Base1 = [(2*num_x - 1, 1)]
    Base2 = [(2*num_x - 1, 2*num_y -1)]
    Base3 = [(1, 2*num_y -1)]

    WS_node_dict = dict()
    if 1:
        for x in x_seq:
            for y in y_seq:
                node = (x, y)
                if node in Base1:
                    WS_node_dict[node] = {frozenset(['base1']): 1.0, }
                elif node in Base2:
                    WS_node_dict[node] = {frozenset(['base2']): 1.0, }
                elif node in Base3:
                    WS_node_dict[node] = {frozenset(['base3']): 1.0, }
                elif node in Obstacle:
                    WS_node_dict[node] = {frozenset(['obstacle']): 1.0, }
                else:
                    WS_node_dict[node] = {frozenset(): 1.0, }
    
    # ------------------------------------
    robot_nodes = dict()
    for loc, prop in WS_node_dict.items():
        for d in [1, 2, 3, 4]: # ['N', 'S', 'E', 'W']
            node = (loc[0], loc[1], d)
            robot_nodes[tuple(node)] = prop
    print('Number of states: %s' % len(robot_nodes))

    #------------
    C_dict = dict()
    C_dict[U[0]] = 2
    C_dict[U[1]] = 4
    C_dict[U[2]] = 3
    C_dict[U[3]] = 3
    P_FR = [0.1, 0.9]
    P_BK = [0.1, 0.9]
    P_TR = [0.1, 0.9]
    P_TL = [0.1, 0.9]
    P_dict = dict()
    P_dict[U[0]] = P_FR
    P_dict[U[1]] = P_BK
    P_dict[U[2]] = P_TR
    P_dict[U[3]] = P_TL
    # -------------

    set_nodes = ()
    robot_edges = dict()
    robot_state_action = dict()
    for fnode in robot_nodes.keys():
        fx = fnode[0]
        fy = fnode[1]
        fd = fnode[2]
        # action FR
        u = U[0]
        c = C[0]
        if fd == 1:
            t_nodes = [[(fx-2, fy+2, fd), (fx+2, fy+2, fd)], [(fx, fy+2, fd)]]
        if fd == 2:
            t_nodes = [[(fx-2, fy-2, fd), (fx+2, fy-2, fd)], [(fx, fy-2, fd)]]
        if fd == 3:
            t_nodes = [[(fx+2, fy-2, fd), (fx+2, fy+2, fd)], [(fx+2, fy, fd)]]
        if fd == 4:
            t_nodes = [[(fx-2, fy-2, fd), (fx-2, fy+2, fd)], [(fx-2, fy, fd)]]
        tnode_valid_set = []
        for k, tnode_set in enumerate(t_nodes):
            tnode_valid = ()
            for tnode in tnode_set:
                if tnode in list(robot_nodes.keys()):
                    tnode_valid += (tuple(tnode),)
            if tnode_valid:
                robot_edges[(fnode, u, tnode_valid)] = (P_FR[k], c)
                tnode_valid_set.append(tnode_valid)
                if len(tnode_valid)>1:
                    set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set
        # action BK
        u = U[1]
        c = C[1]
        if fd == 1:
            t_nodes = [[(fx-2, fy-2, fd), (fx+2, fy-2, fd)], [(fx, fy-2, fd)]]
        if fd == 2:
            t_nodes = [[(fx-2, fy+2, fd), (fx+2, fy+2, fd)], [(fx, fy+2, fd)]]
        if fd == 3:
            t_nodes = [[(fx-2, fy-2, fd), (fx-2, fy+2, fd)], [(fx-2, fy, fd)]]
        if fd == 4:
            t_nodes = [[(fx+2, fy-2, fd), (fx+2, fy+2, fd)], [(fx+2, fy, fd)]]
        tnode_valid_set = []
        for k, tnode_set in enumerate(t_nodes):
            tnode_valid = ()
            for tnode in tnode_set:
                if tnode in list(robot_nodes.keys()):
                    tnode_valid += (tuple(tnode),)
            if tnode_valid:
                robot_edges[(fnode, u, tnode_valid)] = (P_BK[k], c)
                tnode_valid_set.append(tnode_valid)
                if len(tnode_valid)>1:
                    set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set
        # action TR
        u = U[2]
        c = C[2]
        if fd == 1:
            t_nodes = [[(fx, fy, 1), (fx, fy, 2)], [(fx, fy, 3)]]
        if fd == 2:
            t_nodes = [[(fx, fy, 2), (fx, fy, 1)], [(fx, fy, 4)]]
        if fd == 3:
            t_nodes = [[(fx, fy, 3), (fx, fy, 4)], [(fx, fy, 2)]]
        if fd == 4:
            t_nodes = [[(fx, fy, 4), (fx, fy, 3)], [(fx, fy, 1)]]
        tnode_valid_set = []
        for k, tnode_set in enumerate(t_nodes):
            tnode_valid = ()
            for tnode in tnode_set:
                if tnode in list(robot_nodes.keys()):
                    tnode_valid += (tuple(tnode),)
            if tnode_valid:
                robot_edges[(fnode, u, tnode_valid)] = (P_TR[k], c)
                tnode_valid_set.append(tnode_valid)
                if len(tnode_valid)>1:
                    set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set
        # action TL
        u = U[3]
        c = C[3]
        if fd == 2:
            t_nodes = [[(fx, fy, 2), (fx, fy, 1)], [(fx, fy, 3)]]
        if fd == 1:
            t_nodes = [[(fx, fy, 1), (fx, fy, 2)], [(fx, fy, 4)]]
        if fd == 4:
            t_nodes = [[(fx, fy, 4), (fx, fy, 3)], [(fx, fy, 2)]]
        if fd == 3:
            t_nodes = [[(fx, fy, 3), (fx, fy, 4)], [(fx, fy, 1)]]
        tnode_valid_set = []
        for k, tnode_set in enumerate(t_nodes):
            tnode_valid = ()
            for tnode in tnode_set:
                if tnode in list(robot_nodes.keys()):
                    tnode_valid += (tuple(tnode),)
            if tnode_valid:
                robot_edges[(fnode, u, tnode_valid)] = (P_TL[k], c)
                tnode_valid_set.append(tnode_valid)
                if len(tnode_valid)>1:
                    set_nodes += (tuple(tnode_valid),)
        robot_state_action[(fnode, u)] = tnode_valid_set

    return WS_node_dict, robot_nodes, robot_edges, set_nodes, robot_state_action, C_dict, P_dict
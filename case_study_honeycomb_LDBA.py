from MDP_TG.MDPST import Motion_MDPST, Product_MDPST_ldba, syn_full_plan_mdpst
from MDP_TG.smdp import Motion_MDP
from MDP_TG.ldba import LDBA, Product_Ldba
from MDP_TG.compute_SEC import compute_SEC
from MDP_TG.vis import visualize_world_paths, visualize_world, analyze_events

import math
import time

t0 = time.time()

# -------- real example -------
WS_d = 1
square_root=math.sqrt(3)

WS_node_dict = {
    # base stations
    (1, square_root): {frozenset(): 1.0, },
    (1, square_root*2): {frozenset(): 1.0, },
    (1, square_root*3): {frozenset(): 1.0, },
    (1, square_root*4): {frozenset(['base3']): 1.0, },
    (1, square_root*5): {frozenset(['base3']): 1.0, },
    (4, square_root): {frozenset(): 1.0, },
    (4, square_root*2): {frozenset(): 1.0, },
    (4, square_root*3): {frozenset(): 1.0, },
    (4, square_root*4): {frozenset(): 1.0, },
    (4, square_root*5): {frozenset(): 1.0, },
    (7, square_root): {frozenset(): 1.0, },
    (7, square_root*2): {frozenset(): 1.0, },
    (7, square_root*3): {frozenset(): 1.0, },
    (7, square_root*4): {frozenset(['obstacle']): 1.0, },
    (7, square_root*5): {frozenset(): 1.0, },
    (10, square_root): {frozenset(): 1.0, },
    (10, square_root*2): {frozenset(): 1.0, },
    (10, square_root*3): {frozenset(): 1.0, },
    (10, square_root*4): {frozenset(): 1.0, },
    (10, square_root*5): {frozenset(): 1.0, },
    (13, square_root): {frozenset(): 1.0, },
    (13, square_root*2): {frozenset(): 1.0, },
    (13, square_root*3): {frozenset(): 1.0, },
    (13, square_root*4): {frozenset(): 1.0, },
    (13, square_root*5): {frozenset(): 1.0, },
    (5/2, square_root/2): {frozenset(['obstacle']): 1.0, },
    (5/2, square_root*3/2): {frozenset(['obstacle']): 1.0, },
    (5/2, square_root*5/2): {frozenset(): 1.0, },
    (5/2, square_root*7/2): {frozenset(): 1.0, },
    (5/2, square_root*9/2): {frozenset(): 1.0, },
    (11/2, square_root/2): {frozenset(): 1.0, },
    (11/2, square_root*3/2): {frozenset(): 1.0, },
    (11/2, square_root*5/2): {frozenset(): 1.0, },
    (11/2, square_root*7/2): {frozenset(): 1.0, },
    (11/2, square_root*9/2): {frozenset(): 1.0, },
    (17/2, square_root/2): {frozenset(): 1.0, },
    (17/2, square_root*3/2): {frozenset(): 1.0, },
    (17/2, square_root*5/2): {frozenset(): 1.0, },
    (17/2, square_root*7/2): {frozenset(): 1.0, },
    (17/2, square_root*9/2): {frozenset(): 1.0, },
    (23/2, square_root/2): {frozenset(): 1.0, },
    (23/2, square_root*3/2): {frozenset(): 1.0, },
    (23/2, square_root*5/2): {frozenset(): 1.0, },
    (23/2, square_root*7/2): {frozenset(): 1.0, },
    (23/2, square_root*9/2): {frozenset(): 1.0, },
    (29/2, square_root/2): {frozenset(['base1']): 1.0, },
    (29/2, square_root*3/2): {frozenset(['base1']): 1.0, },
    (29/2, square_root*5/2): {frozenset(): 1.0, },
    (29/2, square_root*7/2): {frozenset(): 1.0, },
    (29/2, square_root*9/2): {frozenset(['base2']): 1.0, },
}

# ----
visualize_world(WS_d, WS_node_dict, 'world')
t1 = time.time()
print('visualize world done, time: %s' %str(t1-t0))

# ------------------------------------
robot_nodes = dict()
for loc, prop in WS_node_dict.items():
    #for d in ['N', 'S', 'E', 'W']:
    for d in [1, 2, 3, 4]:
        node = (loc[0], loc[1], d)
        robot_nodes[tuple(node)] = prop
# ------------------------------------
print('Number of states: %s' % len(robot_nodes))
initial_node = (1, square_root, 1)
initial_label = frozenset()

#----
#U = [tuple('FR'), tuple('BK'), tuple('TR'), tuple('TL'), tuple('ST')]
U=[tuple('1'), tuple('2'), tuple('3'),tuple('4')]
C = [2, 4, 3, 3, 1]
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

#--------------------------------------------
#--------construct MDPST---------------------
motion_mdpst = Motion_MDPST(robot_nodes, robot_edges, robot_state_action, U, C_dict,
                        initial_node, initial_label)

#-----Build the LDBA from LTL formula----
reach_avoid = '(! obstacle) U target'
persist_reach_avoid = 'G F base1 & G F base2 & G F base3 & G ! obstacle'
ldba = LDBA(persist_reach_avoid)
t2 = time.time()
print('LDBA done, time: %s' % str(t2-t1))

# ----Build the Product MDPST-----------------
prod_mdpst = Product_MDPST_ldba(motion_mdpst, ldba, P_dict)
t3 = time.time()
print('Product LDBA done, time: %s' % str(t3-t2))

#####----calculate accepting maximal set-valued end-component for Product MDPST----------
motion_mdp = Motion_MDP(robot_nodes, robot_edges, U,
                        initial_node, initial_label)

prod_mdp = Product_Ldba(motion_mdp, ldba)
t4 = time.time()
print('Product LDBA done, time: %s' % str(t4-t3))
#----
prod_mdp.compute_S_f(set(prod_mdp.nodes()))
t4 = time.time()
print('Compute accepting MEC done, time: %s' % str(t4-t3))

#----
AM_SEC, SEC_single, Pre_node, suffix_action = compute_SEC(prod_mdp, prod_mdpst, U)

#-------------------Strategy synthesis--------------------------
print(prod_mdpst.prod_init)
if prod_mdpst.prod_init[0] in set(AM_SEC):
    print("Initial state is accepting!")
    print('Optimal probability from initial state is 1.')
else:
    best_prefix_plan = syn_full_plan_mdpst(prod_mdpst, AM_SEC, Pre_node)
    prefix_action = best_prefix_plan[0]
    print(prefix_action)
    Value = best_prefix_plan[1]
    print(Value)
    print('Optimal probability from initial state is %s .' %Value[prod_mdpst.prod_init[0]])

# ------------Simulation and Visualization----------------------------
print("----------------------------------------")
print("||||||||Simulation start||||||||||||||||")
print("----------------------------------------")
total_T = 2000
state_seq = [initial_node, ]
label_seq = [initial_label, ]
N = 3
n = 0
print("Try %s simulations of length %s" % (str(N), str(total_T)))

XX = []
LL = []
UU = []
MM = []
PP = []

while (n < N):
    print('=======simulation %s starts=======' % str(n))
    X, L, U, M, PX = prod_mdpst.execution(motion_mdpst,
        Pre_node, prefix_action, AM_SEC, suffix_action, total_T, state_seq, label_seq)
    # print 'State trajectory: %s' %str(X)
    # print 'Label trajectory: %s' %str(L)
    # print 'Control Actions: %s' %str(U)
    # print 'Marker sequence: %s' %str(M)
    print('=======simulation %s ends=======' % str(n))
    XX.append(X)
    LL.append(L)
    UU.append(U)
    MM.append(M)
    PP.append(PX)
    #run_movie(motion_mdp, WS_d, WS_node_dict, X, L, U, M)
    n += 1

t6 = time.time()
# print('MC simulation done, time: %s' % str(t6-t5))

visualize_world_paths(WS_d, WS_node_dict, XX, LL, UU, MM, 'GFabc')
t7 = time.time()
# print('Visualize paths done, time: %s' %str(t7-t6))

analyze_events(MM, LL)













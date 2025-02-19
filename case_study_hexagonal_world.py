from MDP_TG.MDPST import Motion_MDPST, Product_MDPST, Product_MDPST_ldba, syn_full_plan_mdpst
from MDP_TG.mdp import Motion_MDP_from_MDPST
from MDP_TG.dra import Dra, Product_Dra
from MDP_TG.ldba import LDBA, Product_Ldba
from MDP_TG.compute_WR import compute_WR, compute_WR_dra, compute_valid_prefix_state_dra, compute_valid_prefix_state_ldba
from MDP_TG.create_world import create_world_honeycomb, create_world_grid
from MDP_TG.vis import visualize_run, visualize_world_paths, visualize_world, run_movie, analyze_events
import pickle
import _pickle as pkl

import math
import time

# -------- real example -------
auto_choice = 1  # {1: LDBA; 2:DRA, 3:DFA}
workspace = 1 # {1: Honeycomb World; 2:GridWorld}
workspace_size = [(5, 5), (8, 8), (10, 10)]
WS_d = 1

#----
Model_Const = dict()
Winning_Comp = dict()
Synthesis = dict()
Intial_satisfy = dict()
Num_States_Transitions = dict()
for ex in range(len(workspace_size)):
    for ao in range(2):
        auto_choice = ao+1
        t0 = time.time()
        if workspace == 1:
            num_x = workspace_size[ex][0]
            num_y = workspace_size[ex][1]
            square_root=math.sqrt(3)
            Obstacle = [(7, square_root*4), (5/2, square_root/2), (5/2, square_root*3/2)]
            U=[tuple('1'), tuple('2'), tuple('3'),tuple('4')] #U = [tuple('FR'), tuple('BK'), tuple('TR'), tuple('TL')]
            C = [2, 4, 3, 3, 1]
            WS_node_dict, robot_nodes, robot_edges, set_nodes, robot_state_action, C_dict, P_dict = create_world_honeycomb(num_x, num_y, square_root, Obstacle, U, C)
            initial_node = (1, square_root, 1)
            initial_label = frozenset()
        elif workspace == 2:
            num_x = workspace_size[ex][0]
            num_y = workspace_size[ex][1]
            square_root=math.sqrt(3)
            #Obstacle = [(7, 1), (3, 5)]
            Obstacle = []
            U=[tuple('1'), tuple('2'), tuple('3'),tuple('4')]  #U = [tuple('FR'), tuple('BK'), tuple('TR'), tuple('TL')]
            C = [2, 4, 3, 3, 1]
            WS_node_dict, robot_nodes, robot_edges, set_nodes, robot_state_action, C_dict, P_dict = create_world_grid(num_x, num_y, Obstacle, U, C)
            initial_node = (1, 1, 1)
            initial_label = frozenset()

        t1 = time.time()
        print('create honeycomb world done, time: %s' %str(t1-t0))

        # # ----
        # WS_d = 1
        # visualize_world(WS_d, WS_node_dict, 'world')
        # t1 = time.time()
        # print('visualize world done, time: %s' %str(t1-t0))

        #--------------------------------------------
        #--------construct MDPST---------------------
        motion_mdpst = Motion_MDPST(robot_nodes, robot_edges, robot_state_action, U, C_dict,
                                initial_node, initial_label)

        #-----Build the LDBA/DRA/DFA from LTL or LTLf formula----
        if auto_choice == 1:
            reach_avoid = '(! obstacle) U target'
            persist_reach = 'G F base1 & G F base2 & G F base3'
            persist_reach_avoid = 'G F base1 & G F base2 & G F base3 & G ! obstacle'
            ldba = LDBA(persist_reach_avoid)
            t2 = time.time()
            print('LDBA done, time: %s' % str(t2-t1))
            # ----Build the Product MDPST-----------------
            prod_mdpst = Product_MDPST_ldba(motion_mdpst, ldba, P_dict)
            t3 = time.time()
            print('Product LDBA done, time: %s' % str(t3-t2))
            Num_States_Transitions[((num_x, num_y), auto_choice)] = (
            str(len(prod_mdpst.prod_nodes)), str(len(prod_mdpst.prod_edges.keys())))
        elif auto_choice == 2:
            persist_reach = '& G F base1 & G F base2  G F base3'
            persist_reach_avoid = '& G F base1 & G F base2 & G F base3 G ! obstacle'
            dra = Dra(persist_reach_avoid)
            t2 = time.time()
            print('DRA done, time: %s' % str(t2-t1))
            # ----
            prod_mdpst = Product_MDPST(motion_mdpst, dra, P_dict)  # mdpst with dra
            t3 = time.time()
            print('Product DRA done, time: %s' % str(t3-t2))
            Num_States_Transitions[((num_x, num_y), auto_choice)] = (
            str(len(prod_mdpst.prod_nodes)), str(len(prod_mdpst.prod_edges.keys())))

        model_construction_time = str(t3-t0)

        #####----calculate winning region for Product MDPST----------
        if auto_choice == 1:
            motion_mdp = Motion_MDP_from_MDPST(robot_nodes, robot_edges, U,
                                    initial_node, initial_label)

            prod_mdp = Product_Ldba(motion_mdp, ldba)
            prod_mdp.compute_S_f(set(prod_mdp.nodes()))
            #----
            WR, WR_action = compute_WR(ldba, prod_mdp, prod_mdpst, U)
            #print(WR)
            t4 = time.time()
            print('Compute winning region done, time: %s' % str(t4-t3)) 
            print('Winning region has %s states:' %len(WR))
            Winning_Comp[((num_x, num_y), auto_choice)] = (str(t4-t3), len(WR))
        elif auto_choice == 2:
            motion_mdp = Motion_MDP_from_MDPST(robot_nodes, robot_edges, U,
                                    initial_node, initial_label)

            prod_mdp = Product_Dra(motion_mdp, dra)
            prod_mdp.compute_S_f()   
            #----
            WR, WR_action = compute_WR_dra(dra, prod_mdp, prod_mdpst, U)
            t4 = time.time()
            print('Compute winning region done, time: %s' % str(t4-t3))
            print('Winning region has %s states:' %len(WR))
            Winning_Comp[((num_x, num_y), auto_choice)] = (str(t4-t3), len(WR))

        # #---------------------------------------------------------------
        # #-------------------Strategy synthesis--------------------------
        #----compute valid prefix states
        if auto_choice ==1:
            Sr = compute_valid_prefix_state_ldba(prod_mdp)
            print(len(Sr))
        elif auto_choice == 2:
            Sr = compute_valid_prefix_state_dra(prod_mdp)
            print(len(Sr))

        Pre_Node = Sr.difference(WR)

        if prod_mdpst.prod_init[0] in set(WR):
            print("Initial state is accepting!")
            print('Optimal probability from initial state is 1.')
            t5 = time.time()
        else:
            if auto_choice != 3:
                best_prefix_plan = syn_full_plan_mdpst(prod_mdpst, WR, Pre_Node)
                prefix_action = best_prefix_plan[0]
                #print(prefix_action)
                Value = best_prefix_plan[1]
                #print(Value)
                t5 = time.time()
                print('Strategy synthesis done, time: %s' % str(t5-t3))
                print('Optimal probability from initial state is %s .' %Value[prod_mdpst.prod_init[0]])

        Synthesis_time = str(t5-t3)

        Model_Const[((num_x, num_y), auto_choice)] = model_construction_time
        Synthesis[((num_x, num_y), auto_choice)] = Synthesis_time
        Intial_satisfy[((num_x, num_y), auto_choice)] = Value[prod_mdpst.prod_init[0]]

        
        print("Number of states and transitions of the product mdpst: %s" % Num_States_Transitions)
        #print("Optimal probability from initial state: %s" %Intial_satisfy)
        print("Model construstion time: %s" % Model_Const)
        print("Winning region computation time: %s" % Winning_Comp)
        print("Synthesis time: %s" %Synthesis)

        # ------------Simulation and Visualization----------------------------
        # ----visualize workspace_size = (5, 5) and auto_choince = 1 (LDBA)
        if ex == 0 and auto_choice == 1:
            print("----------------------------------------")
            print("||||||||Simulation start||||||||||||||||")
            print("----------------------------------------")
            total_T = 2000
            state_seq = [initial_node, ]
            label_seq = [initial_label, ]
            N = 5
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
                    Pre_Node, prefix_action, WR, WR_action, total_T, state_seq, label_seq)
                # print('State trajectory: %s' %str(X))
                # print('Label trajectory: %s' %str(L)) 
                # print('Control Actions: %s' %str(U)) 'Control Actions: %s' %str(U)
                # print('Marker sequence: %s' %str(M)) 
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














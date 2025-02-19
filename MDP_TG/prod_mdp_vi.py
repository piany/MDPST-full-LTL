# -*- coding: utf-8 -*-

from math import sqrt
from networkx.classes.digraph import DiGraph
from networkx import strongly_connected_components_recursive,strongly_connected_components
from networkx.algorithms import shortest_path
import numpy as np


# ----------------------------------------------------------------------
#--------------Optimal policy synthesis for MDP--------------------------
def syn_full_plan_mdp(mdp, Acc, SR):
    # ----Optimal plan synthesis, total cost over plan prefix and suffix----
    print("==========[Optimal full plan synthesis start]==========")
    index_prefix, v_new = optimal_plan_prefix_mdp(mdp, Acc, SR)
    plan_prefix = index_prefix
    prefix_risk = 1 - v_new[mdp.init_state] 
    if plan_prefix:
        print("Best plan prefix obtained, risk %s" %str(prefix_risk))
        return plan_prefix, v_new
    else:
        print("No valid plan found")
        return None, None

def optimal_plan_prefix_mdp(mdp, Acc, SR):
    # ----Synthesize optimal plan prefix to reach accepting MEC or SCC----
    sf = Acc
    Sr = set()
    v_old = dict()
    for node in mdp.nodes:
        if node not in sf:
            Sr.add(node)
            v_old[node] = 0
        else:
            v_old[node] = 1
    print('Number of suffix states: %s' %len(sf))
    print('Number of prefix states: %s' %len(SR))
     # ---------solve vi------------
    print('-----')
    print('Value iteration for prefix starts now')
    print('-----')
    num_iteration = 0
    delta_old = 1
    while delta_old >= 0.01:
        print('Number of interation: %s' %num_iteration)
        v_new, index_prefix, delta_new = optimal_prefix_value_iteration_mdp(mdp, SR, v_old)
        for s in SR:                    
            v_old[s] = v_new[s]
        for s in sf:
            v_old[s] = 1
        num_iteration += 1           
        delta_old = delta_new
        print(delta_old)
        if delta_old < 0.01:
            print("Prefix Value iteration completed in interations: %s" %num_iteration)
            print("delta: %s" %delta_new)
            return index_prefix, v_new
                      
    # print("Prefix Value iteration completed in interations: %s" %num_iteration)
    # print("delta: %s" %delta_new)

    # return index_prefix, v_new

def optimal_prefix_value_iteration_mdp(mdp, Sr, v_old):
    num1 = len(Sr)
    U = mdp.U
    v_new = dict()
    num2 = len(U)
    vlist = [[0] * num2 for _ in range(num1)]
    index = dict()
    delta = 0
    #print(vlist)
    for idx, s in enumerate(Sr):
        for idu, u in enumerate(U):
            if (s, u) in mdp.state_action.keys():
                t_set = mdp.state_action[(s, u)]
                #print(len(t_set))
                for k, t_group in enumerate(t_set):
                    if len(t_set) == 1:
                        pe = 1
                    else:
                        pe = mdp.state_action_state[(s, u, t_group)]
                    #print(pe)
                    fd = t_group
                    #print(fd)
                    if len(fd) > 0:
                        for t_node in fd: 
                            ss = tuple(t_node) 
                            vlist[idx][idu] += pe*v_old[ss]/len(fd)            
    #print(vlist)
    for idx, s in enumerate(Sr):
        v_new[s], index[s] =  max((value, index) for index, value in enumerate(vlist[idx]))
        error = abs(v_new[s] - v_old[s])
        if error > delta:
            delta = error
    # print(v_new)
    # print(index) 
    # print(delta)  

    return v_new, index, delta
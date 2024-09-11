from networkx.classes.digraph import DiGraph
from ortools.linear_solver import pywraplp
from math import sqrt
from networkx import single_source_shortest_path
from networkx import strongly_connected_components_recursive,strongly_connected_components
from networkx.algorithms import shortest_path
import random
import time

# --------------------------------
new_var = DiGraph
class Motion_Set_MDP(new_var):
    # ----construct probabilistic-labeled MDP----
    def __init__(self, node, edge_dict, U, initial_node):
        DiGraph.__init__(self, name='motion_set_mdp',
                         init_state=initial_node)
        for n in node:
            self.add_node(n, act=set())
        print("-------Motion Set MDP Initialized-------")
        self.add_edges(edge_dict, U)
        print("%s states and %s edges" %
              (str(len(self.nodes())), str(len(self.edges()))))
        #self.unify_mdp()

    def add_edges(self, edge_dict, U):
        self.graph['U'] = set()
        dummy = []
        num = 0
        for u in U:
            self.graph['U'].add(tuple(u))
        for edge, attri in edge_dict.items():
            f_node = edge[0]
            u = edge[1]
            t_node = edge[2]
            prob_cost = dict()
            prob_cost[tuple(u)] = attri
            self.add_edge(f_node, t_node, prop=prob_cost)
        # ----
        for f_node in self.nodes():
            Us = set()
            for t_node in self.successors(f_node):
                prop = self[f_node][t_node]['prop']
                Us.update(set(prop.keys()))
            if Us:
                self.nodes[f_node]['act'] = Us.copy()
        print("-------Motion Set MDP Constructed-------")

    def unify_mdp(self):
        # ----verify the probability sums up to 1----
        for f_node in self.nodes():
            for u in self.nodes[f_node]['act']:
                sum_prob = 0
                N = 0
                for t_node in self.successors(f_node):
                    prop = self[f_node][t_node]['prop']
                    if u in list(prop.keys()):
                        sum_prob += prop[u][0]
                        N += 1
                if sum_prob < 1.0:
                    to_add = (1.0-sum_prob)/N
                    for t_node in self.successors(f_node):
                        prop = self[f_node][t_node]['prop']
                        if u in list(prop.keys()):
                            prop[u][0] += to_add
                if sum_prob > 1.0:
                    for t_node in self.successors(f_node):
                        prop = self[f_node][t_node]['prop']
                        if u in list(prop.keys()):
                            prop[u][0] = prop[u][0]/sum_prob
        print('Unify Motion Set MDP Done')

#-----------------------------------------------------
def compute_SEC(prod_mdp, prod_mdpst, U):
    Sf = prod_mdp.Sf   
    [T, common, act] = Sf[0][0]

    #----compute valid prefix states
    for init_node in prod_mdp.graph['initial']:
        path_init = single_source_shortest_path(prod_mdp, init_node)
        print('Reachable from init size: %s' % len(list(path_init.keys())))
        if not set(path_init.keys()).intersection(T):
            print("Initial node can not reach sf")
        init_reach = set(path_init.keys())
        # ----find bad states that can not reach MEC
        simple_digraph = DiGraph()
        simple_digraph.add_edges_from(((v, u) for u, v in prod_mdp.edges()))
        path = single_source_shortest_path(
            simple_digraph, random.sample(sorted(common), 1)[0])
        reachable_set = set(path.keys())
        print('States that can reach sf, size: %s' % str(len(reachable_set)))
        Sd = init_reach.difference(reachable_set)
        Sr = init_reach.intersection(reachable_set) #prefix states
        # #--------------
        print('Initial reachable size: %s; Sd inside size: %s; Sr inside size: %s' %
                (len(init_reach), len(Sd), len(Sr)))

    #----construct set-valued transition system
    acc_nodes = T
    acc_action = act
    print(len(acc_nodes))

    prod_set_node_valid = set()
    for fnode in acc_nodes:
        for u in acc_action[fnode]:
            if (fnode, u) in prod_mdpst.prod_state_action.keys():
                tnode_set_set = prod_mdpst.prod_state_action[(fnode, u)]
                for tnode_set in tnode_set_set:
                    #print(tnode_set)
                    t0 = tnode_set[0]
                    t1 = tnode_set[1]
                    t2 = tnode_set[2]
                    if len(t0)>1:
                        result = tuple((s, t1, t2) for s in t0)
                        if set(result).issubset(acc_nodes):
                            prod_set_node_valid.add(tuple(tnode_set))
                            #print("tnode set plus 1!")
    print(len(prod_set_node_valid))
    TS_nodes = acc_nodes.union(prod_set_node_valid)
    print(len(TS_nodes))

    TS_edges = dict()
    for fnode in acc_nodes:
        for u in acc_action[fnode]:
            if (fnode, u) in prod_mdpst.prod_state_action.keys():
                tnode_valid_set = prod_mdpst.prod_state_action[(fnode, u)]
                #print(tnode_valid_set)
                for tnode in tnode_valid_set:
                    if len(tnode[0]) == 1:
                        tnode = (tnode[0][0], tnode[1], tnode[2])
                    #print(tnode)
                    if tnode in TS_nodes: 
                        #print("yes")                   
                        TS_edges[(fnode, u, tnode)] = (1, 1)
    print(len(TS_edges))

    for fnode_set in prod_set_node_valid:
        #print(fnode_set)
        t0 = fnode_set[0]
        t1 = fnode_set[1]
        t2 = fnode_set[2]
        result = tuple((s, t1, t2) for s in t0)
        tnode_set = dict()
        for id, fnode in enumerate(result):
            tnode_set[id] = prod_mdpst.successors[fnode]

        # Convert the dictionary values (tuples) to sets
        sets = [set(states) for states in tnode_set.values()]

        # Find the common elements in all sets
        common_states = set.intersection(*sets)
        if common_states:
            print(f"Common state(s) in all tuples: {common_states}")
        else:
            print("There is no common state in all tuples.")

        for tnode in common_states:
            if len(tnode[0]) == 1:
                tnode = (tnode[0][0], tnode[1], tnode[2])
            if tnode in TS_nodes:
                 TS_edges[(fnode_set, U[0], tnode)] = (1, 1)

    print(len(TS_edges))
    TS_MDP = Motion_Set_MDP(TS_nodes, TS_edges, prod_mdpst.U, prod_mdpst.prod_init)

    # t1 = time.time()
    # print('Transition system MDP done, time: %s' % str(t1))

    TS_MEC, suffix_action = find_MECs(TS_MDP, TS_MDP.nodes())
    #----
    AM_SEC = set()
    SEC_set = set()
    SEC_single = set()
    for ts_mec in TS_MEC:
        if len(ts_mec) > 5:
            for node in ts_mec:
                AM_SEC.add(node)
                t0 = node[0] 
                t1 = node[1]
                t2 = node[2]
                if len(t0) == 2:
                    SEC_set.add(node)
                else:
                    SEC_single.add(node)  
    print(len(AM_SEC))
    print(len(SEC_set))
    print(len(SEC_single))
    #print('Suffix node: %s' %SEC_single)

    #----
    Pre_node = Sr.difference(SEC_single)
    print(len(Pre_node))

    return AM_SEC, SEC_single, Pre_node, suffix_action

#---------------------------------
def find_MECs(mdp, Sneg):
    # ----implementation of Alg.47 P866 of Baier08----
    print('Remaining states size', len(Sneg))
    U = mdp.graph['U']
    A = dict()
    for s in Sneg:
        A[s] = mdp.nodes[s]['act'].copy()
        if not A[s]:
            print("Isolated state")
    MEC = set()
    MECnew = set()
    MECnew.add(frozenset(Sneg))
    # ----
    k = 0
    while MEC != MECnew:
        print("<============iteration %s============>" % k)
        k += 1
        MEC = MECnew
        MECnew = set()
        print("MEC size: %s" % len(MEC))
        print("MECnew size: %s" % len(MECnew))
        for T in MEC:
            R = set()
            T_temp = set(T)
            simple_digraph = DiGraph()
            for s_f in T_temp:
                if s_f not in simple_digraph:
                    simple_digraph.add_node(s_f)
                for s_t in mdp.successors(s_f):
                    if s_t in T_temp:
                        simple_digraph.add_edge(s_f, s_t)
            print("SubGraph of one MEC: %s states and %s edges" % (
                str(len(simple_digraph.nodes())), str(len(simple_digraph.edges()))))
            i = 0
            for Scc in strongly_connected_components_recursive(simple_digraph):
                i += 1
                obsticle=None
                if (len(Scc) >= 1):
                    for s in Scc:
                        U_to_remove = set()
                        for u in A[s]:
                            for t in mdp.successors(s):
                                if ((u in list(mdp[s][t]['prop'].keys())) and (t not in Scc)):
                                    U_to_remove.add(u)
                        A[s].difference_update(U_to_remove)
                        if not A[s]:
                            R.add(s)
                # print(i,'number of sccs',len(Scc))
            while R:
                s = R.pop()
                T_temp.remove(s)
                for f in mdp.predecessors(s):
                    if f in T_temp:
                        A[f].difference_update(
                            set(mdp[f][s]['prop'].keys()))
                        if not A[f]:
                            R.add(f)
            j = 0
            for Scc in strongly_connected_components_recursive(simple_digraph):
                j += 1
                if (len(Scc) >= 1):
                    common = set(Scc).intersection(T_temp)
                    if common:
                        MECnew.add(frozenset(common))
    # ---------------
    print('Final MEC and MECnew size:', len(MEC))
    return MEC, A
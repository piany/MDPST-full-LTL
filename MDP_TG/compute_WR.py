from MDP_TG.MDPST import Motion_MDPST, Motion_Prod_MDPST, Product_MDPST, Product_MDPST_ldba, syn_full_plan_mdpst
from networkx.classes.digraph import DiGraph
from math import sqrt
from networkx import single_source_shortest_path
import random
import time

# --------------------------------
#-----------------------------------------------------
def compute_WR(ldba, prod_mdp, prod_mdpst, U):
    Sf = prod_mdp.Sf   #accepting maximum end component
    [T, common_state, common_tran, act] = Sf[0][0]

    #----construct set-valued transition system
    amec_nodes = T
    valid_acc_nodes = common_state
    amec_transition = common_tran
    
    print(len(amec_nodes))
    print(len(valid_acc_nodes))

    # Compute winning region
    num_iteration = 1
    WR_nodes = dict()
    WR_acc_nodes = dict()
    WR_nodes[1] = amec_nodes
    WR_acc_nodes[1] = valid_acc_nodes
    flag = 1
    while flag == 1 and WR_acc_nodes[num_iteration]:
        node_In = []
        node_Out = []
        for fnode in WR_acc_nodes[num_iteration]:
            fnode_in = (fnode, 1)
            fnode_out = (fnode, 2)
            node_In.append(tuple(fnode_in))
            node_Out.append(tuple(fnode_out))

        TS_nodes = WR_nodes[num_iteration] # include WR_acc_nodes
        TS_nodes = TS_nodes.union(node_In)
        TS_nodes = TS_nodes.union(node_Out)

        acc_action = dict()
        for fnode in TS_nodes:
            if fnode in node_Out:
                fnode_x = fnode[0]
                acc_action[fnode] = act[fnode_x]          
            elif fnode in node_In:
                acc_action[fnode] = tuple('99')
            else:
                acc_action[fnode] = act[fnode]
    

        TS_edges = dict()
        TS_state_action = dict()
        for fnode in TS_nodes:
            if fnode in node_Out:
                for u in acc_action[fnode]:
                    tnode_valid_set = []
                    if (fnode[0], u) in prod_mdpst.prod_state_action.keys():
                        tnode_set_set = prod_mdpst.prod_state_action[(fnode[0], u)]
                        for tnode_set in tnode_set_set:
                            p = prod_mdpst.prod_state_action_state[(fnode[0], u, tnode_set)]
                            ss = (fnode[0][2], fnode[0][1], tnode_set[0][2])
                            if ss in ldba.graph['accept_tran']:
                                ttnode = ()
                                for tnode in tnode_set:
                                    tnode = (tnode, 1)
                                    ttnode += (tuple(tnode),)
                            else:
                                ttnode = tnode_set
                                
                            TS_edges[(fnode, u, ttnode)] = (p, 1)
                            tnode_valid_set.append(ttnode)                       
                        TS_state_action[(fnode, u)] = tnode_valid_set
            elif fnode in node_In:
                u = acc_action[fnode]
                tnode_valid_set_set = []
                tnode_valid_set = (tuple(fnode),)
                tnode_valid_set_set.append(tnode_valid_set)
                TS_edges[(fnode, u, tnode_valid_set)] = (1, 1)
                TS_state_action[(fnode, u)] = tnode_valid_set_set
            else:
                for u in acc_action[fnode]:
                    tnode_valid_set = []
                    if (fnode, u) in prod_mdpst.prod_state_action.keys():
                        tnode_set_set = prod_mdpst.prod_state_action[(fnode, u)]
                        for tnode_set in tnode_set_set:
                            p = prod_mdpst.prod_state_action_state[(fnode, u, tnode_set)]
                            ss = (fnode[2], fnode[1], tnode_set[0][2])
                            if ss in ldba.graph['accept_tran']:
                                ttnode = ()
                                for tnode in tnode_set:
                                    tnode = (tnode, 1)
                                    ttnode += (tuple(tnode),)
                            else:
                                ttnode = tnode_set
                            TS_edges[(fnode, u, ttnode)] = (p, 1)
                            tnode_valid_set.append(ttnode)                       
                        TS_state_action[(fnode, u)] = tnode_valid_set
        print(len(TS_nodes))
        print(len(TS_edges))
        print(len(TS_state_action))

        TS_MDPST = Motion_Prod_MDPST(TS_nodes, TS_edges, TS_state_action, prod_mdpst.U)

        Pre_node = TS_nodes.difference(node_In)
        best_plan = syn_full_plan_mdpst(TS_MDPST, node_In, Pre_node)
        WR_action = best_plan[0]
        Value = best_plan[1]

        num_iteration += 1
        WR_nodes[num_iteration] = set()
        WR_acc_nodes[num_iteration] =  set()
        count = 0
        for fnode in Pre_node:
            print(Value[fnode])
            if fnode in node_Out:
                if Value[fnode] >= 0.95:
                    WR_nodes[num_iteration].add(fnode[0])
                    WR_acc_nodes[num_iteration].add(fnode[0])                    
                else:
                    count += 1
            else:
                if Value[fnode] >= 0.95:
                    WR_nodes[num_iteration].add(fnode)
        
        print(count)
        if count == 0:
            flag = 0

    WR =  WR_nodes[num_iteration]

    return WR, WR_action

#---------------------------------
def compute_WR_dra(dra, prod_mdp, prod_mdpst, U):
    Sf = prod_mdp.Sf   
    [T, common, act] = Sf[0][0]
    amec_nodes = T
    valid_acc_nodes = common
    print(len(amec_nodes))
    print(len(valid_acc_nodes))
   
    # Compute winning region
    num_iteration = 1
    WR_nodes = dict()
    WR_acc_nodes = dict()
    WR_nodes[1] = amec_nodes
    WR_acc_nodes[1] = valid_acc_nodes
    flag = 1
    while flag == 1 and WR_acc_nodes[num_iteration]:
        node_In = []
        node_Out = []
        for fnode in WR_acc_nodes[num_iteration]:
            fnode_in = (fnode, 1)
            fnode_out = (fnode, 2)
            node_In.append(tuple(fnode_in))
            node_Out.append(tuple(fnode_out))

        print(node_In)

        TS_nodes = WR_nodes[num_iteration].difference(WR_acc_nodes[num_iteration]) # exclude WR_acc_nodes
        TS_nodes = TS_nodes.union(node_In)
        TS_nodes = TS_nodes.union(node_Out)

        acc_action = dict()
        for fnode in TS_nodes:
            if fnode in node_Out:
                fnode_x = fnode[0]
                acc_action[fnode] = act[fnode_x]          
            elif fnode in node_In:
                acc_action[fnode] = tuple('99')
            else:
                acc_action[fnode] = act[fnode]
    

        TS_edges = dict()
        TS_state_action = dict()
        for fnode in TS_nodes:
            if fnode in node_Out:
                for u in acc_action[fnode]:
                    tnode_valid_set = []
                    if (fnode[0], u) in prod_mdpst.prod_state_action.keys():
                        tnode_set_set = prod_mdpst.prod_state_action[(fnode[0], u)]
                        for tnode_set in tnode_set_set:
                            p = prod_mdpst.prod_state_action_state[(fnode[0], u, tnode_set)]
                            if tnode_set[0][2] in dra.graph['accept'][0][0]:
                                ttnode = ()
                                for tnode in tnode_set:
                                    tnode = (tnode, 1)
                                    ttnode += (tuple(tnode),)
                            else:
                                ttnode = tnode_set
                            print(ttnode)
                            TS_edges[(fnode, u, ttnode)] = (p, 1)
                            tnode_valid_set.append(ttnode)                       
                        TS_state_action[(fnode, u)] = tnode_valid_set
            elif fnode in node_In:
                u = acc_action[fnode]
                tnode_valid_set_set = []
                tnode_valid_set = (tuple(fnode),)
                tnode_valid_set_set.append(tnode_valid_set)
                TS_edges[(fnode, u, tnode_valid_set)] = (1, 1)
                TS_state_action[(fnode, u)] = tnode_valid_set_set
            else:
                for u in acc_action[fnode]:
                    tnode_valid_set = []
                    if (fnode, u) in prod_mdpst.prod_state_action.keys():
                        tnode_set_set = prod_mdpst.prod_state_action[(fnode, u)]
                        for tnode_set in tnode_set_set:
                            p = prod_mdpst.prod_state_action_state[(fnode, u, tnode_set)]
                            if tnode_set[0][2] in dra.graph['accept'][0][0]:
                                ttnode = ()
                                for tnode in tnode_set:
                                    tnode = (tnode, 1)
                                    ttnode += (tuple(tnode),)
                            else:
                                ttnode = tnode_set
                            
                            TS_edges[(fnode, u, ttnode)] = (p, 1)
                            tnode_valid_set.append(ttnode)                       
                        TS_state_action[(fnode, u)] = tnode_valid_set
        print(len(TS_nodes))
        print(len(TS_edges))
        print(len(TS_state_action))

        TS_MDPST = Motion_Prod_MDPST(TS_nodes, TS_edges, TS_state_action, prod_mdpst.U)

        Pre_node = TS_nodes.difference(node_In)
        best_plan = syn_full_plan_mdpst(TS_MDPST, node_In, Pre_node)
        WR_action = best_plan[0]
        Value = best_plan[1]

        num_iteration += 1
        WR_nodes[num_iteration] = set()
        WR_acc_nodes[num_iteration] =  set()
        count = 0
        for fnode in Pre_node:
            # print(fnode)
            # print(Value[fnode])
            if fnode in node_Out:
                if Value[fnode] >= 0.95:
                    WR_nodes[num_iteration].add(fnode[0])
                    WR_acc_nodes[num_iteration].add(fnode[0])                    
                else:
                    count += 1
            else:
                if Value[fnode] >= 0.95:
                    WR_nodes[num_iteration].add(fnode)
        
        print(count)
        if count == 0:
            flag = 0

    WR =  WR_nodes[num_iteration]

    return WR, WR_action


#---------------------------------
def compute_valid_prefix_state_ldba(prod_mdp):
    Sf = prod_mdp.Sf   #accepting maximum end component
    [T, common_state, common_tran, act] = Sf[0][0]
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
            simple_digraph, random.sample(sorted(common_state), 1)[0])
        reachable_set = set(path.keys())
        print('States that can reach sf, size: %s' % str(len(reachable_set)))
        Sd = init_reach.difference(reachable_set)
        Sr = init_reach.intersection(reachable_set) #prefix states
        # #--------------
        print('Initial reachable size: %s; Sd inside size: %s; Sr inside size: %s' %
                (len(init_reach), len(Sd), len(Sr)))
        
    return Sr
    
def compute_valid_prefix_state_dra(prod_mdp):
    Sf = prod_mdp.Sf   #accepting maximum end component
    [T, common, act] = Sf[0][0]
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
        
    return Sr

#------------------------------------------------------- 
#-------------------------------------------------------   
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
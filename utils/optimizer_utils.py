import numpy as np
import random
import math
import copy

from itertools import combinations
from numpy import random as npr
from operator import attrgetter
from timeit import default_timer as timer

"""
Modification notes:
- inverse was removed from both the instnace and the operators, because in the examined cases there are no inverse tasks
- if the TC of the solutions are equal, then choose the one which has the shortest longest route plan
- the sub-route plan operator was fixed
    -> the route plan was treated as a line, but it's a circle (tour)
    -> fixed the sub-route plan rotation method
        -> select the top 5 nearest neighbours from ALL the tasks in the route plan 
            not just the ones before/after sr_1/sr_2 in the (line) route plan
    -> transforms the new_route(s) to normal route plan(s) (since 0 or vt_id may not be the first task in them)
"""

inf = 2^31-1

# HMA
alpha = 0.6

class Arc:
    
    def __init__(self, head_node, tail_node, trav_cost):
        self.head_node = head_node
        self.tail_node = tail_node
        self.trav_cost = trav_cost
    
    def __str__(self):
        return "("+str(self.head_node)+","+str(self.tail_node)+")\ttc:"+str(self.trav_cost)

    def __repr__(self):
        return "("+str(self.head_node)+","+str(self.tail_node)+")\tc:"+str(self.trav_cost)


class Task(Arc):
    
    def __init__(self, id, arc_id, serv_cost, demand):
        self.id = id                     # IDindex
        self.arc_id = arc_id
        self.serv_cost = serv_cost
        self.demand = demand

    def __str__(self):
        return str(self.id)+"\t("+str(self.head_node)+","+str(self.tail_node)+")\ttc: "+str(self.trav_cost) \
            +"\tsc: "+str(self.serv_cost)+"\td: "+str(self.demand)+"\n"

    def __repr__(self):
        return str(self.id)+"\t("+str(self.head_node)+","+str(self.tail_node)+")\ttc: "+str(self.trav_cost) \
            +"\tsc: "+str(self.serv_cost)+"\td: "+str(self.demand)+"\n"


class Individual:

    def __init__(self):
        self.index = 0
        self.sequence = None
        self.route_seg = None         # (start_id, end_id)
        self.route_seg_load = None
        self.total_cost = inf
        self.tc_route_max = inf       # the cost of the most expensive route plan in the solution
        self.age = 0

class Instance:
    
    def __init__(self):
        self.name = None
        self.vertex_num = 0
        self.req_edge_num = 0
        self.nonreq_edge_num = 0
        self.vehicle_num = 0
        self.capacity = 0
        self.depot = None
        
        self.solution_lb = 0
        self.task_num = 0
        self.total_arc_num = 0
        
        self.virtual_task_ids = dict() # for every route plan it contains its current virtual task id
        self.not_allowed_vt_ids = set()
        self.task_ids = set()  # tasks that still need to be served
        
        self.tasks = None
        self.arcs = dict()      # was None before
        
        self.finished_routes = set()
        self.free_vehicles = set()
        self.route_to_vehicle = None
        self.last_processed_event_id = -1
        
        self.start_time = 0
    

    def ind_from_seq(self, seq):
        ind = Individual()
        ind.sequence = seq
        ind.route_seg = self.find_route_segments(ind.sequence)
        ind.route_seg_load = self.calculate_route_segments_load(ind.sequence, ind.route_seg)
        ind.total_cost, ind.tc_route_max = self.calculate_tc(ind.sequence)
        return ind

    
    def fix_sequence(self, seq):
        fixed_seq = list()
        for x in range(len(seq)-1):
            if seq[x] == 0 and seq[x+1] != 0:
                fixed_seq.append(0)
            elif seq[x] != 0:
                fixed_seq.append(seq[x])
        fixed_seq.append(seq[-1])
        if fixed_seq[-1] != 0:
            fixed_seq.append(0)
        return fixed_seq
    
    
    # construct route plan that contains only the the task arcs
    def construct_routing_plan(self, seq):
        routing_plan = list()
        for i in range(len(seq)):
            if seq[i] != 0:
                routing_plan.append( (self.tasks[seq[i]].head_node, self.tasks[seq[i]].tail_node) )
        return routing_plan      

    
    # construct route plan that includes the non task arcs, too
    def construct_whole_routing_plan(self, seq):
        routing_plan = list()
        for i in range(len(seq)-1):
            for j in range(1,self.shortest_path[self.tasks[seq[i]].tail_node][self.tasks[seq[i+1]].head_node][0]+1):
                routing_plan.append(self.shortest_path[self.tasks[seq[i]].tail_node][self.tasks[seq[i+1]].head_node][j])
        return routing_plan
    

    def find_arc_id(self, head_node, tail_node):
        return next((arc_id for arc_id, arc in enumerate(self.arcs) \
                if arc.head_node == head_node and arc.tail_node == tail_node), -1)
    
    
    def find_task_id(self, head_node, tail_node):
        return next((task_id for task_id, task in enumerate(self.tasks) \
                if task != None and task.head_node == head_node and task.tail_node == tail_node), -1)
    

    def get_vehicle_from_vt(self, vt_index):
        for v_id, vt_id in self.virtual_task_ids.items():
            if vt_id == vt_index:
                return v_id
        return None

    # find the route segments (i.e., the start and end tasks of each route plan)
    def find_route_segments(self, s):
        route_seg = []
        i = 0
        while i < len(s)-1:
            start = i
            i += 1
            while s[i] != 0:
                i +=1
            end = i
            route_seg.append((start, end))
        return(route_seg)
    
    
    # calculate the load of the route segments
    def calculate_route_segments_load(self, s, route_seg):
        route_seg_load = []
        for r in route_seg:
            load = 0
            for i in range(r[0]+1,r[1]):
                load += self.tasks[s[i]].demand
            route_seg_load.append(load)
        return(route_seg_load)
    
    
    # for IDP and to check if a solution is feasible
    def calculate_excess_demand(self, route_seg_load):
        excess_demand = 0
        for load in route_seg_load:
            if load > self.capacity:
                excess_demand += (load - self.capacity)
        return excess_demand
    
    
    # calculate the total cost of a solution
    def calculate_tc(self, s):
        total_cost = 0
        tc_route_max = 0
        tc_route = 0
        for i in range(len(s)-1):
            i_min_cost = self.min_cost[self.tasks[s[i]].tail_node][self.tasks[s[i+1]].head_node]
            if i_min_cost == inf:
                return inf, inf
            tc_route += (self.tasks[s[i]].serv_cost + i_min_cost)
            if s[i+1] == 0:
                total_cost += tc_route
                tc_route_max = max(tc_route, tc_route_max)
                tc_route = 0
            #if the vt isn't the first task of a route plan then return inf
            if s[i] in self.virtual_task_ids.values() and s[i-1] != 0:
                return inf, inf
        return total_cost, tc_route_max
    

    def get_route_segment_index(self, route_seg, index):
        for i, r in enumerate(route_seg):
            if index < r[1]:
                return i
    
    
    def get_task_ids_from_sequence(self, seq):
        task_ids = set(seq)
        task_ids.remove(0)
        return list(task_ids)


    def get_best_solution(self, ind_list):
        min_tc = min([x.total_cost for x in ind_list])
        min_tc_inds = [x for x in ind_list if x.total_cost == min_tc]
        if len(min_tc_inds) == 1:
            return min_tc_inds[0]
        else:
            return min(min_tc_inds, key=attrgetter('tc_route_max'))
    
    def is_newer_ind_better(self, ind_old, ind_new):
        if ind_new.total_cost < ind_old.total_cost or \
            (ind_new.total_cost == ind_old.total_cost and \
            ind_new.tc_route_max < ind_old.tc_route_max):
            return True
        else:
            return False

    
# ----------------------------- generate solution ----------------------------------

    def random_routing_plan_generator(self, task_ids):
        routes = list()
        route_demand = list()
        for vt_id in self.virtual_task_ids.values():
            routes.append([vt_id])
            route_demand.append(self.tasks[vt_id].demand)

        task_seq = npr.permutation(list(task_ids))
        for i in range(len(task_seq)):
            task_added = False
            for j in range(len(routes)):
                if (route_demand[j] + self.tasks[task_seq[i]].demand <= self.capacity):
                    routes[j].append(task_seq[i])
                    route_demand[j] += self.tasks[task_seq[i]].demand
                    task_added = True
                    break
            if not task_added:
                routes.append([task_seq[i]])
                route_demand.append(self.tasks[task_seq[i]].demand)

        ind_seq = [0]
        for r in routes:
            ind_seq.extend(r)
            ind_seq.append(0)

        ind = Individual()
        ind.sequence = ind_seq
        ind.route_seg = self.find_route_segments(ind.sequence)
        ind.route_seg_load = self.calculate_route_segments_load(ind.sequence, ind.route_seg)       
        ind.total_cost, ind.tc_route_max = self.calculate_tc(ind.sequence)
        ind.age = 0

        return ind

    
    def randomized_path_scanning_heuristic(self, task_ids, vt_ids):
        sequences = [[0] for i in range(5)]
        inds = [0 for i in range(5)]
        
        for i in range(5):
            unserved_task_ids = copy.deepcopy(task_ids)
            unserved_vt_ids = copy.deepcopy(vt_ids)
            current_route_load = 0
            if len(unserved_vt_ids) > 0:
                vt_id = unserved_vt_ids.pop(0)
                sequences[i].append(vt_id)
                current_route_load = self.tasks[vt_id].demand
            while len(unserved_task_ids) > 0:
                servable_task_ids = list(filter(lambda task_id: \
                    current_route_load + self.tasks[task_id].demand <= self.capacity, unserved_task_ids))
                if len(servable_task_ids) == 0:
                    sequences[i].append(0)
                    if len(unserved_vt_ids) > 0:
                        vt_id = unserved_vt_ids.pop(0)
                        sequences[i].append(vt_id)
                        current_route_load = self.tasks[vt_id].demand
                    else:                     
                        current_route_load = 0
                elif len(servable_task_ids) == 1:
                    sequences[i].append(servable_task_ids[0])
                    unserved_task_ids.remove(servable_task_ids[0])
                else:
                    min_cost_task_ids = []
                    min_cost = inf
                    for task_id in servable_task_ids:
                        cost = self.min_cost[self.tasks[sequences[i][-1]].tail_node][self.tasks[task_id].head_node]
                        if cost < min_cost:
                            min_cost = cost
                            min_cost_task_ids = [task_id]
                        if cost == min_cost:
                            min_cost_task_ids.append(task_id)
                    if len(min_cost_task_ids) == 1:
                        sequences[i].append(min_cost_task_ids[0])
                        current_route_load += self.tasks[min_cost_task_ids[0]].demand
                        unserved_task_ids.remove(min_cost_task_ids[0])                 
                    else:
                        best_value_task_ids = []
                        best_value = None
                        # 1) maximize the distance from the head of task to the depot
                        if i == 0:
                            best_value = 0
                            for task_id in min_cost_task_ids:
                                value = self.min_cost[self.tasks[task_id].head_node][self.tasks[0].head_node]
                                if best_value != None and value > best_value:
                                    best_value = value
                                    best_value_task_ids = [task_id]
                                elif value == best_value:
                                    best_value_task_ids.append(task_id)
                        # 2) minimize the distance from the head of task to the depot
                        if i == 1:
                            best_value = inf
                            for task_id in min_cost_task_ids:
                                value = self.min_cost[self.tasks[task_id].head_node][self.tasks[0].head_node]
                                if best_value != None and value < best_value:
                                    best_value = value
                                    best_value_task_ids = [task_id]
                                elif value == best_value:
                                    best_value_task_ids.append(task_id)
                        # 3) maximize the term dem(t)/sc(t)
                        if i == 2:
                            best_value = 0
                            for task_id in min_cost_task_ids:
                                value = self.tasks[task_id].demand / self.tasks[task_id].serv_cost \
                                    if self.tasks[task_id].serv_cost != 0\
                                    else self.tasks[task_id].demand
                                if best_value != None and value > best_value:
                                    best_value = value
                                    best_value_task_ids = [task_id]
                                elif value == best_value:
                                    best_value_task_ids.append(task_id)                        
                        # 4) minimize the term dem(t)/sc(t)
                        if i == 3:
                            best_value = inf
                            for task_id in min_cost_task_ids:
                                value = self.tasks[task_id].demand / self.tasks[task_id].serv_cost \
                                    if self.tasks[task_id].serv_cost != 0\
                                    else self.tasks[task_id].demand
                                if best_value != None and value < best_value:
                                    best_value = value
                                    best_value_task_ids = [task_id]
                                elif value == best_value:
                                    best_value_task_ids.append(task_id)   
                        # 5) use rule 1) if the vehicle is less than halffull, otherwise use rule 2)
                        if i == 4:
                            if current_route_load < self.capacity / 2:
                                best_value = 0
                                for task_id in min_cost_task_ids:
                                    value = self.min_cost[self.tasks[task_id].head_node][self.tasks[0].head_node]
                                    if best_value != None and value > best_value:
                                        best_value = value
                                        best_value_task_ids = [task_id]
                                    elif value == best_value:
                                        best_value_task_ids.append(task_id)                                
                            else:
                                best_value = inf
                                for task_id in min_cost_task_ids:
                                    value = self.min_cost[self.tasks[task_id].head_node][self.tasks[0].head_node]
                                    if best_value != None and value < best_value:
                                        best_value = value
                                        best_value_task_ids = [task_id]
                                    elif value == best_value:
                                        best_value_task_ids.append(task_id)                                
                        
                        next_task_id = None
                        if len(best_value_task_ids) > 1:                       
                            next_task_id = random.choice(min_cost_task_ids)
                        else:
                            next_task_id = min_cost_task_ids[0]
                        sequences[i].append(next_task_id)
                        current_route_load += self.tasks[next_task_id].demand
                        unserved_task_ids.remove(next_task_id)
            sequences[i].append(0)

            while len(unserved_vt_ids) > 0:
                vt_id = unserved_vt_ids.pop(0)
                sequences[i] += [vt_id, 0]
        
        for i in range(5):
            inds[i] = Individual()
            inds[i].sequence = sequences[i]
            inds[i].route_seg = self.find_route_segments(inds[i].sequence)
            inds[i].route_seg_load = self.calculate_route_segments_load(inds[i].sequence, inds[i].route_seg)       
            inds[i].total_cost, inds[i].tc_route_max = self.calculate_tc(inds[i].sequence)
            #print(inds[i].sequence, inds[i].total_cost)
        
        best_ind = self.get_best_solution(inds)
        
        return best_ind
    
    
# ----------------------------------- search operators ----------------------------------

    def insert(self, ind, task_id_1=None, task_id_2=None, only_feasible=True):
        # for inserting task_id_1 before task_id_2
        ind_new1 = copy.deepcopy(ind)   # for case 1: e.g., (a,b),(c,d)
        # for inserting task_id_1 after task_id_2
        ind_new2 = copy.deepcopy(ind)   # for case 3: e.g., (c,d),(a,b)
        
        task_ids = copy.deepcopy(self.task_ids)

        # select task1
        if task_id_1 == None:            
            task_id_1 = random.sample(task_ids, k=1)[0]
        x = ind.sequence.index(task_id_1)
        
        rs_index = self.get_route_segment_index(ind.route_seg, x)
        
        # remove task1 from its original position
        del(ind_new1.sequence[x])
        
        # if it was the only task in the route plan remove the excess 0
        if ind.route_seg[rs_index][1] - ind.route_seg[rs_index][0] == 2:
            del(ind_new1.sequence[x])

        # insert task1 before/after task2
        ind_new2 = copy.deepcopy(ind_new1)
        if task_id_2 == None:
            task_ids.remove(task_id_1)
            task_id_2 = random.sample(task_ids, k=1)[0]
        x_new = ind_new1.sequence.index(task_id_2)
        ind_new1.sequence.insert(x_new, task_id_1)
        ind_new2.sequence.insert(x_new+1, task_id_1)
        
        # insertion into a new route plan is impossible if the task can be inserted only right before/after another task
        try:
            self.find_route_segments(ind_new1.sequence)
        except:
            print("insert error")
            print(ind.sequence, task_id_1, task_id_2)
            print(ind_new1.sequence)
            
            ind_new1.sequence = self.fix_sequence(ind_new1.sequence)
            ind_new2.sequence = self.fix_sequence(ind_new2.sequence)

        ind_new1.route_seg = self.find_route_segments(ind_new1.sequence)
        ind_new1.route_seg_load = self.calculate_route_segments_load(ind_new1.sequence, ind_new1.route_seg)
        ind_new1.total_cost, ind_new1.tc_route_max = self.calculate_tc(ind_new1.sequence)
        if only_feasible and self.calculate_excess_demand(ind_new1.route_seg_load)>0:
            return ind

        ind_new2.route_seg = copy.deepcopy(ind_new1.route_seg)
        ind_new2.route_seg_load = copy.deepcopy(ind_new1.route_seg_load)
        ind_new2.total_cost, ind_new2.tc_route_max = self.calculate_tc(ind_new2.sequence)

        # return the solution with the lowest total cost
        return self.get_best_solution([ind_new1, ind_new2])
    
    
    def double_insert(self, ind, task_id_1=None, task_id_2=None, only_feasible=True):
        # for inserting task_id_1 and the next task before task_id_2
        ind_new1 = copy.deepcopy(ind)   # for case 1: (a,b),(c,d)
        # for inserting task_id_1 and the next task after task_id_2
        ind_new2 = copy.deepcopy(ind)   # for case 5: (a,b),(c,d)
        
        task_ids = copy.deepcopy(self.task_ids)
        
        # select task1 and next task as the task sequence (len = 2)
        if task_id_1 == None:
            task_id_1 = random.sample(task_ids, k=1)[0]
        x = ind.sequence.index(task_id_1)
        tasks = ind.sequence[x:x+2]

        # if the next task is the depot or the 2nd given taskid is the same as the id of the 2nd task in the sequence
        # then the operation cannot be executed
        if tasks[1] == 0 or tasks[1] == task_id_2:
            return ind

        rs_index = self.get_route_segment_index(ind.route_seg, x)

        # remove the tasks from their original positions
        del(ind_new1.sequence[x])
        del(ind_new1.sequence[x])
        
        # if they were the only tasks in the route plan remove the excess 0
        if ind.route_seg[rs_index][1] - ind.route_seg[rs_index][0] == 3:
            del(ind_new1.sequence[x])
        
        # insert the tasks before/after task2
        ind_new2 = copy.deepcopy(ind_new1)
        if task_id_2 == None:
            task_ids.remove(tasks[0])
            task_ids.remove(tasks[1])
            task_id_2 = random.sample(task_ids, k=1)[0]
        x_new = ind_new1.sequence.index(task_id_2)
        ind_new1.sequence[x_new:x_new] = tasks
        ind_new2.sequence[x_new+1:x_new+1] = tasks
        
        # insertion into a new route plan is impossible if the task can be inserted only right before/after another task

        ind_new1.route_seg = self.find_route_segments(ind_new1.sequence)
        ind_new1.route_seg_load = self.calculate_route_segments_load(ind_new1.sequence, ind_new1.route_seg)
        ind_new1.total_cost, ind_new1.tc_route_max = self.calculate_tc(ind_new1.sequence)
        if only_feasible and self.calculate_excess_demand(ind_new1.route_seg_load)>0:
            return ind

        ind_new2.route_seg = copy.deepcopy(ind_new1.route_seg)
        ind_new2.route_seg_load = copy.deepcopy(ind_new1.route_seg_load)
        ind_new2.total_cost, ind_new2.tc_route_max = self.calculate_tc(ind_new2.sequence)

        # return the solution with the lowest total cost
        return self.get_best_solution([ind_new1, ind_new2])
    
    
    def swap(self, ind, task_id_1=None, task_id_2=None, only_feasible=True):
        ind_new = copy.deepcopy(ind)
        
        task_ids = copy.deepcopy(self.task_ids)
        
        if task_id_1 == None:
            task_id_1 = random.sample(task_ids, k=1)[0]      
        x = ind.sequence.index(task_id_1)
        
        if task_id_2 == None:
            task_ids.remove(task_id_1)
            task_id_2 = random.sample(task_ids, k=1)[0] 
        y = ind.sequence.index(task_id_2)
        
        if ind.sequence[x] != ind.sequence[y]:
            ind_new.sequence[x] = copy.deepcopy(ind.sequence[y])
            ind_new.sequence[y] = copy.deepcopy(ind.sequence[x])
        else:
            return ind
        
        try:
            self.find_route_segments(ind_new.sequence)
        except:
            print("swap error")
            print(ind.sequence, task_id_1, task_id_2)
            print(ind_new.sequence)
            
            ind_new.sequence = self.fix_sequence(ind_new.sequence)
            ind_new.route_seg = self.find_route_segments(ind_new.sequence)
        
        ind_new.route_seg_load = self.calculate_route_segments_load(ind_new.sequence, ind_new.route_seg)
        ind_new.total_cost, ind_new.tc_route_max = self.calculate_tc(ind_new.sequence)
        if only_feasible and self.calculate_excess_demand(ind_new.route_seg_load)>0:
            return ind

        return ind_new
        
    
    def two_opt(self, ind, task_id_1=None, task_id_2=None, only_feasible=True):
        task_ids = copy.deepcopy(self.task_ids)
        
        if task_id_1 == None:
            task_id_1 = random.sample(task_ids, k=1)[0]   
        x = ind.sequence.index(task_id_1)    
        x_rid = self.get_route_segment_index(ind.route_seg, x)
        
        if task_id_2 == None:
            task_ids.remove(task_id_1)
            task_id_2 = random.sample(task_ids, k=1)[0] 
        y = ind.sequence.index(task_id_2)
        y_rid = self.get_route_segment_index(ind.route_seg, y)
        
        # one route
        if x_rid == y_rid:
            ind_new = copy.deepcopy(ind)
            
            # normally cannot happen
            if x == y:
                return ind
            
            # if y is before x
            if y < x:
                sub_seq = ind_new.sequence[y:x+1]
                sub_seq.reverse()
                ind_new.sequence[y:x+1] = sub_seq
            # if x is before y
            else:
                sub_seq = ind_new.sequence[x:y+1]
                sub_seq.reverse()
                ind_new.sequence[x:y+1] = sub_seq
            
            ind_new.total_cost, ind_new.tc_route_max = self.calculate_tc(ind_new.sequence)
            
            return ind_new
            
        # two routes
        else:
            ind_new1 = copy.deepcopy(ind)
            ind_new2 = copy.deepcopy(ind)
            
            # to include the delimiter task on the left side seq
            x = x+1
            y = y+1
            
            x_seq_left = ind.sequence[ind.route_seg[x_rid][0]+1:x]            
            x_seq_right = ind.sequence[x:ind.route_seg[x_rid][1]]
            y_seq_left = ind.sequence[ind.route_seg[y_rid][0]+1:y]
            y_seq_right = ind.sequence[y:ind.route_seg[y_rid][1]]
            
            x_seq_right_rev = ind.sequence[x:ind.route_seg[x_rid][1]]
            x_seq_right_rev.reverse()
            
            y_seq_left_rev = ind.sequence[ind.route_seg[y_rid][0]+1:y]
            y_seq_left_rev.reverse()
            
            x_rid_seq1 = x_seq_left + y_seq_right            
            y_rid_seq1 = y_seq_left + x_seq_right
            if y_rid < x_rid:          
                if (len(x_rid_seq1) != 0):
                    ind_new1.sequence[ind.route_seg[x_rid][0]+1:ind.route_seg[x_rid][1]] = x_rid_seq1
                else:
                    del(ind_new1.sequence[ind.route_seg[x_rid][0]:ind.route_seg[x_rid][1]])
                if (len(y_rid_seq1) != 0):
                    ind_new1.sequence[ind.route_seg[y_rid][0]+1:ind.route_seg[y_rid][1]] = y_rid_seq1
                else:
                    del(ind_new1.sequence[ind.route_seg[y_rid][0]:ind.route_seg[y_rid][1]])
            else:
                if (len(y_rid_seq1) != 0):
                    ind_new1.sequence[ind.route_seg[y_rid][0]+1:ind.route_seg[y_rid][1]] = y_rid_seq1
                else:
                    del(ind_new1.sequence[ind.route_seg[y_rid][0]:ind.route_seg[y_rid][1]])
                if (len(x_rid_seq1) != 0):
                    ind_new1.sequence[ind.route_seg[x_rid][0]+1:ind.route_seg[x_rid][1]] = x_rid_seq1
                else:
                    del(ind_new1.sequence[ind.route_seg[x_rid][0]:ind.route_seg[x_rid][1]])                
            
            # originally not in HMA!
            x_rid_seq2 = x_seq_left + y_seq_left_rev            
            y_rid_seq2 = x_seq_right_rev + y_seq_right
            if y_rid < x_rid: 
                if (len(x_rid_seq2) != 0):
                    ind_new2.sequence[ind.route_seg[x_rid][0]+1:ind.route_seg[x_rid][1]] = x_rid_seq2
                else:
                    del(ind_new2.sequence[ind.route_seg[x_rid][0]:ind.route_seg[x_rid][1]])
                if (len(y_rid_seq2) != 0):
                    ind_new2.sequence[ind.route_seg[y_rid][0]+1:ind.route_seg[y_rid][1]] = y_rid_seq2
                else:
                    del(ind_new2.sequence[ind.route_seg[y_rid][0]:ind.route_seg[y_rid][1]])
            else:
                if (len(y_rid_seq2) != 0):
                    ind_new2.sequence[ind.route_seg[y_rid][0]+1:ind.route_seg[y_rid][1]] = y_rid_seq2
                else:
                    del(ind_new2.sequence[ind.route_seg[y_rid][0]:ind.route_seg[y_rid][1]])
                if (len(x_rid_seq2) != 0):
                    ind_new2.sequence[ind.route_seg[x_rid][0]+1:ind.route_seg[x_rid][1]] = x_rid_seq2
                else:
                    del(ind_new2.sequence[ind.route_seg[x_rid][0]:ind.route_seg[x_rid][1]])                
            
            
            try:
                self.find_route_segments(ind_new1.sequence)
            except:
                print("2-opt error")
                print(ind.sequence, task_id_1, task_id_2)
                
                print(x_seq_left, x_seq_right, y_seq_left, y_seq_right)
                print(x_seq_right_rev, y_seq_left_rev)
                
                print(ind_new1.sequence)
                print(ind_new2.sequence)
                
                for ind_new in [ind_new1, ind_new2]:
                    ind_new.sequence = self.fix_sequence(ind_new.sequence)
            
            
            ind_new1.route_seg = self.find_route_segments(ind_new1.sequence)
            ind_new1.route_seg_load = self.calculate_route_segments_load(ind_new1.sequence, ind_new1.route_seg)
            if only_feasible and self.calculate_excess_demand(ind_new1.route_seg_load)>0:
                ind_new1 = Individual()
            else:
                ind_new1.total_cost, ind_new1.tc_route_max = self.calculate_tc(ind_new1.sequence)
            
            ind_new2.route_seg = self.find_route_segments(ind_new2.sequence)
            ind_new2.route_seg_load = self.calculate_route_segments_load(ind_new2.sequence, ind_new2.route_seg)
            if only_feasible and self.calculate_excess_demand(ind_new2.route_seg_load)>0:
                ind_new2 = Individual()
            else:
                ind_new2.total_cost, ind_new2.tc_route_max = self.calculate_tc(ind_new2.sequence)
            
            ind_new = self.get_best_solution([ind_new1, ind_new2])

            if ind_new.total_cost == inf:
                return ind
            else:
                return ind_new
    
    def greedy_sub_tour_mutation(self, ind, selected_route_id=None):
        if selected_route_id == None:
            selected_route_id = random.randint(0,len(ind.route_seg)-1)
        
        vt_id = ind.sequence[ind.route_seg[selected_route_id][0]+1] \
            if ind.sequence[ind.route_seg[selected_route_id][0]+1] in self.virtual_task_ids.values() else 0

        # route without the last 0 (if there is a virtual task, then the first 0 is removed too)
        # route = [vt_id, t_1, t_2, ..., t_n] OR [0, t_1, t_2, ..., t_n]
        route = ind.sequence[ind.route_seg[selected_route_id][0]:ind.route_seg[selected_route_id][1]] if vt_id == 0 \
                else ind.sequence[ind.route_seg[selected_route_id][0]+1:ind.route_seg[selected_route_id][1]]
        
        if len(route) < 4:
            return ind
        
        # fix values
        l_min = 2
        l_max = max(4, int(math.sqrt(len(route))))
        p_rc = 0.5
        p_cp = 0.8
        p_l = 0.2
        nl_max = 5
        
        # random values
        l = random.randint(l_min, l_max)
        sr_1 = random.randint(0, len(route)-1)
        sr_2 = sr_1+l-1 if sr_1+l<=len(route) else sr_1+l-len(route)-1
        p_rnd = random.uniform(0,1)
        
        route_wo_subroute = copy.deepcopy(route)
        if sr_1 < sr_2:
            subroute = route[sr_1:sr_2+1]
            route_wo_subroute[sr_1:sr_2+1] = []
        elif sr_2 < sr_1:
            subroute = route[sr_1:] + route[:sr_2+1]
            route_wo_subroute[sr_1:] = []
            route_wo_subroute[:sr_2+1] = []
        else:
            print("sub-route plan operator error sr_1 & sr_2")
        
        new_routes = list()     # max. 2
        
        #print(selected_route_id, sr_1, sr_2)
        #print(route)
        #print(route_wo_subroute)
        #print(subroute)

        # greedy reconnection
        if p_rnd <= p_rc:
            best_i_cost_diff = inf
            best_i = 0
            for i in range(0,len(route_wo_subroute)):
                cost_diff = self.min_cost[self.tasks[route_wo_subroute[i-1]].tail_node][self.tasks[subroute[0]].head_node] \
                    + self.min_cost[self.tasks[subroute[-1]].tail_node][self.tasks[route_wo_subroute[i]].head_node]
                if cost_diff < best_i_cost_diff:
                    best_i_cost_diff = cost_diff
                    best_i = i
            new_route = route_wo_subroute[:best_i] + subroute + route_wo_subroute[best_i:]
            new_routes.append(new_route)
        else:
            # random distortion (rolling or mixing)
            if p_rnd <= p_cp:
                w = sr_1
                new_route = route_wo_subroute
                while len(subroute) != 0:
                    rnd = random.uniform(0,1)
                    added_task = None
                    # random task -> mixing
                    if rnd <= p_l:
                        added_task = random.sample(subroute, k=1)[0]
                    # (currently) last task -> rolling
                    else:
                        added_task = subroute[-1]
                    new_route.insert(w, added_task)
                    subroute.remove(added_task)
                    w += 1
                new_routes.append(new_route)
            # sub-route plan rotation
            else:
                task_list_1 = copy.deepcopy(route)
                task_list_2 = copy.deepcopy(route)
                task_id_1 = task_list_1.pop(sr_1)
                task_id_2 = task_list_2.pop(sr_2)

                # note: only the tail node is considered since there are no inv.0 tasks
                sr_1_tasks_dists = [self.min_cost[self.tasks[task_id].tail_node][self.tasks[task_id_1].head_node] for task_id in task_list_1]
                sr_2_tasks_dists = [self.min_cost[self.tasks[task_id_2].tail_node][self.tasks[task_id].head_node] for task_id in task_list_2]
                sr_1_nearest_tasks = [task for _,task in sorted(zip(sr_1_tasks_dists, task_list_1))][:5]
                sr_2_nearest_tasks = [task for _,task in sorted(zip(sr_2_tasks_dists, task_list_2))][:5]
                nl_sr_1 = route.index(random.sample(sr_1_nearest_tasks, k=1)[0])
                nl_sr_2 = route.index(random.sample(sr_2_nearest_tasks, k=1)[0])
                
                if nl_sr_1 < sr_1:
                    new_route_1 = copy.deepcopy(route)
                    new_route_1[nl_sr_1:sr_1] = list(reversed(route[nl_sr_1:sr_1]))
                elif sr_1 < nl_sr_1:
                    sub_route = route[nl_sr_1:] + route[:sr_1]
                    sub_route.reverse()
                    new_route_1 = route[sr_1:nl_sr_1] + sub_route
                else:
                    print("sub-route plan operator error sr_1 & nl_sr_1")
                new_routes.append(new_route_1)

                if sr_2 < nl_sr_2:
                    new_route_2 = copy.deepcopy(route)
                    new_route_2[sr_2+1:nl_sr_2+1] = list(reversed(route[sr_2+1:nl_sr_2+1]))
                elif nl_sr_2 < sr_2:
                    sub_route = route[sr_2+1:] + route[:nl_sr_2+1]
                    sub_route.reverse()
                    new_route_2 = route[nl_sr_2+1:sr_2+1] + sub_route
                else:
                    print("sub-route plan operator error sr_2 & nl_sr_2")
                new_routes.append(new_route_2)

        # fix the new route plans (0 or vt_id may not be the first task in the new route plans)
        # note: vt_id is 0 if there is no vt in the route plan
        new_inds = list()       # max. 2
        for new_route in new_routes:
            fixed_route = list()
            i = 0
            while new_route[i] != vt_id:
                fixed_route.append(new_route[i])
                i += 1
            fixed_route = new_route[i+1:] + fixed_route if vt_id == 0 else \
                [vt_id] + new_route[i+1:] + fixed_route
            ind_new_sequence = ind.sequence[:ind.route_seg[selected_route_id][0]+1] + fixed_route + ind.sequence[ind.route_seg[selected_route_id][1]:]
            #print(new_route)
            #print(fixed_route)
            #print(ind_new_sequence)
            new_inds.append(self.ind_from_seq(ind_new_sequence))
        
        return self.get_best_solution(new_inds)
        
    
    def merge_split(self, ind, selected_route_ids=None):
        if selected_route_ids == None:
            p = random.randint(1,len(ind.route_seg)) # number of routes that are selected
            selected_route_ids = random.sample(list(np.arange(len(ind.route_seg), dtype=int)), k=p)
        unserved_tasks = list()
        unserved_vts = list()

        new_sequence = [0]   
        for route_id in range(len(ind.route_seg)):
            if route_id in selected_route_ids:
                if ind.sequence[ind.route_seg[route_id][0]+1] in self.virtual_task_ids.values():
                    unserved_vts.append(ind.sequence[ind.route_seg[route_id][0]+1])
                else:
                    unserved_tasks.append(ind.sequence[ind.route_seg[route_id][0]+1])
                for i in range(ind.route_seg[route_id][0]+2,ind.route_seg[route_id][1]):
                    unserved_tasks.append(ind.sequence[i])
            else:
                new_sequence += ind.sequence[ind.route_seg[route_id][0]+1:ind.route_seg[route_id][1]+1]
        new_sequence += self.randomized_path_scanning_heuristic(unserved_tasks, unserved_vts).sequence[1:]
        new_ind = self.ind_from_seq(new_sequence)
        """
        if len(ind.sequence) > len(new_sequence):
            print("")
            print(p, selected_route_ids)
            print(unserved_vts)
            print(ind.sequence)
            print(new_sequence)
        """
        return new_ind
 

# ----------------------------- DCARP components -----------------------------------


    def prepare_tasks(self):
        self.not_allowed_vt_ids = set()
        for r_id, vt_id in enumerate(self.virtual_task_ids):
            if r_id in self.finished_routes:
                self.not_allowed_vt_ids.add(vt_id)
        
        for vt_id in self.not_allowed_vt_ids:
            self.task_ids.remove(vt_id)
    
    
    def add_finished_routes(self, seq):
        for vt_id in self.not_allowed_vt_ids:
            seq.extend([vt_id, 0])
        return self.ind_from_seq(seq)
        
    
    def remove_finished_routes(self, seq):
        for vt_id in self.not_allowed_vt_ids:
            vt_id_index = seq.index(vt_id)
            seq[vt_id_index:vt_id_index+2] = []
        return self.ind_from_seq(seq)
    
    
    def generate_travel_service_log(self, ind, min_r_cost=0, max_r_cost=inf, stopping_r_cost=None):
        
        seq = ind.sequence      
        
        if max_r_cost == inf:
            max_r_cost = 0
            for r_id in range(len(ind.route_seg)):
                #print(ind.sequence[ind.route_seg[r_id][0]:ind.route_seg[r_id][1]+1])
                r_i_cost, tc_route_max = self.calculate_tc(seq[ind.route_seg[r_id][0]:ind.route_seg[r_id][1]+1])
                if r_i_cost > max_r_cost:
                    max_r_cost = r_i_cost 
        if stopping_r_cost == None:
            stopping_r_cost = random.randint(min_r_cost,max_r_cost)
        print(min_r_cost, max_r_cost, stopping_r_cost)
        
        log = list()
        
        r_i_cost = 0
        r_i_finished = False
        r_id = -1
        v_id = None
        for i in range(len(seq)-1):
            if seq[i] == 0:
                r_id += 1
                v_id = self.route_to_vehicle[r_id]
                r_i_cost = 0
                r_i_finished = False
            elif r_i_finished is False:
                if r_i_cost + self.tasks[seq[i]].serv_cost <= stopping_r_cost:
                    log.append((v_id, (self.tasks[seq[i]].head_node, self.tasks[seq[i]].tail_node), 1))
                    r_i_cost += self.tasks[seq[i]].serv_cost
                else:
                    # print(r_i_cost)
                    r_i_finished = True
            if r_i_finished is False:
                for j in range(1,self.shortest_path[self.tasks[seq[i]].tail_node][self.tasks[seq[i+1]].head_node][0]):
                    head_node = self.shortest_path[self.tasks[seq[i]].tail_node][self.tasks[seq[i+1]].head_node][j]
                    tail_node = self.shortest_path[self.tasks[seq[i]].tail_node][self.tasks[seq[i+1]].head_node][j+1]
                    if r_i_cost + self.trav_cost[head_node][tail_node] <= stopping_r_cost:
                        log.append((v_id, (head_node, tail_node), 0))
                        r_i_cost += self.trav_cost[head_node][tail_node]
                    else:
                        # print(r_i_cost)
                        r_i_finished = True
                        break
        
        return log
    
    
    def generate_task_appearance_event(self, log):
        served_arcs = self.served_arcs_from_log(log)
        required_arcs = set([(arc.head_node, arc.tail_node) for arc in self.tasks])
        all_arcs = set([(arc.head_node, arc.tail_node) for arc in self.arcs])
        # unrequired arcs/edges and required arcs/edges that have been already served (based on the log)
        potential_arcs = all_arcs.difference(required_arcs).union(served_arcs)
        
        if len(potential_arcs) == 0:
            return None, None, None
        
        # choose an arc
        new_task_arc = random.sample(potential_arcs, k=1)[0]
        
        # generate demand
        new_task_demand = random.randint(1,int(self.capacity/3))
        
        # get serv_cost (= trav_cost)
        new_task_arc_id = self.find_arc_id(new_task_arc[0], new_task_arc[1])
        new_task_serv_cost = self.arcs[new_task_arc_id].trav_cost        
        
        print(new_task_arc, new_task_demand, new_task_serv_cost)
        
        return new_task_arc, new_task_demand, new_task_serv_cost
        
    
    def generate_demand_increased_event(self, log):
        served_arcs = self.served_arcs_from_log(log)
        required_arcs = set([(arc.head_node, arc.tail_node) for arc in self.tasks])
        unserved_arcs = required_arcs.difference(served_arcs)
        unserved_arcs.remove((1,1))
      
        if len(unserved_arcs) == 0:
            return None, None, None
        
        # choose an arc
        di_task_arc = random.sample(unserved_arcs, k=1)[0]
        
        # choose the percentage of the increase for demand and service cost
        di_task_arc_id = self.find_task_id(di_task_arc[0], di_task_arc[1])
        curr_demand = self.tasks[di_task_arc_id].demand     
        incr_percentage = random.randint(1,100)/100
        while curr_demand + incr_percentage * curr_demand > self.capacity or \
            int(incr_percentage * curr_demand) <= 0:
            incr_percentage = random.randint(1,100)/100
        
        # calculate the increase in demand
        di_task_demand_increase = int(curr_demand * incr_percentage)
        
        # calculate the increase in service cost
        di_task_serv_cost_increase = int(self.tasks[di_task_arc_id].serv_cost * incr_percentage)
        
        print(di_task_arc_id, di_task_arc, di_task_demand_increase, di_task_serv_cost_increase)
        
        return di_task_arc, di_task_demand_increase, di_task_serv_cost_increase
    
    
    def generate_vehicle_breakdown_event(self, log):
        outside_vehicles = set()
        for event in log:
            if event[1][0] == self.depot:
                outside_vehicles.add(event[0])
            elif event[1][1] == self.depot:
                outside_vehicles.remove(event[0])
        
        if len(outside_vehicles) == 0:
            return None
        
        broken_v_id = random.sample(outside_vehicles, k=1)[0]
        
        print(broken_v_id)
        
        return broken_v_id
        
        
    def served_arcs_from_log(self, log):
        served_arcs = set()
        for event in log[self.last_processed_event_id+1:len(log)]:
            if event[2] == 1:
                task_id = self.find_task_id(event[1][0], event[1][1])
                if task_id != -1:
                    served_arcs.add((event[1][0], event[1][1]))             
                else:
                    print("Error: unknown task in the log on arc "+str(event))
        return served_arcs
    
    
    # collect the IDs of the served tasks 
    def served_task_ids_from_log(self, log):
        served_task_ids = set()        
        for event in log[self.last_processed_event_id+1:len(log)]:
            if event[2] == 1:
                # find the ID of a task by head and tail node
                task_id = self.find_task_id(event[1][0], event[1][1])
                if task_id != -1:
                    served_task_ids.add(task_id)
                else:
                    print("Error: unknown task in the log on arc "+str(event))
        return served_task_ids
    
    
    def create_virtual_tasks(self, ind, log, exception_v_id=-1):
        seq = list()
        exception_v_task_ids = list()
        # create virtual tasks and new individual (solution)
        for r_id in range(len(ind.route_seg)):
            v_id = self.route_to_vehicle[r_id]
            v_last_node = None
            # calculate the total cost and served demand
            cost = 0
            demand = 0
            for e_id in range(self.last_processed_event_id+1, len(log)):               
                if log[e_id][0] == v_id:
                    v_last_node = log[e_id][1][1]
                    # if reached the depot
                    if log[e_id][1][1] == self.depot:
                        self.finished_routes.add(r_id)
                        # if there are no more routes assigned to it and there are unassigned routes
                        if v_id not in self.route_to_vehicle[r_id+1:] and \
                            None in self.route_to_vehicle:
                                r_id = min(r_id for r_id, _v_id in enumerate(self.route_to_vehicle) if _v_id == None)
                                self.route_to_vehicle[r_id] = v_id
                        else:
                            self.free_vehicles.add(v_id)
                    # served arc -> find task (arc)
                    if log[e_id][2] == 1:
                        task_id = self.find_task_id(log[e_id][1][0], log[e_id][1][1])
                        if task_id != -1:
                            demand += self.tasks[task_id].demand
                            cost += self.tasks[task_id].serv_cost
                        else:
                            print("Error: unknown task in the log on arc "+str(log[e_id]))
                    # traversed arc -> find arc
                    elif log[e_id][2] == 0:
                        arc_id = self.find_arc_id(log[e_id][1][0], log[e_id][1][1])
                        if arc_id != -1:
                            cost += self.arcs[arc_id].trav_cost
                        else:
                            print("Error: unknown arc in the log: "+str(log[e_id]))
            
            # if it is a new route (i.e., doesn't have vt yet) and has been started to be executed
            if self.virtual_task_ids[r_id] == 0 and v_last_node != None:
                virtual_task = Task(self.depot, v_last_node, inf, cost, demand, None)
                virtual_task.id = len(self.tasks)
                self.tasks.append(virtual_task)
                vt_id = len(self.tasks)-1
                self.virtual_task_ids[r_id] = vt_id
                self.task_ids.add(vt_id)
            # if it has a vt and moved since last examination
            elif v_last_node != None:
                self.tasks[self.virtual_task_ids[r_id]].tail_node = v_last_node
                self.tasks[self.virtual_task_ids[r_id]].demand += demand
                self.tasks[self.virtual_task_ids[r_id]].serv_cost += cost

            seq.append(0)
            
            if self.virtual_task_ids[r_id] != 0:
                seq.append(vt_id)
            
            if v_id != exception_v_id:
                for task_id in ind.sequence[ind.route_seg[r_id][0]+1:ind.route_seg[r_id][1]]:
                    if task_id in self.task_ids:
                        seq.append(task_id)
            else:
                for task_id in ind.sequence[ind.route_seg[r_id][0]+1:ind.route_seg[r_id][1]]:
                    if task_id in self.task_ids:
                        exception_v_task_ids.append(task_id)
            
        seq.append(0)
        
        self.last_processed_event_id = len(log)-1
        
        if exception_v_id != -1:
            
            seq2 = copy.deepcopy(seq)
            if len(exception_v_task_ids) != 0:
                for task_id in exception_v_task_ids:
                    seq2.append(task_id)
                seq2.append(0)
            
            return seq, seq2, exception_v_task_ids
        
        return seq
    
    
    # nt_type: edge (0) / arc (1)
    def event_new_task(self, ind, log, nt_arc, nt_serv_cost, nt_demand, nt_type):

        if nt_type not in {0, 1}:
            print("Error: invalid task type")
            return None, None
        if nt_demand > self.capacity:
            print("Error: the demand of the new task is greater than the maximum capacity of the vehicles")
            return None, None
        
        # get the arc(s) of the task(s)
        arc_id = self.find_arc_id(nt_arc[0], nt_arc[1])

        if (nt_type == 0 and arc_id == -1) or (nt_type == 1 and arc_id == -1):
            print("Error: the new task cannot be added to an unknown arc")
            return None, None
        
        new_instance = copy.deepcopy(self)
        new_task_id = len(self.tasks)
        
        # create and add new task(s)
        if nt_type == 0:
            new_task = Task(self.arcs[arc_id].head_node, self.arcs[arc_id].tail_node, self.arcs[arc_id].trav_cost, \
                        nt_serv_cost, nt_demand)
            new_instance.tasks.append(new_task)
            new_instance.task_ids.add(new_task_id)
            new_instance.task_ids_with_inv.add(new_task_id)
        elif nt_type == 1:
            new_task = Task(self.arcs[arc_id].head_node, self.arcs[arc_id].tail_node, self.arcs[arc_id].trav_cost, \
                        nt_serv_cost, nt_demand, None)
            new_instance.tasks.append(new_task)
            new_instance.task_ids.add(new_task_id)
            new_instance.task_ids_with_inv.add(new_task_id)
        
        # remove the IDs of the served tasks from the IDs of the unserved tasks
        served_task_ids = self.served_task_ids_from_log(log)
        new_instance.task_ids_with_inv.difference_update(served_task_ids)
        new_instance.task_ids.difference_update(served_task_ids)
        
        # create virtual tasks and new sequence
        ind_wo_nr_seq = new_instance.create_virtual_tasks(ind, log)
        ind_w_nr_seq = copy.deepcopy(ind_wo_nr_seq) + [new_task_id, 0]
        #print(ind_w_nr_seq, ind_wo_nr_seq, new_task_id)

        return new_instance, (ind_w_nr_seq, ind_wo_nr_seq, [new_task_id])

    # t_type: arc (1)
    def event_increased_demand(self, ind, log, t_arc, t_serv_cost_incr, t_demand_incr):

        served_task_ids = self.served_task_ids_from_log(log)
        
        # get the task(s)
        task_id = self.find_task_id(t_arc[0], t_arc[1])
        print("The task id:", task_id)
        print("The old and the new demand:", self.tasks[task_id].demand, t_demand_incr + self.tasks[task_id].demand)

        if (task_id == -1):
            print("Error: the demand of an unknow task cannot be increased")
            return None, None, (None, None, None)
        if task_id in served_task_ids:
            print("Error: the demand of an already served task cannot be increased")
            return None, None, (None, None, None)
        if t_demand_incr + self.tasks[task_id].demand > self.capacity:
            print("Error: the new demand of the task is greater than the maximum capacity of the vehicles")
            return None, None, (None, None, None)
        
        # update the demand of the task(s)
        new_instance = copy.deepcopy(self)
        new_instance.tasks[task_id].demand += t_demand_incr
        new_instance.tasks[task_id].serv_cost += t_serv_cost_incr    
        
        # check if the vehicle still can serve all the tasks on its planned route
        v_demand = t_demand_incr
        r_id = -1
        v_id = -1
        # find v_id
        for t_id in ind.sequence:
            if t_id == 0:
                r_id += 1
            if t_id == task_id:
                v_id = self.route_to_vehicle[r_id]
                break
        # calculate the total demand of the served tasks
        for event in log:
            if event[0] == v_id and event[2] == 1:
                t_id = self.find_task_id(event[1][0], event[1][1])
                if t_id != -1:
                    v_demand += self.tasks[t_id].demand
                else:
                    print("Error: unknown task in the log on arc "+str(event))
        # calculate the total demand of the planned tasks (includind the tasl in question)
        current_route_plan = ind.sequence[ind.route_seg[r_id][0]+1:ind.route_seg[r_id][1]]
        for t_id in current_route_plan:
            if t_id not in served_task_ids:
                v_demand += self.tasks[t_id].demand
        
        # if the capacity constraint still holds -> no rerouting is needed
        excess_demand =  v_demand - self.capacity
        if excess_demand <= 0:
            print("No rerouting is needed.")
            return new_instance, 0, (ind, None, None)
        else:
            print("The demand exceeds the capacity by", excess_demand,". Rerouting is needed.")
            
        # remove the IDs of the served tasks (inv. too) from the IDs of the unserved tasks
        new_instance.task_ids_with_inv.difference_update(served_task_ids)
        new_instance.task_ids.difference_update(served_task_ids)
        
        # create virtusl tasks and new sequence
        ind_wo_nr_seq = new_instance.create_virtual_tasks(ind, log)              

        if task_id in ind_wo_nr_seq:
            ind_wo_nr_seq.remove(task_id)
        
        ind_w_nr_seq = copy.deepcopy(ind_wo_nr_seq) + [task_id, 0]
        #print(ind_w_nr_seq, ind_wo_nr_seq, task_id)
        
        return new_instance, 1, (ind_w_nr_seq, ind_wo_nr_seq, [task_id])
        

    # log: the set of event (travel and service) logs of the vehicles
    #   - log[v][i][0]: vehicle v, i-th event, traversed arc
    #   - log[v][i][1]: vehicle v, i-th event, only traversed (0) / served (1)
    # broken_v_id: the ID of the broken down vehicle
    def event_vehicle_breakdown(self, ind, log, broken_v_id):
        
        new_instance = copy.deepcopy(self)

        # remove the IDs of the served tasks (inv. too) from the IDs of the unserved tasks
        served_task_ids = self.served_task_ids_from_log(log)
        new_instance.task_ids_with_inv.difference_update(served_task_ids)
        new_instance.task_ids.difference_update(served_task_ids)

        # create virtual tasks and new sequence
        ind_wo_nr_seq, ind_w_nr_seq, bv_task_ids = new_instance.create_virtual_tasks(ind, log, broken_v_id)
        #print(ind_w_nr_seq, ind_wo_nr_seq, bv_task_ids)
        
#        new_instance.broken_vehicles.add(broken_v_id)
        r_ids = [r_id for r_id, v_id in enumerate(self.route_to_vehicle) if v_id == broken_v_id]
        current_r_id = max(r_ids)
        new_instance.finished_routes.add(current_r_id)
        
        print("The ID of the broken vehicle:", broken_v_id)
        print("The ID of the affected route plan:", current_r_id)
        
        # check whether the broken down vehicle still have tasks to serve
        if len(bv_task_ids) == 0:
            print("No rerouting is needed.")
            return new_instance, 0, (new_instance.ind_from_seq(ind_wo_nr_seq), None, None)
        else:
            print("Rerouting is needed. The affected task(s):")
            for task_id in bv_task_ids:
                print(task_id)
            return new_instance, 1, (ind_w_nr_seq, ind_wo_nr_seq, bv_task_ids)
    

# -------------------------------------- RR1 ------------------------------------------------

    
    def reroute_one_route(self, ind_w_nr_seq, ind_wo_nr_seq, task_ids):
        start_time = timer()
        output_text = ""
        
        # create individual with new route which contains the tasks of the broken down vehicle
        ind_w_nr = self.ind_from_seq(ind_w_nr_seq)       
        # create individual without the tasks of the broken down vehicle
        ind_wo_nr = self.ind_from_seq(ind_wo_nr_seq)

        tasks_demand = 0
        for task_id in task_ids:
            tasks_demand += self.tasks[task_id].demand
        
        potential_route_ids = set()
        for r_id in range(len(ind_wo_nr.route_seg)):
            if r_id not in self.finished_routes and \
                tasks_demand + ind_wo_nr.route_seg_load[r_id] <= self.capacity:
                potential_route_ids.add(r_id)
        
        if len(potential_route_ids) == 0:
            """
            self.virtual_task_ids.append(0)
            if len(self.free_vehicles) != 0:
                v_id = random.sample(self.free_vehicles, k=1)[0]
                self.free_vehicles.remove(v_id)
            else:
                v_id = None
            self.route_to_vehicle.append(v_id)
            """
            print("RR1 executed\t"+str(timer()-start_time))
            output_text += "RR1 executed\t"+str(timer()-start_time)+"\n"

            return ind_w_nr, output_text
               
        best_ind = ind_w_nr 
        for r_id in potential_route_ids:
            ind = copy.deepcopy(ind_wo_nr)
            for task_id in task_ids:
                r_task_best_i_cost_diff = inf
                r_task_best_i = -1
                for i in range(ind.route_seg[r_id][0]+2,ind.route_seg[r_id][1]+1):                  
                    cost_diff = self.tasks[task_id].serv_cost \
                    + self.min_cost[self.tasks[ind.sequence[i-1]].tail_node][self.tasks[task_id].head_node] \
                    + self.min_cost[self.tasks[task_id].tail_node][self.tasks[ind.sequence[i]].head_node] \
                    - self.min_cost[self.tasks[ind.sequence[i-1]].tail_node][self.tasks[ind.sequence[i]].head_node]

                    if cost_diff < r_task_best_i_cost_diff:
                        r_task_best_i_cost_diff = cost_diff
                        r_task_best_i = i
                
                ind.sequence.insert(r_task_best_i, task_id)
                ind.route_seg[r_id] = (ind.route_seg[r_id][0], ind.route_seg[r_id][1]+1)
                if r_id < len(ind.route_seg)-1:
                    for r_id_right in range(r_id+1, len(ind.route_seg)):
                        ind.route_seg[r_id_right] = (ind.route_seg[r_id_right][0]+1, ind.route_seg[r_id_right][1]+1)
                ind.route_seg_load[r_id] += self.tasks[task_id].demand
                ind.total_cost += r_task_best_i_cost_diff
            
            # if ind cost less or the same but have less routes than the current best ind -> new best ind
            if ind.total_cost < best_ind.total_cost:
                best_ind = ind
        
        """
        if best_ind == ind_w_nr:
            self.virtual_task_ids.append(0)
            if len(self.free_vehicles) != 0:
                v_id = random.sample(self.free_vehicles, k=1)[0]
                self.free_vehicles.remove(v_id)     
            else:
                v_id = None
            self.route_to_vehicle.append(v_id)
        """

        print("RR1 executed\t"+str(timer()-start_time))
        output_text += "RR1 executed\t"+str(timer()-start_time)+"\n"
        
        return best_ind, output_text
        

# --------------------------------- ABC --------------------------------------------

    def abc(self, colony_size=10, max_number=100000, no_improvement_limit=200, \
            local_search_limit=20, max_solution_age=10, initial_solution=None):
        output_text = ""
        self.start_time = timer()
        self.abc_initialize_population(colony_size, initial_solution)
        if initial_solution != None:
            print("initial solution: "+str(self.population[0].total_cost))
            output_text += "initial solution: "+str(self.population[0].total_cost)+"\n"
        self.best_solution = self.get_best_solution(self.population)
        current_best = copy.deepcopy(self.best_solution)
        print(str(current_best.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
        c = 0
        no_improvement = 0
        #self.print_population()
        while no_improvement < no_improvement_limit and timer()-self.start_time <= 60:
            self.employed_bee_phase(local_search_limit)
            self.onlooker_bee_phase(local_search_limit)
            self.scout_bee_phase(max_solution_age)

            if self.is_newer_ind_better(current_best, self.best_solution):
                current_best = copy.deepcopy(self.best_solution)
                real_solution = self.add_finished_routes(copy.deepcopy(current_best.sequence))
                print(str(real_solution.total_cost)+" ("+str(real_solution.tc_route_max)+")\t"+str(timer()-self.start_time)+"\t"+str(c))
                output_text += str(real_solution.total_cost)+" ("+str(real_solution.tc_route_max)+")\t"+str(timer()-self.start_time)+"\t"+str(c)+"\n"
                no_improvement = 0
            else:
                no_improvement += 1
            #print(str(c))
            #self.print_population()
            c += 1
            
        real_solution = self.add_finished_routes(self.best_solution.sequence)
        return real_solution, output_text
    

    def print_population(self):
        output = ""
        for (i, ind) in enumerate(self.population):
            output += str(ind.total_cost) + " \t"
        print(output)

    def abc_initialize_population(self, pop_size, initial_solution=None):        
        self.population_size = pop_size
        self.population = list(np.zeros(pop_size, dtype=int))
        self.population_probability = np.zeros(pop_size, dtype=float)

        start_id = 0
        if initial_solution != None:
            self.population[0] = initial_solution
            self.population[0].index = 0
            start_id = 1
        
        #print(list(self.virtual_task_ids.values()))
        for i in range(start_id, pop_size):
            self.population[i] = self.random_routing_plan_generator(self.task_ids)
            self.population[i].index = i
        
        
        """
        sols = list()
        for i in range(100):
            sols.append(self.random_routing_plan_generator(self.task_ids))
        sols.sort(key=lambda x: x.total_cost, reverse=False)
        for i in range(pop_size):
            ind = sols[i]
            ind.index = i
            self.population[i] = ind
        """
        
        self.best_solution = self.get_best_solution(self.population)
    
    
    def employed_bee_phase(self, search_trial):
        for i, ind in enumerate(self.population):           
            best_ind_i = copy.deepcopy(ind)
            best_ind_i.age = 0
            while best_ind_i.age < search_trial:
                new_ind = self.greedy_sub_tour_mutation(ind)
                best_ind = self.get_best_solution([best_ind_i, new_ind])
                if self.is_newer_ind_better(best_ind_i, best_ind):
                    best_ind_i = best_ind
                    best_ind_i.age = 0
                    """
                    if best_ind_i.total_cost < self.best_solution.total_cost:
                        print(str(best_ind_i.total_cost)+"\t"+str(timer()-self.start_time))
                    """
                else:
                    best_ind_i.age += 1
            if best_ind_i.sequence == ind.sequence:
                self.population[i].age += 1
            else:
                self.population[i] = best_ind_i
                self.population[i].age = 0
                self.population[i].index = i
        

    def onlooker_bee_phase(self, search_trial):
        #best_inds = random.choices(self.population, weights = self.population_probability, k = 3)
        #ind = max(best_inds, key=attrgetter('fitness'))
        best_inds = random.choices(self.population, k = 3)
        ind = self.get_best_solution(best_inds)

        neighbour_inds = [self.merge_split(ind) for i in range(self.population_size)]
        for i in range(self.population_size):
            neighbour_inds[i].age = 0
            best_ind_i = copy.deepcopy(neighbour_inds[i])
            best_ind_i.age = 0
            next_ind_i = copy.deepcopy(best_ind_i)
            while best_ind_i.age < search_trial:
                new_inds = [self.insert(next_ind_i, only_feasible=False), \
                            self.swap(next_ind_i, only_feasible=False), \
                            self.two_opt(next_ind_i, only_feasible=False), \
                            self.greedy_sub_tour_mutation(next_ind_i)]
                #next_ind_i = min([next_ind_i] + new_inds, key=attrgetter('total_cost'))
                next_ind_i = self.get_best_solution(new_inds)
                if self.is_newer_ind_better(best_ind_i, next_ind_i) and \
                    self.calculate_excess_demand(next_ind_i.route_seg_load)==0:
                    best_ind_i = next_ind_i
                    best_ind_i.age = 0
                    """
                    if best_ind_i.total_cost < self.best_solution.total_cost:
                        print(str(best_ind_i.total_cost)+"\t"+str(timer()-self.start_time))
                    """
                else:
                    best_ind_i.age += 1
            if best_ind_i.sequence == neighbour_inds[i].sequence:
                neighbour_inds[i].age += 1
            else:
                neighbour_inds[i] = best_ind_i
                neighbour_inds[i].age = 0
                neighbour_inds[i].index = ind.index 
            #print(neighbour_inds[i].total_cost, neighbour_inds[i].age)
            
        best_neighbour_ind = self.get_best_solution(neighbour_inds + [ind])
        #print(best_neighbour_ind.total_cost, ind.total_cost, self.population[ind.index].total_cost)
        
        if best_neighbour_ind.total_cost < ind.total_cost:
            self.population[ind.index] = best_neighbour_ind
            self.population[ind.index].age = 0
        else:
            self.population[ind.index].age += 1
        
        self.best_solution = self.get_best_solution(self.population + [self.best_solution])
        
    
    def scout_bee_phase(self, x):
        for i, ind in enumerate(self.population):
            if ind.age > x:                
                self.population[i] = self.random_routing_plan_generator(self.task_ids)
    
    
    def print_population_total_cost(self):
        return list((ind.total_cost, ind.age) for ind in self.population)
    

# --------------------------------- HMA -----------------------------------------

    # psize: population size
    # r: a set of treshold ratios
    # w: the number of non-improving attractors visited
    def hma(self, psize=10, r=[0.003, 0.004, 0.005, 0.006], w=5):
        self.start_time = timer()
        self.time_limit = inf
        # population initialization
        self.hma_initialize_population(psize)
        # record best solution so far
        self.best_solution = self.get_best_solution(self.population)
        real_solution = self.add_finished_routes(copy.deepcopy(self.best_solution.sequence))
        print(str(real_solution.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
        # initialize the counter array
        cnt = list(np.ones(len(r), dtype=int))
        # main search procedure
        x = 0
        while x<10 and timer()-self.start_time <= self.time_limit:
            #print(str(x)+". iteration")
            # randomly select 2 solutions
            ind1, ind2 = random.sample(self.population, 2)
            #print(ind1.sequence, ind1.total_cost)
            #print(ind2.sequence, ind2.total_cost)
            # Route-based crossover
            ind0 = self.route_based_crossover(ind1, ind2)
            #print("Route-based crossover")
            #print(ind0.sequence, ind0.total_cost)
            # Determine the order of conducting RTTP (0) and IDP (1)
            od = random.randint(0,1)
            # Select a ratio
            k = self.probabilistic_select_ratio(cnt)
            # Improve the solution
            ind0 = self.local_refinement(ind0, r[k], w, od)
            # Pool updating
            new_solution = False
            if self.calculate_quality_and_distance_fitness(ind0):
                new_best_solution = self.get_best_solution(self.population)
                if new_best_solution.total_cost < self.best_solution.total_cost:
                    new_solution = True
                self.best_solution = new_best_solution
                cnt[k] += 1
            #print("Global best")
            real_solution = self.add_finished_routes(copy.deepcopy(self.best_solution.sequence))
            if new_solution:
                print(str(real_solution.total_cost)+"\t"+str(timer()-self.start_time)+"\t"+str(x))
            #print()
            x += 1
        return real_solution
    
    
    def hma_initialize_population(self, pop_size):        
        self.population_size = pop_size       
        self.population = list(np.zeros(pop_size, dtype=int))
    
        self.population[0] = self.randomized_path_scanning_heuristic(self.task_ids)       
        for i in range(1,pop_size):
            ind = self.random_routing_plan_generator(self.task_ids)
            ind.index = i
            self.population[i] = ind

    
    def probabilistic_select_ratio(self, cnt):
        pr = copy.deepcopy(cnt)
        sum_cnt = sum(cnt)
        cnt_indexes = list(range(len(cnt)))
        for i in cnt_indexes:
            pr[i] = cnt[i] / sum_cnt
        return random.choices(cnt_indexes, weights = pr, k = 1)[0]

    
    def local_refinement(self, ind0, r, w, od):
        # RTTP first
        if od == 0:
            ind0 = self.rttp(r, w, ind0)
            ind0 = self.idp(ind0)
        # IDP first
        else:
            ind0 = self.idp(ind0)
            ind0 = self.rttp(r, w, ind0)
        return ind0
    

    # tasks1 - tasks2
    def route_difference(self, tasks1, tasks2):
        return tasks1.difference(tasks2)
    
    
    def route_based_crossover(self, ind1, ind2):
        ind0_seq = copy.deepcopy(ind1.sequence)
        a = random.randint(0,len(ind1.route_seg)-1)
        b = random.randint(0,len(ind2.route_seg)-1)
        ind0_seq[ind1.route_seg[a][0]:ind1.route_seg[a][1]+1] = \
            copy.deepcopy(ind2.sequence[ind2.route_seg[b][0]:ind2.route_seg[b][1]+1])
        
        ind1_ra_tasks = set(ind1.sequence[ind1.route_seg[a][0]+1:ind1.route_seg[a][1]])
        ind2_rb_tasks = set(ind2.sequence[ind2.route_seg[b][0]+1:ind2.route_seg[b][1]])
        
        unserved_tasks = self.route_difference(ind1_ra_tasks, ind2_rb_tasks)
        duplicate_tasks = self.route_difference(ind2_rb_tasks, ind1_ra_tasks)
        
        # remove the duplicate tasks from the positions which results with a better total cost
        for task in duplicate_tasks:
            task_pos = [i for i, t in enumerate(ind0_seq) if t == task]
            
            # cost of keeping task at pos0 and removing from pos1
            keeping_task_at_pos0_cost = \
                self.min_cost[self.tasks[ind0_seq[task_pos[0]-1]].tail_node, \
                              self.tasks[ind0_seq[task_pos[0]]].head_node] \
                + self.min_cost[self.tasks[ind0_seq[task_pos[0]]].tail_node, \
                                self.tasks[ind0_seq[task_pos[0]+1]].head_node] \
                + self.min_cost[self.tasks[ind0_seq[task_pos[1]-1]].tail_node,\
                                self.tasks[ind0_seq[task_pos[1]+1]].head_node]
            
            # cost of keeping task at pos1 and removing from pos0
            keeping_task_at_pos1_cost = \
                self.min_cost[self.tasks[ind0_seq[task_pos[1]-1]].tail_node, \
                              self.tasks[ind0_seq[task_pos[1]]].head_node] \
                + self.min_cost[self.tasks[ind0_seq[task_pos[1]]].tail_node, \
                                self.tasks[ind0_seq[task_pos[1]+1]].head_node] \
                + self.min_cost[self.tasks[ind0_seq[task_pos[0]-1]].tail_node,\
                                self.tasks[ind0_seq[task_pos[0]+1]].head_node]
            
            if keeping_task_at_pos0_cost < keeping_task_at_pos1_cost:
                del(ind0_seq[task_pos[1]])
            else:
                del(ind0_seq[task_pos[0]])
        
        # remove excess 0s (it is needed if a route plan became empty)
        task_pos = 1
        while task_pos < len(ind0_seq):
            if ind0_seq[task_pos-1] == 0 and ind0_seq[task_pos] == 0:
                del(ind0_seq[task_pos])
            else:
                task_pos += 1
        if ind0_seq[-1] != 0:
            ind0_seq.append(0)
        
        # sort the unserved tasks in random order
        duplicate_tasks = list(duplicate_tasks) 
        random.shuffle(duplicate_tasks)
        
        #print(ind0_seq)
        
        ind0 = self.ind_from_seq(ind0_seq)
        
        for task_id in unserved_tasks:
            potential_route_ids = set()
            for r_id in range(len(ind0.route_seg)):
                if ind0.route_seg_load[r_id] + self.tasks[task_id].demand <= self.capacity:
                    potential_route_ids.add(r_id)
            
            if potential_route_ids != set():       
                r_task_best_i_cost_diff = inf   # the cost difference of the best position
                r_task_best_r = -1              # the route plan id of the best position
                r_task_best_i = -1              # the best position
                for r_id in potential_route_ids:
                    for i in range(ind0.route_seg[r_id][0]+2,ind0.route_seg[r_id][1]+1):                  
                        cost_diff = self.tasks[task_id].serv_cost \
                        + self.min_cost[self.tasks[ind0.sequence[i-1]].tail_node][self.tasks[task_id].head_node] \
                        + self.min_cost[self.tasks[task_id].tail_node][self.tasks[ind0.sequence[i]].head_node] \
                        - self.min_cost[self.tasks[ind0.sequence[i-1]].tail_node][self.tasks[ind0.sequence[i]].head_node]
                        
                        if cost_diff < r_task_best_i_cost_diff:
                            r_task_best_i_cost_diff = cost_diff
                            r_task_best_r = r_id
                            r_task_best_i = i
                
                ind0.sequence.insert(r_task_best_i, task_id)
                ind0.route_seg[r_task_best_r] = (ind0.route_seg[r_task_best_r][0], ind0.route_seg[r_task_best_r][1]+1)
                if r_task_best_r < len(ind0.route_seg)-1:
                    for r_id_right in range(r_task_best_r+1, len(ind0.route_seg)):
                        ind0.route_seg[r_id_right] = (ind0.route_seg[r_id_right][0]+1, ind0.route_seg[r_id_right][1]+1)
                ind0.route_seg_load[r_task_best_r] += self.tasks[task_id].demand
                ind0.total_cost += r_task_best_i_cost_diff
            else:
                ind0.sequence.extend([task_id, 0])
                ind0 = self.ind_from_seq(ind0.sequence)
        
        #print(ind0.sequence)
        
        return ind0
    
    
    # Randomized Tabu Tresholding Procedure (RTTP)
    # r: threshold ratio
    # W: the number of non-improving attractors visited
    # ind: an initial solution
    def rttp(self, r, W, ind):
        operators = [self.insert, self.double_insert, self.swap, self.two_opt]
        ind_global_best = ind
        ind_curr = ind
        ind_local_best_total_cost = ind_global_best.total_cost
        w = 0
        while w < W:
            # Tabu timing value
            T = 5 # random.randint(28,33)
            
            # Mixed phase
            #print("RTTP Mixed phase")
            for k in range(1,T+1):
                random.shuffle(operators)
                for operator in operators:
                    task_ids = self.get_task_ids_from_sequence(ind_curr.sequence)
                    random.shuffle(task_ids)
                    for task_id in task_ids:
                        best_feasible_move_ind = ind
                        task_id_candidate_list = self.construct_candidate_list(task_id, task_ids)
                        for task_id_2 in task_id_candidate_list:                     
                            new_ind = operator(ind_curr, task_id, task_id_2)
                            if new_ind.total_cost <= (1 + r) * ind_local_best_total_cost and \
                                new_ind.total_cost < best_feasible_move_ind.total_cost:
                                best_feasible_move_ind = new_ind
                                if new_ind.total_cost < ind_global_best.total_cost:
                                    ind_global_best = new_ind
                                    """
                                    if ind_global_best.total_cost < self.best_solution.total_cost and timer()-self.start_time <= self.time_limit:
                                        print(str(ind_global_best.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
                                    """
                                    break
                            ind_curr = best_feasible_move_ind
                        else:
                            new_ind = operator(ind_curr, task_id)
                            if new_ind.total_cost <= (1 + r) * ind_local_best_total_cost:
                                ind_curr = new_ind
                                if new_ind.total_cost < ind_global_best.total_cost:
                                    ind_global_best = new_ind
                                    """
                                    if ind_global_best.total_cost < self.best_solution.total_cost and timer()-self.start_time <= self.time_limit:
                                        print(str(ind_global_best.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
                                    """
            
            #print(ind_global_best.sequence, ind_global_best.route_seg_load, ind_global_best.total_cost)
            
            # Improving phase
            #print("RTTP Improving phase")
            improvement = True
            while improvement:
                improvement = False
                random.shuffle(operators)
                for operator in operators:
                    task_ids = self.get_task_ids_from_sequence(ind_curr.sequence)
                    random.shuffle(task_ids)
                    for task_id in task_ids:
                        task_id_candidate_list = self.construct_candidate_list(task_id, task_ids)
                        for task_id_2 in task_id_candidate_list:
                            new_ind = operator(ind_curr, task_id, task_id_2)
                            if new_ind.total_cost < ind_global_best.total_cost:
                                ind_global_best = new_ind
                                ind_curr = new_ind
                                improvement = True
                                """
                                if new_ind.total_cost < self.best_solution.total_cost and timer()-self.start_time <= self.time_limit:
                                    print(str(new_ind.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
                                """
                                break
                        else:
                            new_ind = operator(ind_curr, task_id)
                            if new_ind.total_cost < ind_global_best.total_cost:
                                ind_global_best = new_ind
                                ind_curr = new_ind
                                improvement = True
                                """
                                if new_ind.total_cost < self.best_solution.total_cost and timer()-self.start_time <= self.time_limit:
                                    print(str(new_ind.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
                                """
            
            #print(ind_global_best.sequence, ind_global_best.route_seg_load, ind_global_best.total_cost)
            
            if ind_curr.total_cost < ind_local_best_total_cost:
                ind_local_best_total_cost = ind_curr.total_cost
                w = 0
            else:
                w += 1
        
        return ind_global_best
    
    
    def manage_penalty_parameter(self, penalty_parameter, feas_count, infeas_count, excess_dem):
        # infeasible solution
        if excess_dem != 0:
            feas_count += 1
            if infeas_count > 0:
                infeas_count = 0
        # feasible solution
        else:
            infeas_count += 1
            if feas_count > 0:
                feas_count = 0
        # halve the penalty parameter
        if feas_count == 5:
            penalty_parameter = penalty_parameter / 2
            feas_count = 0
        # double the penalty parameter
        elif infeas_count == 5:
            penalty_parameter = penalty_parameter * 2
            infeas_count = 0
        return penalty_parameter, feas_count, infeas_count
        
    
    # infeasible descent procedure
    def idp(self, ind):
        penalty_parameter = ind.total_cost / (2 * self.capacity)
        feas_count = 0
        infeas_count = 0
        
        best_ind = ind
        best_ind_cost = ind.total_cost
        
        # first stage
        #print("IDP first stage")
        task_ids = self.get_task_ids_from_sequence(ind.sequence)
        operators = [self.insert, self.double_insert, self.swap]
        for operator in operators:
            for task_id_1 in task_ids:
                for task_id_2 in task_ids:
                    if task_id_1 != task_id_2:
                        
                        new_ind = operator(ind, task_id_1, task_id_2, False)
                        excess_dem = self.calculate_excess_demand(new_ind.route_seg_load)
                        new_ind_cost = new_ind.total_cost + penalty_parameter * excess_dem                          
                        
                        if new_ind_cost < best_ind_cost:
                            best_ind = new_ind
                            best_ind_cost = new_ind_cost
                            
                            # temp
                            if new_ind.total_cost < self.best_solution.total_cost and \
                                excess_dem == 0 and \
                                timer()-self.start_time <= self.time_limit:
                                print(str(new_ind.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
                                                           
                            penalty_parameter, feas_count, infeas_count = \
                                self.manage_penalty_parameter(penalty_parameter, feas_count, infeas_count, excess_dem)
        
        #print(best_ind.sequence, best_ind.route_seg_load, best_ind.total_cost, best_ind_cost)
        
        # second stage
        improvement_found = False
        if best_ind.total_cost == ind.total_cost:
            #print("IDP second stage")
            r_nr = len(ind.route_seg)
            r_ids = [i for i in range(r_nr)]
            # possible combinations of route ids for merge-split operator
            possible_combinations = [[i] for i in range(r_nr)] + [r_ids]
            if r_nr > 2:
                for i in range(2,r_nr):
                    possible_combinations += combinations(r_ids, i)
            random.shuffle(possible_combinations)
            
            tried_combinations_nr = len(possible_combinations) if len(possible_combinations) < 100 else 100
            # try the first max. 100 possible combinations
            for i in range(tried_combinations_nr):
                # use merge-split
                new_ind = self.merge_split(best_ind, selected_route_ids=possible_combinations[i])
                
                excess_dem = self.calculate_excess_demand(new_ind.route_seg_load)
                new_ind_cost = new_ind.total_cost + penalty_parameter * excess_dem                          
                
                if new_ind_cost < best_ind_cost:
                    improvement_found = True
                    best_ind = new_ind
                    best_ind_cost = new_ind_cost
                    
                    # temp
                    if new_ind.total_cost < self.best_solution.total_cost and \
                        excess_dem == 0 and \
                        timer()-self.start_time <= self.time_limit:
                        print(str(new_ind.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")                              
                    
                    penalty_parameter, feas_count, infeas_count = \
                        self.manage_penalty_parameter(penalty_parameter, feas_count, infeas_count, excess_dem)
            
            #print(best_ind.sequence, best_ind.route_seg_load, best_ind.total_cost, best_ind_cost)
                            
        if improvement_found:
            # first stage
            #print("IDP first stage")
            task_ids = self.get_task_ids_from_sequence(ind.sequence)
            operators = [self.insert, self.double_insert, self.swap]
            for operator in operators:
                for task_id_1 in task_ids:
                    for task_id_2 in task_ids:
                        if task_id_1 != task_id_2:
                            
                            new_ind = operator(ind, task_id_1, task_id_2, False)
                            excess_dem = self.calculate_excess_demand(new_ind.route_seg_load)
                            new_ind_cost = new_ind.total_cost + penalty_parameter * excess_dem                          
                            
                            if new_ind_cost < best_ind_cost:
                                best_ind = new_ind
                                best_ind_cost = new_ind_cost
                                
                                # temp
                                if new_ind.total_cost < self.best_solution.total_cost and \
                                    excess_dem == 0 and \
                                    timer()-self.start_time <= self.time_limit:
                                    print(str(new_ind.total_cost)+"\t"+str(timer()-self.start_time)+"\t0")
                                
                                penalty_parameter, feas_count, infeas_count = \
                                    self.manage_penalty_parameter(penalty_parameter, feas_count, infeas_count, excess_dem)
            #print(best_ind.sequence, best_ind.route_seg_load, best_ind.total_cost, best_ind_cost)
        
        return best_ind
                
    
    # calculate the distance for each task, sort it, keep only the top x
    
    def calculate_distance_between_two_tasks(self, task_id_1, task_id_2):
        return self.min_cost[self.tasks[task_id_1].tail_node][self.tasks[task_id_2].head_node]
    
    
    def construct_candidate_list(self, task_id, task_ids):
        csize = 12  # candidate list size
        task_ids.remove(task_id)
        dist_values = [inf for i in range(len(task_ids))]
        for index, other_task_id in enumerate(task_ids):
            dist_values[index] = (other_task_id, self.calculate_distance_between_two_tasks(task_id, other_task_id))
        
        return [t_id for t_id, dist_value in sorted(dist_values, key=lambda x: x[1])[:csize]]
    
    
    def hamming_distance(self, ind1, ind2):
        n = self.req_edge_num
        m = min(len(ind1.route_seg),len(ind2.route_seg))
        
        same_dh_link_nr = 0
        ind1_dh_links = set()
        
        for i in range(len(ind1.sequence)-1):
           ind1_dh_links.add((self.tasks[ind1.sequence[i]].head_node, \
                     self.tasks[ind1.sequence[i+1]].tail_node))
        
        for i in range(len(ind2.sequence)-1):
            if (self.tasks[ind2.sequence[i]].head_node, \
                     self.tasks[ind2.sequence[i+1]].tail_node) in ind1_dh_links:
                same_dh_link_nr += 1
        
        return n + m - same_dh_link_nr
    
    
    def calculate_quality_and_distance_fitness(self, ind_new):
        self.population.append(ind_new)
        new_population_size = self.population_size + 1
        
        AD_pop = np.zeros(new_population_size, dtype=int)
        for i in range(new_population_size):
            AD_pop[i] = 0
            for j in range(new_population_size):
                if i != j:
                    AD_pop[i] += self.hamming_distance(self.population[i], \
                                                      self.population[j])
            AD_pop[i] = AD_pop[i] / new_population_size - 1
        
        f_pop = [ind.total_cost for ind in self.population]
        
        OR = np.argsort(f_pop)
        DR = AD_pop.argsort()[::-1]
        
        population_qdf = np.zeros(new_population_size, dtype=int)       
        for i in range(new_population_size):
            population_qdf[OR[i]] += (i+1) * alpha
            population_qdf[DR[i]] += (i+1) * (1-alpha)
        
        max_qdf_value = max(population_qdf)
        max_qdf_value_index = list(population_qdf).index(max_qdf_value)
        del(self.population[max_qdf_value_index])
        
        if max_qdf_value_index == self.population_size:
            return False
        else:
            return True

# ------------------------------------- HyLS -----------------------------------------------


    # HyLS main algorithm
    def hyls(self, initial_solution=None):
        output_text = ""
        self.AXVSIZE = 15
        terminal_duration = 60

        ind_archive = [initial_solution]                        # initialize solution archive
        ind_archive_len = 1
        self.best_solution = copy.deepcopy(initial_solution)    # set global best solution
        print("initial solution: "+str(self.best_solution.total_cost))
        output_text += "initial solution: "+str(self.best_solution.total_cost)+"\n"

        operators = [self.insert, self.double_insert, self.swap, self.two_opt]

        self.start_time = timer()
        while len(ind_archive) > 0 and (timer()-self.start_time) <= terminal_duration:
            ind = ind_archive.pop(0)
            local_best_ind = copy.deepcopy(ind)
            opertor_best_inds = [ind for i in range(4)]

            while True:
                for operator_id in range(4):
                    while True:
                        improvement = False
                        for task_id_1 in self.task_ids:
                            for task_id_2 in self.task_ids:
                                if task_id_1 != task_id_2:
                                    next_ind = operators[operator_id](opertor_best_inds[operator_id], task_id_1, task_id_2)
                                    if ind_archive_len < self.AXVSIZE and \
                                        self.is_newer_ind_better(opertor_best_inds[operator_id], next_ind):
                                        ind_archive.append(next_ind)
                                        ind_archive_len += 1
                                        improvement = True
                                        opertor_best_inds[operator_id] = next_ind
                        if not improvement or (timer()-self.start_time) > terminal_duration:
                            break
                    if (timer()-self.start_time) > terminal_duration:
                        break
                iter_best_ind = self.get_best_solution(opertor_best_inds)
                if self.is_newer_ind_better(local_best_ind, iter_best_ind):
                    local_best_ind = iter_best_ind
                else:
                    break
                if (timer()-self.start_time) > terminal_duration:
                    break
            # update best solution
            if self.is_newer_ind_better(self.best_solution, local_best_ind):
                self.best_solution = local_best_ind
                real_solution = self.add_finished_routes(self.best_solution.sequence)
                print(str(real_solution.total_cost)+" ("+str(real_solution.tc_route_max)+")\t"+str(timer()-self.start_time))
                output_text += str(real_solution.total_cost)+" ("+str(real_solution.tc_route_max)+")\t"+str(timer()-self.start_time)+"\n"
            # stop if exceeds time limit
            if timer()-self.start_time > terminal_duration:
                break

        real_solution = self.add_finished_routes(self.best_solution.sequence)
        # print(str(real_solution.total_cost)+" ("+str(real_solution.tc_route_max)+")\t"+str(timer()-self.start_time))
        print(real_solution.sequence)
        output_text += str(real_solution.sequence)+"\n"
        return real_solution, output_text



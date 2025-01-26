import sys
import time, copy, os
import sumolib, traci  # noqa
import pickle
import subprocess
import xml.etree.ElementTree as ET
from utils.carp import depot_in, depot_out, gc
from utils.carp import task, routine, solution, dis_flag
from utils.carp import cal_route_cost_with_vt
import numpy as np
from utils.plot_net_selection import plot_route
from contextlib import redirect_stdout

import utils.optimizer_utils as u

class sumo_dcarp_init():

    # pre-defined new tasks (Note: I decreased the number from 20 to 10 with random choice)
    new_tasks_whole = [2721, 554, 1001, 3617, 278, 3810, 5545, 2618, 2788, 5831]
    busy_area = [2220, 6513, 4063, 1199, 1606, 540, 178, 4340, 3685, 3474]
    not_busy_area = [498, 941, 5708, 5831, 4176, 1326, 1832, 5752, 5707, 6400]
    added_task_count = 0

    @classmethod
    def init(self, filepath):
        fs = sumo_dcarp_init.init_solution(filepath)
        sumo_dcarp_init.add_routes(fs)
        disflags = dis_flag()
        return fs, disflags

    @classmethod
    def init_solution(self, scenario):
        global edge_list

        # scenario = "dcarp/scenario1.xml"
        tree = ET.ElementTree(file=scenario)
        init_solution = tree.getroot().find("solution")

        x1 = list(init_solution.iter("task"))[0].attrib["seq"]
        x2 = list(init_solution.iter("demand"))[0].attrib["seq"]
        tasks_seqs = list(map(lambda x:int(x), x1.split()))
        demand_seqs = list(map(lambda x:int(x), x2.split()))

        fs = solution() # first solution
        route = routine()
        for t, d in zip(tasks_seqs[1:], demand_seqs[1:]):
            if t == 0:
                route.complete()
                fs.add_routine(route)
                del route
                route = routine() # new route
                continue
            
            tt = task(edge_list[t], d) # constrcut tasks
            tt.id = t
            route.add_tasks(tt)
            traci.gui.toggleSelection(tt.edge,  objType='edge')

        return fs


    @classmethod
    def add_routes(self, es):
        route_num = 0
        
        for k in range(len(es.routines)):
            route_edges_seqs = ()
            route = es.routines[k]
            task_seqs = route.tasks_seqs
            for i in range(len(task_seqs)-1):
                start = task_seqs[i].edge
                end = task_seqs[i+1].edge
                rr = traci.simulation.findRoute(start, end, routingMode=1)
                route_edges_seqs += rr.edges[:-1]

                rr.length

            route_edges_seqs += (depot_in.edge, )
            route_num += 1
            route_id = str(time.time()+route_num).replace('.', '')
            vechile_id = "nv" + str(route_num)
            traci.route.add(route_id, route_edges_seqs)
            # type = delivery (self defined in rou.xml)
            traci.vehicle.add(vechile_id, route_id, typeID="carp", depart=traci.simulation.getTime()+1) 
            traci.vehicle.setColor(vechile_id, (255,0,0))
            traci.vehicle.setParameter(vechile_id, "has.rerouting.device", "true")
            traci.vehicle.setParameter(vechile_id, "device.rerouting.period", 100000.0)
            gc.add_vehicle(vechile_id, gc.begin_time)
            es.veh_route[vechile_id] = k
            es.routines[k].vid = vechile_id
            gc.set_veh_load(vechile_id)
            traci.gui.toggleSelection(vechile_id)
            # break
        return route_num


class sumo_dcarp():

    def __init__(self):
        pass
    
    @classmethod
    def detect_vehicles(self):
        for vid in gc.vehicles:
            try:
                traci.vehicle.getLaneID(vid)
            except:
                gc.remove_vehicle(vid)
    
    @classmethod
    def reschedule(self, scenario, instance_nr, new_tasks_added=False):
        global disflags, fs
        output_text = ""

        # determine which tasks have been served first.
        served_tasks = {}
        for vid in disflags.flags.keys():
            if vid not in gc.vehicles:
                served_tasks[vid] = -1
            else:
                flags = disflags.flags[vid]
                task_num = len(flags)
                dis = traci.vehicle.getDrivingDistance(vid, depot_in.edge, 1)
                # dis >= flags[j]: not been served; dis < flags[j]: been served
                for j in range(1, task_num+1):
                    if dis >= flags[j-1]:
                        break
                served_tasks[vid] = j
        
        # construct virtual tasks 
        for vid in disflags.flags.keys():
            if served_tasks[vid] < 0:
                continue

            # 1. obtain stop locations
            edge_id = traci.vehicle.getRoadID(vid)
            stop_edge = net.getEdge(edge_id).getID()
            #print(vid, edge_id)

            # 2. obtaining served load
            i = fs.veh_route[vid]
            for j in range(1, served_tasks[vid]):
                remove_tasks = fs.routines[i].tasks_seqs.pop(1)

                if gc.veh_load[vid] == 1:
                    gc.veh_load[vid] -= 1
                gc.veh_add_load(vid, remove_tasks.demand)
            
            # make virtual tasks' demand not equal to 0
            if  gc.veh_load[vid] == 0:
                gc.veh_load[vid] += 1
            
            # construct new solutions
            vt = task(stop_edge, gc.veh_load[vid], vt=True)
            fs.routines[i].tasks_seqs.insert(1, vt)
            fs.routines[i].cost = cal_route_cost_with_vt(fs.routines[i].tasks_seqs)

        remove_idx = []
        for vid in disflags.flags.keys():
            if served_tasks[vid] < 0:
                remove_idx.append(fs.veh_route[vid])
        remove_idx.sort(reverse=True)
        
        for idx in remove_idx:
            fs.routines.pop(idx)

        fs.veh_route = {}
        for i in range(len(fs.routines)):
            fs.veh_route[fs.routines[i].vid] = i
            
        fs.complete()
        
        # if has new tasks, use RR1 then rechedule

        # fs --> new solutions with virtual tasks
        
        new_tasks = []

        if new_tasks_added:
            if scenario in [1,4,7,10]:
                new_tasks = sumo_dcarp_init.new_tasks_whole
            elif scenario in [2,5,8,11]:
                new_tasks = sumo_dcarp_init.busy_area
            elif scenario in [3,6,9,12]:
                new_tasks = sumo_dcarp_init.not_busy_area
        
        added_tasks = []
        if len(new_tasks) > 0:
            print("added task current time:", traci.simulation.getTime())
            output_text += "added task current time: "+str(traci.simulation.getTime())+"\n"
            added_task = new_tasks.pop()
            added_tasks.append(added_task)
            output_text += "added task edgeID: "+str(added_task)+"\n"
            sumo_dcarp_init.added_task_count += 1


        folder = "xml/scenario{0}_instance{1}".format(scenario, instance_nr)

        remain_tasks_num = 0

        # calculate and save the distance matrix for the map (weights.xml)
        sumo_dcarp.duaroute_cal_route_cost(folder, added_tasks)

        # save the current solution for local search (solution.xml)
        remain_tasks_num, instance, ind,  ot = sumo_dcarp.parse_tasks_seq(scenario, instance_nr, added_tasks)
        output_text += ot

        if remain_tasks_num > 3:
            
            print("ABC run")
            output_text += "ABC run\n"
            new_solution, abc_output_text = instance.abc(initial_solution=ind)
            output_text += abc_output_text

            """
            print("HyLS run")
            output_text += "HyLS run\n"
            new_solution, hyls_output_text = instance.hyls(initial_solution=ind)
            output_text += hyls_output_text
            """
            #new_solution = ind

            print(new_solution.sequence, new_solution.total_cost)
            output_text += str(new_solution.sequence)+" "+str(new_solution.total_cost)+"\n"

            # load new solution and assign to the vehicle (new_solution.xml)
            output_text += sumo_dcarp.load_new_schedule(instance, new_solution)

            print("Rerouting complete! :)\n")
            output_text += "Rerouting complete! :)\n"
        else:
            for i in range(len(fs.routines)):
                if (fs.routines[i].tasks_seqs[1].vt):
                    fs.routines[i].tasks_seqs.pop(1)
        
        return output_text


    @classmethod
    def duaroute_cal_route_cost(self, folder, added_tasks):
        global edge_list,fs

        if not os.path.isdir(folder):
            os.mkdir(folder)

        for v in gc.vehicles:
            try:
                traci.vehicle.getColor(v)
            except:
                gc.vehicles.remove(v)
        
        root = ET.Element('meandata')
        for eid in edge_list[1:]:
            t = []
            for v in gc.vehicles:
                t.append(float(traci.vehicle.getParameter(v, "device.rerouting.edge:"+eid)))
            ET.SubElement(root,'edge', {'id':eid, 'traveltime':str(np.mean(t))})
        tree=ET.ElementTree(root)
        tree.write(folder+"/weights.xml")

        tasks = [depot_out.edge, depot_in.edge]
        for r in fs.routines:
            for tsk in r.tasks_seqs[1:-1]:
                tasks.append(tsk.edge)
        
        for tsk in added_tasks:
            tsk_eid = edge_list[tsk]
            traci.gui.toggleSelection(tsk_eid,  objType='edge')
            tasks.append(tsk_eid)
        
        root = ET.Element('routes')
        curr_time = str(traci.simulation.getTime())
        k = 0
        for src in tasks:
            k += 1
            dst = src
            ET.SubElement(root,'trip', {'id':str(k), 'depart': curr_time, 'from':src, 'to':dst})
        
        for src in tasks:
            for dst in tasks: 
                if src == dst:
                    continue
                k += 1
                ET.SubElement(root,'trip', {'id':str(k), 'depart': curr_time, 'from':src, 'to':dst})
        tree=ET.ElementTree(root)
        tree.write(folder+"/trips.xml")
        
        subprocess.call(["duarouter","--route-files", folder+"/trips.xml","--net-file","scenario/DCC.net.xml","--weight-files",folder+"/weights.xml","--bulk-routing","true", "--output-file",folder+"/result.rou.xml"])

    @classmethod
    def parse_tasks_seq(self, scenario, instance_nr, added_tasks):
        global fs, net, node_dict, edge_dict

        output_text = ""

        # create instance
        instance = u.Instance()
        instance.capacity = gc.capacity
        instance.tasks = [0 for i in range(301)]

        # add depot task & arcs
        depot_from_node_id = node_dict[net.getEdge(depot_out.edge).getFromNode().getID()]
        depot_to_node_id = node_dict[net.getEdge(depot_out.edge).getToNode().getID()]
        instance.tasks[0] = u.Task(0, depot_from_node_id, depot_from_node_id, 0, 0, 0)
        instance.arcs[depot_out.edge] = u.Arc(depot_from_node_id, depot_to_node_id, u.inf)
        instance.arcs[depot_in.edge] = u.Arc(depot_to_node_id, depot_from_node_id, u.inf)

        # add the other tasks and a solution
        k = 0
        remain_tasks_num = 0
        solution_seq = [0]
        normal_tasks = set()
        current_time = int(traci.simulation.getTime())
        for vid in fs.veh_route:
            if vid not in gc.vehicles:
                continue
            r = fs.routines[fs.veh_route[vid]]
 
            # if all tasks in the route have been served and the remaining edges smaller than 10 edges. ignore this route. 
            route_edges = traci.vehicle.getRoute(vid)
            curr_idx = traci.vehicle.getRouteIndex(vid)
            if (len(route_edges) - curr_idx < 10) and len(r.tasks_seqs) == 3:
                instance.arcs[r.tasks_seqs[1].edge] = \
                    u.Arc(node_dict[net.getEdge(r.tasks_seqs[1].edge).getFromNode().getID()], \
                        node_dict[net.getEdge(r.tasks_seqs[1].edge).getToNode().getID()], u.inf)
                continue

            for tsk in r.tasks_seqs[1:-1]:
                k += 1
                from_node = net.getEdge(tsk.edge).getFromNode().getID()
                to_node = net.getEdge(tsk.edge).getToNode().getID()
                cost = u.inf
                if tsk.vt:
                    instance.virtual_task_ids[vid] = k
                    cost = 0 # current_time - gc.vehicles_depart_time[vid]
                    instance.tasks[k] = u.Task(edge_dict[tsk.edge], depot_from_node_id, node_dict[to_node], u.inf, cost, tsk.demand)
                else:
                    remain_tasks_num += 1
                    instance.task_ids.add(k)
                    instance.tasks[k] = u.Task(edge_dict[tsk.edge], node_dict[from_node], node_dict[to_node], u.inf, cost, tsk.demand)
                solution_seq.append(k)
                instance.arcs[tsk.edge] = u.Arc(node_dict[from_node], node_dict[to_node], u.inf)
            solution_seq.append(0)
        
        if (len(added_tasks) > 0):
            solution_seq_wo_new_tasks = copy.deepcopy(solution_seq)
            new_tasks_ids = list()
            for tsk in added_tasks:
                k += 1
                new_tasks_ids.append(k)
                tsk_id = edge_list[tsk]
                from_node = net.getEdge(tsk_id).getFromNode().getID()
                to_node = net.getEdge(tsk_id).getToNode().getID()
                tsk_demand = int(net.getEdge(tsk_id).getLength() / 10)
                remain_tasks_num += 1
                instance.arcs[tsk_id] = u.Arc(node_dict[from_node], node_dict[to_node], u.inf)
                instance.tasks[k] = u.Task(tsk, node_dict[from_node], node_dict[to_node], u.inf, u.inf, tsk_demand)
                instance.task_ids.add(k)
                solution_seq.append(k)
            solution_seq.append(0)
        
        # load file and calculate min_cost

        folder = "xml/scenario{0}_instance{1}".format(scenario, instance_nr)
        file_path = folder + "/result.rou.alt.xml"
        
        NODE_NUM = 2896
        min_cost = np.full((NODE_NUM,NODE_NUM), -1)
        for i in range(NODE_NUM):
            min_cost[i][i] = 0

        if os.path.isfile(file_path):        
            tree = ET.ElementTree(file=file_path)
            root = tree.getroot()
            for route in root.iter('route'):
                cost = float(route.attrib["cost"])
                edges = route.attrib["edges"].split()

                tmp_cost = int(round(cost))
                from_edge = edges[0]
                to_edge = edges[-1]
                head = instance.arcs[from_edge].head_node
                tail = instance.arcs[from_edge].tail_node
                if (from_edge == to_edge):
                    min_cost[head][tail] = tmp_cost
                    instance.arcs[from_edge].trav_cost = tmp_cost
                else:
                    head_to = instance.arcs[to_edge].head_node
                    tail_to = instance.arcs[to_edge].tail_node
                    if (min_cost[head][head_to] < 0): 
                        min_cost[head][head_to] = tmp_cost - min_cost[head_to][tail_to]
                    if (min_cost[head][tail_to] < 0): 
                        min_cost[head][tail_to] = tmp_cost
                    if (min_cost[tail][head_to] < 0): 
                        min_cost[tail][head_to] = tmp_cost - min_cost[head][tail] - min_cost[head_to][tail_to]
                    if (min_cost[tail][tail_to] < 0): 
                        min_cost[tail][tail_to] = tmp_cost - min_cost[head][tail]
        else:
            print("Error: File not foud!")
        
        for i in range(NODE_NUM):
            for j in range(NODE_NUM):
                if min_cost[i][j] == -1:
                    min_cost[i][j] = u.inf

        instance.min_cost = min_cost

        # update the cost values and create the ind

        for task_id in instance.task_ids:
            instance.tasks[task_id].serv_cost = min_cost[instance.tasks[task_id].head_node][instance.tasks[task_id].tail_node]
            instance.tasks[task_id].trav_cost = instance.tasks[task_id].serv_cost

        ind = instance.ind_from_seq(solution_seq)
        print(ind.sequence, ind.total_cost)
        output_text += str(ind.sequence)+"\t"+str(ind.total_cost)+"\n"

        if (len(added_tasks) > 0):
            ind, rr1_output_text = instance.reroute_one_route(solution_seq, solution_seq_wo_new_tasks, new_tasks_ids)
            print(ind.sequence, ind.total_cost)
            output_text += rr1_output_text
            output_text += str(ind.sequence)+"\t"+str(ind.total_cost)+"\n"
        
        return remain_tasks_num, instance, ind, output_text

    @classmethod
    def load_new_schedule(self, instance, new_solution):
        global fs
        new_sol = solution()
        output_text = ""

        for r in new_solution.route_seg:
            new_route = routine()
            route_edges = []
            vt = False
            
            first_task_id = new_solution.sequence[r[0]+1]
            #print(str(first_task_id), str(instance.tasks[first_task_id].head_node), str(instance.tasks[first_task_id].tail_node))
            e = edge_list[instance.tasks[first_task_id].id]
            # if the first task is a virtual task
            if first_task_id in instance.virtual_task_ids.values():
                vid = instance.get_vehicle_from_vt(first_task_id)
                vt = True
            # assign a new vehicle
            else:
                print("new vehicle needed")
                output_text += "new vehicle needed\n"
                vid = "nv" + str(int(gc.vehicles[-1][2:])+1)
                route_edges.append(depot_out.edge)
                gc.add_vehicle(vid, traci.simulation.getTime()+1)
                gc.set_veh_load(vid)
                demand = instance.tasks[first_task_id].demand
                new_route.add_tasks(task(e, demand))
          
            route_edges.append(e)

            for task_id in new_solution.sequence[r[0]+2:r[1]]:
                e = edge_list[instance.tasks[task_id].id]
                demand = instance.tasks[task_id].demand
                new_route.add_tasks(task(e, demand))
                route_edges.append(e)

            route_edges.append(depot_in.edge)
            route_edges_paths = []
            for i in range(len(route_edges)-1):
                start = route_edges[i]
                end = route_edges[i+1]
                rr = traci.simulation.findRoute(start, end, routingMode=1)
                route_edges_paths += rr.edges[:-1]
            route_edges_paths.append(depot_in.edge)
            
            if (not vt):
                print("new vehicle added")
                output_text += "new vehicle added\n"
                route_id = "new" + str(time.time())
                traci.route.add(route_id, route_edges_paths)
                traci.vehicle.add(vid, route_id, typeID="carp", depart=traci.simulation.getTime()+1) 
                traci.vehicle.setColor(vid, (255,0,0))
                traci.vehicle.setParameter(vid, "has.rerouting.device", "true")
                traci.vehicle.setParameter(vid, "device.rerouting.period", 100000.0)
                traci.gui.toggleSelection(vid)
            # print(route_edges_paths)

            traci.vehicle.setRoute(vid, route_edges_paths)
            new_route.complete()
            new_route.tasks_seqs.insert(0, depot_out)
            new_route.vid = vid
            new_sol.add_routine(new_route)
            new_sol.veh_route[vid] = len(new_sol.routines)-1

        for vid in new_sol.veh_route:
            if vid in fs.veh_route:
                fs.routines[fs.veh_route[vid]] = copy.deepcopy(new_sol.routines[new_sol.veh_route[vid]])
            else:
                fs.add_routine(new_sol.routines[new_sol.veh_route[vid]])
                fs.veh_route[vid] = len(fs.routines) - 1

        return output_text


# remove vehicles which have returned to the depot
def remove_return_vehicle(listener):
    veh_num = len(gc.vehicles)
    output_text = ""
    for i in range(veh_num):
        vid = gc.vehicles[i]
        try:
            traci.vehicle.getLength(vid)
        except:
            gc.vehicles[i] = None
            gc.total_serv_time += traci.simulation.getTime() - gc.vehicles_depart_time[vid]
            print("vehicle " + vid + " returned")
            print("current total_serv_time: "+str(gc.total_serv_time))
            output_text += "vehicle " + vid + " returned" + "\n"
            output_text += "current total_serv_time: "+str(gc.total_serv_time)+"\n"
    gc.vehicles = list(filter(lambda vid: vid != None, gc.vehicles))
    
    if len(output_text) > 0:
        f = open(listener.output_file, "a")
        f.write(output_text)
        f.close()

def is_all_not_in_juction():
    for v in gc.vehicles:
        lid = traci.vehicle.getLaneID(v)
        if (len(lid) == 0):
            return False
        if lid[0] == ":":
            return False
    return True

def is_all_start():
    for v in gc.vehicles:
        l = traci.vehicle.getDistance(v)
        if l < 0:
            return False
    return True

'''
getDistance Returns the distance to the starting point like an odometer.
getDrivingDistance Return the distance to the given edge and position along the vehicles route.
getDrivingDistance: To the start of the edge
'''
def set_task_dis_flag1():
    global fs, disflags
    disflags.clear()
    for vid in gc.vehicles:
        dd = traci.vehicle.getDrivingDistance(vid, depot_in.edge, 1)

        idx = fs.veh_route[vid]
        task_seqs = fs.routines[idx].tasks_seqs
        distance_for_task = []
        for i in range(1, len(task_seqs) - 1):
            # calculate the distance of each task to the incoming depot to help calculate tasks which have been served
            dd1 = dd - traci.vehicle.getDrivingDistance(vid, task_seqs[i].edge, 0)
            distance_for_task.append(dd1)
        distance_for_task.append(0)
        disflags.add_dis_flags(vid, distance_for_task)



class DCARPListener(traci.StepListener):
    
    def __init__(self, scenario, output_file):
        self.period = 1000
        self.detect_freq = 30
        self.count = 0      # the number of the current step
        self.flag1 = True   # check if all vehicles are starting?
        self.flag2 = False  # used if it couldn't reroute because of wrong vehicle positions, for travel cost change event
        self.flag3 = False  # used if it couldn't reroute because of wrong vehicle positions, for task(s) added event
        self.cost = [0]*5   # stores the cost in the last 5 examined steps
        self.fo = open("output/cost_change"+str(scenario)+".txt", "w")
        self.net_fig = None
        self.net_ax = None
        self.instance_nr = 0
        self.scenario = scenario
        self.output_file = output_file

    def step(self, t=0):
        self.count += 1
        output_text = ""
        remove_return_vehicle(self)
        self.hidden_served_task()
        # accumulate cost used
        
        if self.instance_nr >= 15:
            return True

        if self.flag1:
            flag2 = is_all_start()
            if flag2:
                # calculate the distance and distance flag
                set_task_dis_flag1()
                self.flag1 = False
            else:
                return True
        
        new_tasks_added_step = False

        # in every 30th step check the travel cost change
        if self.count % self.detect_freq == 0 or self.flag2:
            cost = self.detect_cost_change()

            if self.flag2 == False:
                t_now = traci.simulation.getTime()
                self.fo.write("{0},{1}\n".format(t_now, cost))

            self.cost.pop(0)
            self.cost.append(cost)            
            if self.cost[0]!=0 and self.cost[-1] > self.cost[0] * 1.05 or self.flag2:
                if not is_all_not_in_juction():
                    self.flag2 = True
                    return True

                # only the future cost!
                cost = self.detect_cost_change()
                t_now = traci.simulation.getTime()
                #self.fo.write("{0},{1}\n".format(t_now, cost))
                print("{0},{1}\n".format(t_now, cost))
                output_text = str(t_now)+" "+str(cost)+"\n"

                self.instance_nr += 1
                print("\nscenario: ", self.scenario, "instance: ", self.instance_nr, "current time:", traci.simulation.getTime())
                output_text += "\nscenario: "+str(self.scenario)+" instance: "+str(self.instance_nr)+" current time:"+str(traci.simulation.getTime())+"\n"

                # if there are also new task(s), add them
                if (self.count % 60 == 0 or self.flag3) and sumo_dcarp_init.added_task_count < 10:
                    self.flag3 = False
                    new_tasks_added_step = True
                    output_text += sumo_dcarp.reschedule(self.scenario, self.instance_nr, new_tasks_added=True)
                else:
                    output_text += sumo_dcarp.reschedule(self.scenario, self.instance_nr)

                cost = self.detect_cost_change()
                t_now = traci.simulation.getTime()
                #self.fo.write("optimise,{0},{1}\n".format(t_now, cost))
                print("optimise,{0},{1}\n".format(t_now, cost))
                output_text += "optimise, "+str(t_now)+" "+str(cost)+"\n"

                self.flag1 = True
                self.flag2 = False
              
        # add new task in every minute
        if (self.count % 60 == 0 or self.flag3) and sumo_dcarp_init.added_task_count < 10 and not new_tasks_added_step:
            if not is_all_not_in_juction():
                self.flag3 = True
                return True

            self.instance_nr += 1
            print("\nscenario: ", self.scenario, "instance: ", self.instance_nr, "current time:", traci.simulation.getTime())
            output_text += "\nscenario: "+str(self.scenario)+" instance: "+str(self.instance_nr)+" current time:"+str(traci.simulation.getTime())+"\n"
            output_text += sumo_dcarp.reschedule(self.scenario, self.instance_nr, new_tasks_added=True)

            self.flag1 = True
            self.flag3 = False
        
        if len(output_text) > 0:
            f = open(self.output_file, "a")
            f.write(output_text)
            f.close()

        return True

    def detect_cost_change(self):
        # calculate cost and draw in the window
        remove_return_vehicle(self)
        cost_future = 0
        for vid in gc.vehicles:
            route_edges = traci.vehicle.getRoute(vid)
            curr_idx = traci.vehicle.getRouteIndex(vid)
            vcost = 0
            for eid in route_edges[curr_idx+1:]:
                vcost += float(traci.vehicle.getParameter(vid, "device.rerouting.edge:"+eid))
            cost_future += vcost 
            #print(vid, vcost)
        #print("cost_future", cost_future)
        return cost_future
    
    def vis_route_in_gui(self):
        route_edges_nv1 = traci.vehicle.getRoute('nv1')
        for e in route_edges_nv1:
            traci.gui.toggleSelection(e,  objType='edge')
    
    def hidden_served_task(self):
        for vid in gc.vehicles:
            e = traci.vehicle.getRoadID(vid)
            if e == '':
                continue
            if traci.gui.isSelected(e, objType='edge'):
                traci.gui.toggleSelection(e, objType='edge')


png_index = -1
def route_visualization(route_edges, fig=None, ax=None):
    global png_index
    png_index += 1
    args = ["-n", "scenario/DCC.net.xml", "--xlim", "-100,7200", "--ylim", "-100,5400", "--xticks", "-100,7201,2000,16", "--yticks", "-100,5401,1000,16", "--selected-width", "2", "--edge-width", ".5", "--edge-color", "#606060", "--selected-color", "#800000", "--online", "on"] #"-o", "output/route_for_vid1_"+str(png_index)+".png",
    n_fig, n_ax = plot_route(route_edges, args, fig, ax)
    return n_fig, n_ax


# main program
with open('traffic/dublin.pickle', 'rb') as f:
    map_data = pickle.load(f)

node_dict = map_data[0]     # node_dict[node] = index
node_list = map_data[1]     # node_list[index] = node (edge from/to in DCC.net.xml)
edge_dict = map_data[2]     # edge_dict[edge] = id_index
edge_list = map_data[3]     # edge_list[id_index] = edge (edge id in DCC.net.xml)

task_list = list()
task_dict = dict()

# if (len(sys.argv) == 1):
#     raise ValueError("please input scenario index")
# scenario = int(sys.argv[1])

for scenario in range(1,13):
    for run in range(1,6):
        print("s"+str(scenario)+"_"+"r"+str(run)+".txt started")

        scenario_file = "dcarp/scenario{0}.xml".format(scenario)
        tree = ET.ElementTree(file=scenario_file)
        info = tree.getroot()
        time_setting = info.find("time")
        begin_time = time_setting.attrib["begin"]
        end_time = time_setting.attrib["end"]
        step_length = time_setting.attrib["step"]

        depot_setting = info.findall("depot")
        if (depot_setting[0].attrib["id"] == "incoming"):
            depot_in.set_depot(depot_setting[0].attrib["edge"])
            depot_out.set_depot(depot_setting[1].attrib["edge"])

        if (depot_setting[0].attrib["id"] == "outgoing"):
            depot_in.set_depot(depot_setting[1].attrib["edge"])
            depot_out.set_depot(depot_setting[0].attrib["edge"])


        cap_setting = info.findall("capacity")[0]
        gc.set_cap(int(cap_setting.attrib["value"])+1)
        gc.set_edge_map(edge_dict)

        net = sumolib.net.readNet("scenario/DCC.net.xml")
        depot_coord = net.getEdge(depot_out.edge).getFromNode().getCoord()
        # added "--quit-on-end", "true" to close SUMO window at the end
        traci.start(["sumo-gui", "-c", "scenario/DCC_simulation.sumo.cfg", "--begin", begin_time, "--end", end_time, "--step-length", step_length, "--start", "true", "--quit-on-end", "true"]) #, "--start", "true"
        traci.poi.add('depot', depot_coord[0],depot_coord[1], (1,0,0,0))
        gc.set_sim(traci.simulation)
        gc.begin_time = begin_time
        gc.total_serv_time = 0

        fs, disflags = sumo_dcarp_init.init(scenario_file)

        sumo_dcarp_init.new_tasks_whole = [2721, 554, 1001, 3617, 278, 3810, 5545, 2618, 2788, 5831]
        sumo_dcarp_init.busy_area = [2220, 6513, 4063, 1199, 1606, 540, 178, 4340, 3685, 3474]
        sumo_dcarp_init.not_busy_area = [498, 941, 5708, 5831, 4176, 1326, 1832, 5752, 5707, 6400]
        sumo_dcarp_init.added_task_count = 0


        # the above is the initial process

        listener = DCARPListener(scenario, "automated_outputs/s"+str(scenario)+"_"+"r"+str(run)+".txt")

        traci.addStepListener(listener)

        while len(gc.vehicles) > 0:
            remove_return_vehicle(listener)
            traci.simulationStep()
        print(str(traci.simulation.getTime()-1))

        f = open(listener.output_file, "a")
        f.write("\n"+str(traci.simulation.getTime()-1)+"\n")
        f.close()

        traci.close()
        listener.fo.close()

        print("s"+str(scenario)+"_"+"r"+str(run)+".txt finished")

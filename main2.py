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

from julia import Main
Main.include("utils/julia_utils.jl")

import utils.optimizer_utils as u

class sumo_dcarp_init():

    # pre-defined new tasks
    new_tasks_whole = [3810,5933,1001,5831,5917,1830,3728,278,2788,4590,4729,942,3617,4041,2618,554,5545,2981,2721,354]
    busy_area = [6513,5609,2056,540,3935,2220,1199,3474,4086,3685,4340,4200,3389,5355,2901,3399,1606,5114,178,4063]
    not_busy_area = [5708,941,1411,6288,5831,6400,1832,5752,2618,6582,1326,5081,6521,5707,5343,4176,483,2786,498,1361]
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
        reroute_done, ot = sumo_dcarp.parse_tasks_seq(scenario, instance_nr, added_tasks)
        output_text += ot

        if not reroute_done:
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
        event = "none"

        tasks = [0 for i in range(301)]
        arcs = dict()
        task_ids = set()
        virtual_task_ids = dict()

        # add depot task & arcs
        depot_from_node_id = node_dict[net.getEdge(depot_out.edge).getFromNode().getID()]
        depot_to_node_id = node_dict[net.getEdge(depot_out.edge).getToNode().getID()]
        tasks[0] = u.Task(0, 0, 0, 0)
        arcs[0] = u.Arc(depot_to_node_id, depot_from_node_id, 0)
        arcs[depot_out.edge] = u.Arc(depot_from_node_id, depot_to_node_id, u.inf)
        arcs[depot_in.edge] = u.Arc(depot_to_node_id, depot_from_node_id, u.inf)

        # add the other tasks and a solution
        k = 0
        remain_tasks_num = 0
        solution_seq = [0]
        current_time = int(traci.simulation.getTime())
        for vid in fs.veh_route:
            if vid not in gc.vehicles:
                continue
            r = fs.routines[fs.veh_route[vid]]
 
            # if all tasks in the route have been served and the remaining edges smaller than 10 edges. ignore this route. 
            route_edges = traci.vehicle.getRoute(vid)
            curr_idx = traci.vehicle.getRouteIndex(vid)
            if (len(route_edges) - curr_idx < 10) and len(r.tasks_seqs) == 3:
                arcs[r.tasks_seqs[1].edge] = \
                    u.Arc(node_dict[net.getEdge(r.tasks_seqs[1].edge).getFromNode().getID()], \
                        node_dict[net.getEdge(r.tasks_seqs[1].edge).getToNode().getID()], u.inf)
                continue

            for tsk in r.tasks_seqs[1:-1]:
                k += 1
                from_node = net.getEdge(tsk.edge).getFromNode().getID()
                to_node = net.getEdge(tsk.edge).getToNode().getID()
                cost = u.inf
                if tsk.vt:
                    virtual_task_ids[vid] = k
                    cost = 0 # current_time - gc.vehicles_depart_time[vid]
                    dummy_arc_id = "d_"+tsk.edge
                    tasks[k] = u.Task(edge_dict[tsk.edge], dummy_arc_id, cost, tsk.demand)
                    arcs[dummy_arc_id] = u.Arc(depot_from_node_id, node_dict[to_node], u.inf)
                else:
                    remain_tasks_num += 1
                    task_ids.add(k)
                    tasks[k] = u.Task(edge_dict[tsk.edge], tsk.edge, cost, tsk.demand)
                arcs[tsk.edge] = u.Arc(node_dict[from_node], node_dict[to_node], u.inf)
                solution_seq.append(k)
            solution_seq.append(0)
        
        new_tasks_ids = list()
        if (len(added_tasks) > 0):
            event = "new_tasks"
            solution_seq_wo_new_tasks = copy.copy(solution_seq)
            for tsk in added_tasks:
                k += 1
                new_tasks_ids.append(k)
                tsk_id = edge_list[tsk]
                from_node = net.getEdge(tsk_id).getFromNode().getID()
                to_node = net.getEdge(tsk_id).getToNode().getID()
                tsk_demand = int(net.getEdge(tsk_id).getLength() / 10)
                remain_tasks_num += 1
                arcs[tsk_id] = u.Arc(node_dict[from_node], node_dict[to_node], u.inf)
                tasks[k] = u.Task(tsk, tsk_id, u.inf, tsk_demand)
                task_ids.add(k)
                solution_seq.append(k)
            solution_seq.append(0)
        
        # load file and calculate min_cost

        folder = "/xml/scenario{0}_instance{1}".format(scenario, instance_nr)
        cost_file_path = folder + "/result.rou.alt.xml"
        instance_file_path = folder + "/instance.txt"
        output_file_path = folder + "/result.txt"

        # print instance into file
        with open(instance_file_path, 'w') as f:
            f.write("capacity\t"+str(gc.capacity)+"\n")
            f.write("tasks\t"+str(k+1)+"\n")
            for task_index in range(k+1):
                f.write(str(task_index)+"\t"+str(tasks[task_index].id)+"\t"+str(tasks[task_index].arc_id)+"\t"+str(tasks[task_index].serv_cost)+"\t"+str(tasks[task_index].demand)+"\n")
            f.write("arcs\t"+str(len(arcs))+"\n")
            for arc_id, arc in arcs.items():
                f.write(str(arc_id)+"\t"+str(arc.head_node)+"\t"+str(arc.tail_node)+"\t"+str(arc.trav_cost)+"\n")
            f.write("virtual_task_ids\t"+str(list(virtual_task_ids.values()))+"\n")
            f.write("solution_sequence\t"+str(solution_seq)+"\n")
            f.write("event\t"+event+"\n")
            if event == "new_tasks":
                f.write(str(solution_seq_wo_new_tasks)+"\n")
                f.write(str(new_tasks_ids))

        reroute_done = False
        if remain_tasks_num > 3:
            reroute_done = True
            algo_output_text, n_seq, n_r_seg, n_r_seg_load, n_tc = Main.main_function(instance_file_path, cost_file_path, output_file_path)
            # fix indexing
            for i in range(len(n_r_seg)):
                n_r_seg[i] = (n_r_seg[i][0]-1, n_r_seg[i][1]-1)

            output_text += "\n".join(algo_output_text) + "\n"

            # load new solution and assign to the vehicle (new_solution.xml)
            #output_text += sumo_dcarp.load_new_schedule(instance, new_solution)

            new_sol = solution()
            for r in n_r_seg:
                new_route = routine()
                route_edges = []
                vt = False
                vid = None
                
                first_task_id = n_seq[r[0]+1]
                #print(str(first_task_id), str(instance.tasks[first_task_id].head_node), str(instance.tasks[first_task_id].tail_node))
                e = edge_list[tasks[first_task_id].id]
                # if the first task is a virtual task
                if first_task_id in virtual_task_ids.values():
                    for v_id, vt_id in virtual_task_ids.items():
                        if vt_id == first_task_id:
                            vid = v_id
                    vt = True
                # assign a new vehicle
                else:
                    print("new vehicle needed")
                    output_text += "new vehicle needed\n"
                    vid = "nv" + str(int(gc.vehicles[-1][2:])+1)
                    route_edges.append(depot_out.edge)
                    gc.add_vehicle(vid, traci.simulation.getTime()+1)
                    gc.set_veh_load(vid)
                    demand = tasks[first_task_id].demand
                    new_route.add_tasks(task(e, demand))
            
                route_edges.append(e)

                for task_id in n_seq[r[0]+2:r[1]]:
                    e = edge_list[tasks[task_id].id]
                    demand = tasks[task_id].demand
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
                    fs.routines[fs.veh_route[vid]] = copy.copy(new_sol.routines[new_sol.veh_route[vid]])
                else:
                    fs.add_routine(new_sol.routines[new_sol.veh_route[vid]])
                    fs.veh_route[vid] = len(fs.routines) - 1

            print("\nRerouting complete! :)\n")
            output_text += "Rerouting complete! :)\n"

        return reroute_done, output_text


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
        
#       if self.instance_nr >= 15:
#           return True

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

for scenario in range(11,12):    #(1,13)
    for run in range(5,6):     #(1,11)
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

        sumo_dcarp_init.new_tasks_whole = [3810,5933,1001,5831,5917,1830,3728,278,2788,4590,4729,942,3617,4041,2618,554,5545,2981,2721,354]
        sumo_dcarp_init.busy_area = [6513,5609,2056,540,3935,2220,1199,3474,4086,3685,4340,4200,3389,5355,2901,3399,1606,5114,178,4063]
        sumo_dcarp_init.not_busy_area = [5708,941,1411,6288,5831,6400,1832,5752,2618,6582,1326,5081,6521,5707,5343,4176,483,2786,498,1361]
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

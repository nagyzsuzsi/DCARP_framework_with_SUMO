using Random
using StatsBase
using TickTock
using Combinatorics
using EzXML

inf = 2^31-1
time_limit = 60
#optimal_solution_tc = 5018 #3548
node_num = 2896

# HMA
alpha = 0.6
psize = 10
#treshold_ratio::Vector{Float16} = [0.003, 0.004, 0.005, 0.006]
treshold_ratio::Vector{Float16} = [0.0001, 0.0005, 0.0010, 0.0015]	# only for egl-g set
W = 10		# the number of non-improving attractors visited in RTTP

mutable struct Arc
	const head_node::Int16
	const tail_node::Int16
	trav_cost::Int32
	
	function Arc(head_node, tail_node, trav_cost)
		new(head_node, tail_node, trav_cost)
	end
end

# Base.show(io::IO, x::Arc) = print("(", x.head_node, ", ", x.tail_node, ")\t", x.trav_cost)

mutable struct Task
	const task_id::Int32
	const arc_id::String
	serv_cost::Int16
	demand::Int16
	
	function Task(task_id, arc_id, serv_cost, demand)
		new(task_id, arc_id, serv_cost, demand)
	end
end

# Base.show(io::IO, x::Task) = print(x.id, "\t", x.inv_id,"\t(", x.arc.head_node, ", ", x.arc.tail_node, ")\t", x.arc.trav_cost, "\t", x.serv_cost, "\t", x.demand)

mutable struct Instance
	capacity::Int16
	task_ids::Set{Int16}
	virtual_task_ids::Set{Int16}
	dummy_arc::Union{Arc, Nothing}
	dummy_task::Union{Task, Nothing}	
	arcs::Dict{String, Arc}
	tasks::Vector{Task}
	min_cost::Matrix{Int64}
	
	function Instance()
		new(0, Set{Int16}(), Set{Int16}(), nothing, nothing, Dict{String, Arc}(), Vector{Task}(), fill(-1, (node_num, node_num)))
	end
	
end

mutable struct Individual
	sequence::Union{Vector{Int16}, Nothing}
	route_seg::Union{Vector{Tuple{Int16,Int16}}, Nothing}
	route_seg_load::Union{Vector{Int32}, Nothing}
	total_cost::Int32
	age::Int8	# -1: newly generated (needs merge split), 0: can be selected for further improvement, 1: needs to be replaced
	
	function Individual()
		new(nothing, nothing, nothing, inf, 0)
	end
	
	# for copying (age is not copied)
	function Individual(sequence, route_seg, route_seg_load, total_cost)
		new(sequence, route_seg, route_seg_load, total_cost, 0)
	end	
end

global global_best_solution::Individual = Individual()
global output_dump::Vector{String} = Vector{String}()

Base.copy(i::Individual) = Individual(i.sequence, i.route_seg, i.route_seg_load, i.total_cost)

Base.show(io::IO, x::Individual) = print(x.sequence, "\n", x.route_seg,"\t", x.route_seg_load, "\t", x.total_cost, "\t", x.age)

function is_feasible(instance::Instance, individual::Individual)::Bool
	# if length(route_seg_load[route_seg_load .> instance.capacity]) > 0
	# check if there is excess load
	for load in individual.route_seg_load
		if load > instance.capacity
			return false
		end
	end
	# check if there are virtual tasks at wrong places
	for rs in individual.route_seg
		if length(intersect(individual.sequence[rs[1]+2:rs[2]-1], instance.virtual_task_ids)) != 0
			return false
		end
	end
	return true
end

function update_global_best_solution(instance::Instance, new_individual::Individual, print_text::Any="")
	if is_newer_individual_better(global_best_solution, new_individual) && is_feasible(instance, new_individual) && peektimer() <= time_limit
		global global_best_solution = copy(new_individual)
		push!(output_dump, string(global_best_solution.total_cost, "\t", print_text, "\t", peektimer()))
		#println(string(global_best_solution.total_cost, "\t", print_text, "\t", peektimer()))
	end
end

function import_from_file(instance::Instance, txt_file::String)
	initial_sequence, initial_sequence_wo_new_tasks, new_tasks = Vector{Int16}(), Vector{Int16}(), Vector{Int16}()
	open(txt_file) do f
		while ! eof(f)
			x = readline(f)
			
			if occursin("capacity\t", x)
				instance.capacity = parse(Int16, split(x, "\t")[2])
			
			elseif occursin("tasks\t", x)
				task_num = parse(Int16, split(x, "\t")[2])-1
				instance.tasks = Vector{Task}(undef, task_num)
				instance.task_ids = Set{Int16}(1:task_num)		
				task_index, task_id, arc_id, serv_cost, demand = split(readline(f), "\t")
				instance.dummy_task = Task(parse(Int32, task_id), arc_id, parse(Int16, serv_cost), parse(Int16, demand))
				for id in 1:task_num
					task_index, task_id, arc_id, serv_cost, demand = split(readline(f), "\t")
					instance.tasks[id] = Task(parse(Int32, task_id), string(arc_id), parse(Int16, serv_cost), parse(Int16, demand))
				end
			
			elseif occursin("arcs\t", x)
				total_arc_num = parse(Int16, split(x, "\t")[2])-1
				arc_id, head_node, tail_node, trav_cost = split(readline(f), "\t")
				instance.dummy_arc = Arc(parse(Int16, head_node), parse(Int16, tail_node), parse(Int16, trav_cost))
				for i in 1:total_arc_num
					arc_id, head_node, tail_node, trav_cost = split(readline(f), "\t")
					instance.arcs[string(arc_id)] = Arc(parse(Int16, head_node), parse(Int16, tail_node), parse(Int32, trav_cost))					
				end
			
			elseif occursin("virtual_task_ids\t", x)
				seq = string(split(x, "\t")[2])
				instance.virtual_task_ids = Set{Int16}(parse.(Int16, split(seq[2:end-1], ", ")))
				instance.task_ids = setdiff(instance.task_ids, instance.virtual_task_ids)

			elseif occursin("solution_sequence\t", x)
				seq = string(split(x, "\t")[2])
				initial_sequence = parse.(Int16, split(seq[2:end-1], ", "))
				initial_sequence_wo_new_tasks = copy(initial_sequence)
			
			elseif occursin("event\t", x)
				event = split(x, "\t")[2]
				if event == "new_tasks"
					initial_sequence_wo_new_tasks = parse.(Int16, split(string(readline(f))[2:end-1], ", "))
					new_tasks = parse.(Int16, split(string(readline(f))[2:end-1], ", "))
				end
			
			end
			
		end
	end

	return initial_sequence, initial_sequence_wo_new_tasks, new_tasks
end

function calculate_shortest_paths(instance::Instance, file_path::String)
	for i in 1:node_num
		instance.min_cost[i, i] = 0
	end	
	if isfile(file_path)
		tree = readxml(file_path)
		tree_root = root(tree)
		for vehicle in eachelement(tree_root), routeDistribution in eachelement(vehicle), route in eachelement(routeDistribution)
			cost = trunc(Int32, parse(Float16, route["cost"]))
			edges = split(route["edges"], " ")
			from_edge = edges[1]
			to_edge = edges[end]
			head = instance.arcs[from_edge].head_node
			tail = instance.arcs[from_edge].tail_node
			if from_edge == to_edge
				instance.min_cost[head, tail] = cost
				instance.arcs[from_edge].trav_cost = cost
			else
				head_to = instance.arcs[to_edge].head_node
				tail_to = instance.arcs[to_edge].tail_node
				if instance.min_cost[head, head_to] < 0
					instance.min_cost[head, head_to] = cost - instance.min_cost[head_to, tail_to]
				end
				if instance.min_cost[head, tail_to] < 0
					instance.min_cost[head, tail_to] = cost
				end
				if instance.min_cost[tail, head_to] < 0
					instance.min_cost[tail, head_to] = cost - instance.min_cost[head, tail] - instance.min_cost[head_to, tail_to]
				end
				if instance.min_cost[tail, tail_to] < 0
					instance.min_cost[tail, tail_to] = cost - instance.min_cost[head, tail]
				end
			end
		end
	else
		println("Error: File not foud!")
	end
	
	for i in 1:node_num, j in 1:node_num
		if instance.min_cost[i, j] == -1
			instance.min_cost[i, j] = inf
		end
	end

	for task_id in collect(instance.task_ids)
		if !(task_id in instance.virtual_task_ids)
			instance.tasks[task_id].serv_cost = instance.min_cost[instance.arcs[instance.tasks[task_id].arc_id].head_node, instance.arcs[instance.tasks[task_id].arc_id].tail_node]
			instance.arcs[instance.tasks[task_id].arc_id].trav_cost = instance.tasks[task_id].serv_cost
		else
			instance.arcs[instance.tasks[task_id].arc_id].trav_cost = inf
		end
	end

end

global algorihms = ["RR1", "ABC", "HMA"]

function main_function(instance_file_path::String, cost_file_path::String, output_file_path::String)
	instance = Instance()
	global global_best_solution = Individual()
	global output_dump = Vector{String}()
	initial_sequence, initial_sequence_wo_new_tasks, new_tasks = import_from_file(instance, instance_file_path)
	calculate_shortest_paths(instance, cost_file_path)
	initial_individual = individual_from_sequence(instance, initial_sequence)

	#=
	# RR1
	tick()
	if is_feasible(instance, initial_individual)
		if length(new_tasks) > 0
			reroute_one_route(instance, individual_from_sequence(instance, initial_sequence), individual_from_sequence(instance, initial_sequence_wo_new_tasks), new_tasks)
		else
			global global_best_solution = individual_from_sequence(instance, initial_sequence)
		end
		#rr1_solution = copy(global_best_solution)
	else
		push!(output_dump, "Error: infeasible initial sequence")
	end
	tock()
	
	# ABC
	# global global_best_solution = Individual()
	tick()
	abc(instance)
	#abc_solution = copy(global_best_solution)
	tock()
	=#
	
	# HMA
	# global global_best_solution = Individual()
	tick()
	hma(instance)
	# hma_solution = copy(global_best_solution)
	tock()

	#=
	all_solutions = [rr1_solution, abc_solution, hma_solution]
	global global_best_solution = get_individual_with_lowest_total_cost(all_solutions)
	winner_algorighm = algorihms[findall(x -> x==global_best_solution, all_solutions)[1]]
	push!(output_dump, string("\n", winner_algorighm, "\n"))
	=#
	
	open(output_file_path, "w") do io
		for line in output_dump
			println(io, line)
		end
	end

	return output_dump, global_best_solution.sequence, global_best_solution.route_seg, global_best_solution.route_seg_load, global_best_solution.total_cost
end

function individual_from_sequence(instance::Instance, sequence::Vector{Int16})::Individual
	individual = Individual()
	individual.sequence = sequence
	individual.route_seg = find_route_segments(individual.sequence)
	individual.route_seg_load = calculate_route_segments_load(instance, individual.sequence, individual.route_seg)
	individual.total_cost = calculate_total_cost(instance, individual.sequence)
	return individual
end

function get_individual_with_lowest_total_cost(individual_list::Vector{Individual})::Individual
	return individual_list[argmin([i.total_cost for i in individual_list])]
end

function get_arc(instance::Instance, task_id::Int16)::Arc
	return task_id == 0 ? instance.dummy_arc : instance.arcs[instance.tasks[task_id].arc_id]
end

function get_task_ids_from_sequence(sequence::Vector{Int16})::Vector{Int16}
	return filter(x -> x != 0, sequence)
end

function find_route_segments(sequence::Vector{Int16})::Vector{Tuple{Int16,Int16}}
	route_seg = Vector{Tuple{Int16,Int16}}()
	i = 1
	while i < length(sequence)
		start_index = i
        i += 1
        while sequence[i] != 0
			i +=1
        end
		end_index = i
		push!(route_seg, (start_index, end_index))
	end
	return route_seg
end

function calculate_route_segments_load(instance::Instance, sequence::Vector{Int16}, route_seg::Vector{Tuple{Int16,Int16}})::Vector{Int32}
	route_seg_load = Vector{Int32}()
	for r in route_seg
		push!(route_seg_load, sum([instance.tasks[sequence[i]].demand for i in r[1]+1:r[2]-1]))
	end
	return route_seg_load
end

function calculate_total_cost(instance::Instance, sequence::Vector{Int16})::Int32
	total_cost = 0
	for i in 1:(length(sequence)-1)
		serv_cost = sequence[i] == 0 ? 0 : instance.tasks[sequence[i]].serv_cost
		tail_node = sequence[i] == 0 ? instance.dummy_arc.tail_node : instance.arcs[instance.tasks[sequence[i]].arc_id].tail_node
		head_node = sequence[i+1] == 0 ? instance.dummy_arc.head_node : instance.arcs[instance.tasks[sequence[i+1]].arc_id].head_node
		if total_cost < inf
			total_cost = min(total_cost + serv_cost + instance.min_cost[tail_node, head_node], inf)
		else
			return inf
		end
	end
	return total_cost >= inf ? inf : total_cost
end

function calculate_excess_demand(instance::Instance, route_seg_load::Vector{Int32})::Int32
	excess_demand = 0
	for load in route_seg_load
		if load > instance.capacity
			excess_demand += (load - instance.capacity)
		end
	end
	return excess_demand
end

function get_task_id_index_in_sequence(instance::Instance, sequence::Vector{Int16}, task_id::Union{Int16, Nothing}=nothing, task_ids::Union{Vector{Int16}, Nothing}=nothing)::Tuple{Int16, Int16}
	if task_id === nothing
		task_ids = task_ids === nothing ? get_task_ids_from_sequence(sequence) : task_ids
		task_id = rand(task_ids)
	end
	index = findall(x -> x==task_id, sequence)[1]
	return task_id, index
end

function get_route_segment_index(route_seg::Vector{Tuple{Int16,Int16}}, task_index::Int16)::Int16
	for (i, r) in enumerate(route_seg)
		if task_index > r[1] && task_index < r[2]
			return i
		end
	end
end

function random_routing_plan_generator(instance::Instance)::Individual
	t_ids = shuffle(collect(instance.task_ids))
	vt_ids = shuffle(collect(instance.virtual_task_ids))
	ind_seq = Vector{Int16}([0])

	route_load::Int32 = 0
	if length(vt_ids) > 0
		vt_id = pop!(vt_ids)
		push!(ind_seq, vt_id)
		route_load = instance.tasks[vt_id].demand
	end
	while length(t_ids) > 0
		t_id = pop!(t_ids)
		if route_load + instance.tasks[t_id].demand > instance.capacity
			push!(ind_seq, 0)
			route_load = 0
			while length(vt_ids) > 0
				vt_id = pop!(vt_ids)
				push!(ind_seq, vt_id)
				route_load = instance.tasks[vt_id].demand
				if route_load + instance.tasks[t_id].demand <= instance.capacity
					break
				end
				push!(ind_seq, 0)
				route_load = 0
			end
		end
		push!(ind_seq, t_id)
		route_load += instance.tasks[t_id].demand
	end
	push!(ind_seq, 0)
	while length(vt_ids) > 0
		push!(ind_seq, pop!(vt_ids))
		push!(ind_seq, 0)
	end

	individual = individual_from_sequence(instance, ind_seq)
	individual.age = -1
	
	return individual
end

function randomized_path_scanning_heuristic(instance::Instance, task_ids::Set{Int16}):Individual
	sequences::Vector{Vector{Int16}} = fill([0], 5)
	new_individuals = Vector{Individual}(undef, 5)
	
	for i in 1:5
		unserved_vt_ids = intersect(task_ids, instance.virtual_task_ids)
		unserved_task_ids = setdiff(task_ids, unserved_vt_ids)
		current_route_load::Int32 = 0
		if length(unserved_vt_ids) > 0
			vt_id = pop!(unserved_vt_ids)
			push!(sequences[i], vt_id)
			current_route_load = instance.tasks[vt_id].demand
		end
		while length(unserved_task_ids) > 0
			servable_task_ids = filter(task_id -> current_route_load + instance.tasks[task_id].demand <= instance.capacity, unserved_task_ids)
			
			if length(servable_task_ids) == 0
                push!(sequences[i], 0)
				if length(unserved_vt_ids) > 0
					vt_id = pop!(unserved_vt_ids)
					push!(sequences[i], vt_id)
					current_route_load = instance.tasks[vt_id].demand
				else
					current_route_load = 0			
				end
			elseif length(servable_task_ids) == 1
				task_id = pop!(servable_task_ids)
				push!(sequences[i], task_id)
				current_route_load += instance.tasks[task_id].demand
				delete!(unserved_task_ids, task_id)
			
			else
				tasks_min_cost_task_ids = Vector{Int16}()
				tasks_min_cost = inf
				last_task_id_arc = get_arc(instance, sequences[i][end])
				while length(servable_task_ids) > 0
					task_id = pop!(servable_task_ids)
					cost = instance.min_cost[last_task_id_arc.tail_node, instance.arcs[instance.tasks[task_id].arc_id].head_node]
					if cost < tasks_min_cost
						tasks_min_cost_task_ids = Vector{Int16}([task_id])
						tasks_min_cost = cost
					elseif cost == tasks_min_cost
						push!(tasks_min_cost_task_ids, task_id)
					end
				end
				
				if length(tasks_min_cost_task_ids) == 1
					task_id = pop!(tasks_min_cost_task_ids)
					push!(sequences[i], task_id)
					current_route_load += instance.tasks[task_id].demand
					delete!(unserved_task_ids, task_id)
				else
					best_value_task_ids = Vector{Int16}()
					best_value::Union{Number, Nothing} = nothing
					
					# 1) maximize the distance from the head of task to the depot
					if i == 1
						best_value = 0
						for task_id in tasks_min_cost_task_ids
							value = instance.min_cost[instance.arcs[instance.tasks[task_id].arc_id].head_node, instance.dummy_arc.head_node]
							if value > best_value
								best_value = value
                                best_value_task_ids = Vector{Int16}([task_id])
							elseif value == best_value
								push!(best_value_task_ids, task_id)
							end
						end
					end
					
					# 2) minimize the distance from the head of task to the depot
					if i == 2
						best_value = inf
						for task_id in tasks_min_cost_task_ids
							value = instance.min_cost[instance.arcs[instance.tasks[task_id].arc_id].head_node, instance.dummy_arc.head_node]
							if value < best_value
								best_value = value
                                best_value_task_ids = Vector{Int16}([task_id])
							elseif value == best_value
								push!(best_value_task_ids, task_id)
							end
						end
					end
					
					# 3) maximize the term dem(t)/sc(t)
					if i == 3
						best_value = 0
						for task_id in tasks_min_cost_task_ids
							value = instance.tasks[task_id].serv_cost > 0 ? instance.tasks[task_id].demand / instance.tasks[task_id].serv_cost : instance.tasks[task_id].demand / 0.001
							if value > best_value
								best_value = value
                                best_value_task_ids = Vector{Int16}([task_id])
							elseif value == best_value
								push!(best_value_task_ids, task_id)
							end
						end
					end
					
					# 4) minimize the term dem(t)/sc(t)
					if i == 4
						best_value = inf
						for task_id in tasks_min_cost_task_ids
							value = instance.tasks[task_id].serv_cost > 0 ? instance.tasks[task_id].demand / instance.tasks[task_id].serv_cost : instance.tasks[task_id].demand / 0.001
							if value < best_value
								best_value = value
                                best_value_task_ids = Vector{Int16}([task_id])
							elseif value == best_value
								push!(best_value_task_ids, task_id)
							end
						end
					end
					
					# 5) use rule 1) if the vehicle is less than halffull, otherwise use rule 2)
					if i == 5
						if current_route_load < instance.capacity * 0.5
							best_value = 0
							for task_id in tasks_min_cost_task_ids
								value = instance.min_cost[instance.arcs[instance.tasks[task_id].arc_id].head_node, instance.dummy_arc.head_node]
								if value > best_value
									best_value = value
									best_value_task_ids = Vector{Int16}([task_id])
								elseif value == best_value
									push!(best_value_task_ids, task_id)
								end
							end
						else
							best_value = inf
							for task_id in tasks_min_cost_task_ids
								value = instance.min_cost[instance.arcs[instance.tasks[task_id].arc_id].head_node, instance.dummy_arc.head_node]
								if value < best_value
									best_value = value
									best_value_task_ids = Vector{Int16}([task_id])
								elseif value == best_value
									push!(best_value_task_ids, task_id)
								end
							end
						end
					end
					
					task_id = rand(best_value_task_ids)
					push!(sequences[i], task_id)
					current_route_load += instance.tasks[task_id].demand
					delete!(unserved_task_ids, task_id)
				end
			end
		end
		push!(sequences[i], 0)

		while length(unserved_vt_ids) > 0
			push!(sequences[i], pop!(unserved_vt_ids))
			push!(sequences[i], 0)
		end

		new_individuals[i] = individual_from_sequence(instance, sequences[i])
		#print(new_individuals[i].total_cost)
		#print("\t")
	end
	#println()
	
	return get_individual_with_lowest_total_cost(new_individuals)
end

function insert(instance::Instance, individual::Individual, task_id_1::Union{Int16, Nothing}=nothing, task_id_2::Union{Int16, Nothing}=nothing, only_feasible::Bool=true)::Individual
	new_sequence_temp = copy(individual.sequence)

	# select and remove task_id_1
	task_id_1, index_1 = get_task_id_index_in_sequence(instance, new_sequence_temp, task_id_1)
	deleteat!(new_sequence_temp, index_1)
	
	# if task_id_1 was the only task in the route plan -> remove the excess 0
	if new_sequence_temp[index_1-1] == 0 && new_sequence_temp[index_1] == 0
		deleteat!(new_sequence_temp, index_1)
	end
	
	# select task_id_2
	task_id_2, index_2 = get_task_id_index_in_sequence(instance, new_sequence_temp, task_id_2)
	
	# insert task_id_1 before & after task_id_2
	new_sequence_1 = copy(new_sequence_temp)
	insert!(new_sequence_1, index_2, task_id_1)
	
	new_route_seg = find_route_segments(new_sequence_1)
	new_route_seg_load = calculate_route_segments_load(instance, new_sequence_1, new_route_seg)
	new_individual_1 = Individual(new_sequence_1, new_route_seg, new_route_seg_load, calculate_total_cost(instance, new_sequence_1))

	# check feasibility
	if !is_feasible(instance, new_individual_1) && only_feasible
		return individual
	end
	
	new_sequence_2 = copy(new_sequence_temp)
	insert!(new_sequence_2, index_2+1, task_id_1)
	new_individual_2 = Individual(new_sequence_2, new_route_seg, new_route_seg_load, calculate_total_cost(instance, new_sequence_2))

	return get_individual_with_lowest_total_cost([new_individual_1, new_individual_2])
end

function double_insert(instance::Instance, individual::Individual, task_id_1::Union{Int16, Nothing}=nothing, task_id_2::Union{Int16, Nothing}=nothing, only_feasible::Bool=true)::Individual
	new_sequence_temp = copy(individual.sequence)
	
	# select task_id_1 and the next task
	task_id_1, index_1 = get_task_id_index_in_sequence(instance, new_sequence_temp, task_id_1)
	task_ids = new_sequence_temp[index_1:index_1+1]
	
	# next task can't be depot or task_id_2
	if task_ids[2] == 0 || task_ids[2] == task_id_2
		return individual
	end
	
	# remove the tasks
	deleteat!(new_sequence_temp, index_1)
	deleteat!(new_sequence_temp, index_1)

	# if the tasks were the only tasks in the route plan -> remove the excess 0
	if new_sequence_temp[index_1-1] == 0 && new_sequence_temp[index_1] == 0
		deleteat!(new_sequence_temp, index_1)
	end
	
	# select task_id_2
	task_id_2, index_2 = get_task_id_index_in_sequence(instance, new_sequence_temp, task_id_2)
	# println(task_id_2)
	
	# insert the tasks before & after task_id_2
	new_sequence_1 = copy(new_sequence_temp)
	splice!(new_sequence_1, index_2:index_2-1, task_ids)
	
	new_route_seg = find_route_segments(new_sequence_1)
	new_route_seg_load = calculate_route_segments_load(instance, new_sequence_1, new_route_seg)
	new_individual_1 = Individual(new_sequence_1, new_route_seg, new_route_seg_load, calculate_total_cost(instance, new_sequence_1))
	
	# check feasibility
	if !is_feasible(instance, new_individual_1) && only_feasible
		return individual
	end
	
	new_sequence_2 = copy(new_sequence_temp)
	splice!(new_sequence_2, index_2+1:index_2, task_ids)
	new_individual_2 = Individual(new_sequence_2, new_route_seg, new_route_seg_load, calculate_total_cost(instance, new_sequence_2))
	
	return get_individual_with_lowest_total_cost([new_individual_1, new_individual_2])
end

function swap(instance::Instance, individual::Individual, task_id_1::Union{Int16, Nothing}=nothing, task_id_2::Union{Int16, Nothing}=nothing, only_feasible::Bool=true)::Individual

	# select task_id_1
	task_id_1, index_1 = get_task_id_index_in_sequence(instance, individual.sequence, task_id_1)
	
	# select task_id_2
	task_id_2, index_2 = get_task_id_index_in_sequence(instance, individual.sequence, task_id_2)
	
	if task_id_1 == task_id_2
		return individual
	end
	
	new_sequence = copy(individual.sequence)
	new_sequence[index_1] = task_id_2
	new_sequence[index_2] = task_id_1
	new_individual = Individual(new_sequence, individual.route_seg, calculate_route_segments_load(instance, new_sequence, individual.route_seg), calculate_total_cost(instance, new_sequence))
	
	# check feasibility
	if !is_feasible(instance, new_individual) && only_feasible
		return individual
	else
		return new_individual
	end
end

function two_opt(instance::Instance, individual::Individual, task_id_1::Union{Int16, Nothing}=nothing, task_id_2::Union{Int16, Nothing}=nothing, only_feasible::Bool=true)::Individual
	new_individuals = Vector{Individual}()

	# select task_id_1
	task_ids = get_task_ids_from_sequence(individual.sequence)
	task_id_1, index_1 = get_task_id_index_in_sequence(instance, individual.sequence, task_id_1, task_ids)
	# println("$task_id_1\t$index_1")
	r_id_1 = get_route_segment_index(individual.route_seg, index_1)
	# println("$r_id_1")
	
	# select task_id_2
	task_ids = filter(x -> x != task_id_1, task_ids)
	task_id_2, index_2 = get_task_id_index_in_sequence(instance, individual.sequence, task_id_2, task_ids)
	# println("$task_id_2\t$index_2")
	r_id_2 = get_route_segment_index(individual.route_seg, index_2)
	# println("$r_id_2")

	# normally cannot happen
	if task_id_1 == task_id_2
		return individual
	end
	
	# one route
	if r_id_1 == r_id_2
		new_sequence = copy(individual.sequence)
		if index_2 < index_1
			index_1, index_2 = index_2, index_1
		end
		reverse!(new_sequence, index_1, index_2)
		
		return Individual(new_sequence, individual.route_seg, individual.route_seg_load, calculate_total_cost(instance, new_sequence))
	# two routes
	# cut r1 before task_id_1 & r2 after task_id_2
	else
		if index_1 > index_2
			index_1, index_2, r_id_1, r_id_2 = index_2, index_1, r_id_2, r_id_1
		end

		new_sequence_temp = reduce(append!, (individual.sequence[1:individual.route_seg[r_id_1][1]-1], individual.sequence[individual.route_seg[r_id_1][2]:individual.route_seg[r_id_2][1]], individual.sequence[individual.route_seg[r_id_2][2]+1:end]), init=Int16[])

		l1 = individual.sequence[individual.route_seg[r_id_1][1]+1:index_1-1]
		r1 = individual.sequence[index_1:individual.route_seg[r_id_1][2]-1]
		l2 = individual.sequence[individual.route_seg[r_id_2][1]+1:index_2]
		r2 = individual.sequence[index_2+1:individual.route_seg[r_id_2][2]-1]

		r1_inv = copy(r1)
		reverse!(r1_inv)
		l2_inv = copy(l2)
		reverse!(l2_inv)

		possible_combinations = [[(l1, r2), (l2, r1)], [(l1, l2_inv), (r1_inv, r2)]]

		for x in possible_combinations
			new_sequence = copy(new_sequence_temp)
			new_r1 = reduce(append!, (x[1][1], x[1][2], [0]), init=Int16[])
			new_r2 = reduce(append!, (x[2][1], x[2][2], [0]), init=Int16[])
			if length(new_r1) > 1
				append!(new_sequence, new_r1)
			end
			if length(new_r2) > 1
				append!(new_sequence, new_r2)
			end

			new_route_segments = find_route_segments(new_sequence)
			new_route_segments_load = calculate_route_segments_load(instance, new_sequence, new_route_segments)
			new_individual = Individual(new_sequence, new_route_segments, new_route_segments_load, calculate_total_cost(instance, new_sequence))

			if (is_feasible(instance, new_individual) && only_feasible) || !only_feasible
				push!(new_individuals, new_individual)
			end
		end
	end

	if length(new_individuals) > 0
		return get_individual_with_lowest_total_cost(new_individuals)
	else
		return individual
	end
end

p_rnd = rand()

function greedy_sub_tour_mutation(instance::Instance, individual::Individual, selected_task_id_1::Union{Int16, Nothing}=nothing, selected_task_id_2::Union{Int16, Nothing}=nothing)::Individual
	if selected_task_id_1 === nothing && selected_task_id_2 === nothing
		selected_route_id = rand(1:length(individual.route_seg))
	else
		selected_task_id_1, task_id_index_1 = get_task_id_index_in_sequence(instance, individual.sequence, selected_task_id_1)
		route_id_1 = get_route_segment_index(individual.route_seg, task_id_index_1)

		selected_task_id_2, task_id_index_2 = get_task_id_index_in_sequence(instance, individual.sequence, selected_task_id_2)
		route_id_2 = get_route_segment_index(individual.route_seg, task_id_index_2)

		if route_id_1 != route_id_2
			selected_task_id_2 = nothing
		end
		selected_route_id = route_id_1
	end
	
	vt_id = individual.sequence[individual.route_seg[selected_route_id][1]+1] in instance.virtual_task_ids ? individual.sequence[individual.route_seg[selected_route_id][1]+1] : 0
	
	# route without the last 0 (if there is a virtual task, then the first 0 is removed too)
    # route = [vt_id, t_1, t_2, ..., t_n] OR [0, t_1, t_2, ..., t_n]
	route = vt_id == 0 ? (individual.sequence[individual.route_seg[selected_route_id][1]:individual.route_seg[selected_route_id][2]-1]) : (individual.sequence[individual.route_seg[selected_route_id][1]+1:individual.route_seg[selected_route_id][2]-1])
	
	if length(route) < 4
		return individual
	end
	
	# fix values
    l_min = 2
    l_max = max(4, trunc(Int16, sqrt(length(route))))
    p_rc = 0.5
    p_cp = 0.8
    p_l = 0.2
    nl_max = 5
	
	if selected_task_id_1 === nothing && selected_task_id_2 === nothing
		# random values
		l = rand(l_min:l_max)
		sr_1 = rand(1:length(route))
		sr_2 = sr_1+l-1 <= length(route) ? sr_1+l-1 : sr_1+l-1-length(route)
	elseif selected_task_id_1 !== nothing && selected_task_id_2 === nothing
		l = rand(l_min:l_max)
		selected_task_id_1, sr_1 = get_task_id_index_in_sequence(instance, route, selected_task_id_1)
		sr_2 = sr_1+l-1 <= length(route) ? sr_1+l-1 : sr_1+l-1-length(route)
	else
		# given values
		selected_task_id_1, sr_1 = get_task_id_index_in_sequence(instance, route, selected_task_id_1)
		selected_task_id_2, sr_2 = get_task_id_index_in_sequence(instance, route, selected_task_id_2)
		l = sr_1 < sr_2 ? sr_2-sr_1+1 : length(route)-sr_1+sr_2+1
		
		if l < l_min || l > l_max
			return individual
		end
	end
	
	p_rnd = rand()
	
	route_wo_subroute = Vector{Int16}()
	subroute = Vector{Int16}()
	if sr_1 < sr_2
		subroute = route[sr_1:sr_2]
		route_wo_subroute = reduce(append!, (route_wo_subroute, route[1:sr_1-1], route[sr_2+1:end]), init=Int16[])
	elseif sr_2 < sr_1
		subroute = append!(route[sr_1:end], route[1:sr_2])
		route_wo_subroute = route[sr_2+1:sr_1-1]
	else
		println("sub-route plan operator error sr_1 ($sr_1) & sr_2 ($sr_2)")
	end

	if length(route_wo_subroute) == 0
		return individual
	end
	
	new_routes = Vector{Vector{Int16}}()		# max. 4
	new_route = Vector{Int16}()
	
	# greedy reconnection
	if p_rnd <= p_rc
		best_i_cost_diff = min(instance.min_cost[get_arc(instance, route_wo_subroute[end]).tail_node, get_arc(instance, subroute[1]).head_node] + instance.min_cost[get_arc(instance, subroute[end]).tail_node, get_arc(instance, route_wo_subroute[1]).head_node], inf)
        best_i = 1
		for i in 2:length(route_wo_subroute)
			cost_diff = min(instance.min_cost[get_arc(instance, route_wo_subroute[i-1]).tail_node, get_arc(instance, subroute[1]).head_node] + instance.min_cost[get_arc(instance, subroute[end]).tail_node, get_arc(instance, route_wo_subroute[i]).head_node], inf)
			if cost_diff < best_i_cost_diff
                best_i_cost_diff = cost_diff
                best_i = i
			end
			new_route = reduce(append!, (route_wo_subroute[1:best_i-1], subroute, route_wo_subroute[best_i:end]), init=Int16[])
			# println("greedy\t$new_route")
			push!(new_routes, new_route)
		end
	else
		# random distortion (rolling or mixing)
		if p_rnd <= p_cp
			w = sr_1 > sr_2 ? length(route_wo_subroute)+1 : sr_1
            new_route = copy(route_wo_subroute)
			while length(subroute) > 0
				rnd = rand()
				selected_task::Int16 = 0
				if rnd <= p_l
					index = rand(1:length(subroute))
					selected_task = subroute[index]
					deleteat!(subroute, index)
				else
					selected_task = pop!(subroute)
				end
				insert!(new_route, w, selected_task)
				w += 1
			end
			# println("distortion\t$new_route")
			push!(new_routes, new_route)
		
		# sub-route plan rotation
		else
			task_list_1 = copy(route)
            task_list_2 = copy(route)
			deleteat!(task_list_1, sr_1)
			deleteat!(task_list_2, sr_2)
            task_id_1 = route[sr_1]
            task_id_2 = route[sr_2]

			sr_1_tasks_dists = [instance.min_cost[get_arc(instance, task_id).tail_node, get_arc(instance, task_id_1).head_node] for task_id in task_list_1]
			sr_2_tasks_dists = [instance.min_cost[get_arc(instance, task_id_2).tail_node, get_arc(instance, task_id).head_node] for task_id in task_list_2]

			sr_1_nearest_tasks = task_list_1[sortperm(sr_1_tasks_dists)][1:min(nl_max, length(sr_1_tasks_dists))]
			sr_2_nearest_tasks = task_list_2[sortperm(sr_2_tasks_dists)][1:min(nl_max, length(sr_2_tasks_dists))]

			#sr_1_selected_task = rand(sr_1_nearest_tasks)
			#sr_2_selected_task = rand(sr_2_nearest_tasks)
			
			sr_1_selected_task = sr_1_nearest_tasks[1]
			sr_2_selected_task = sr_2_nearest_tasks[1]
			
			nl_sr_1 = findall(x -> x==sr_1_selected_task, route)[1]
			nl_sr_2 = findall(x -> x==sr_2_selected_task, route)[1]
			
			# task1	
			new_route_1 = Vector{Int16}()			
			if nl_sr_1 < sr_1
				new_route_1 = reverse(route, nl_sr_1, sr_1-1)
			elseif sr_1 < nl_sr_1
				sub_route = append!(route[nl_sr_1:end], route[1:sr_1-1])
				reverse!(sub_route)
				new_route_1 = append!(route[sr_1:nl_sr_1-1], sub_route)
			else
				println("sub-route plan operator error sr_1 ($sr_1) & nl_sr_1 ($nl_sr_1)")
			end
			if length(new_route_1) > 0
				push!(new_routes, new_route_1)
				# println("rotation\t$new_route_1")
			end
			
			# task2
			new_route_2 = Vector{Int16}()			
			if sr_2 < nl_sr_2
				new_route_2 = reverse(route, sr_2+1, nl_sr_2)
			elseif nl_sr_2 < sr_2
				sub_route = append!(route[sr_2+1:end], route[1:nl_sr_2])
				reverse!(sub_route)
				new_route_2 = append!(route[nl_sr_2+1:sr_2], sub_route)
			else
				println("sub-route plan operator error sr_2 ($sr_2) & nl_sr_2 ($nl_sr_2)")
			end
			if length(new_route_2) > 0
				push!(new_routes, new_route_2)
				# println("rotation\t$new_route_2")
			end

		end
	end
	
	# fix the new route plans (0 or vt_id may not be the first task in the new route plans)
    # note: vt_id is 0 if there is no vt in the route plan

	if length(new_routes) == 0
		# println("No new routes were found.")
		return(individual)
	end
	
	new_individuals = Vector{Individual}(undef, length(new_routes))	# max. 4

	ind_nr = 1
	for new_route in new_routes
		fixed_route = Vector{Int16}()

		i = 1
		while new_route[i] != vt_id
			push!(fixed_route, new_route[i])
			i += 1
		end
		fixed_route = append!(new_route[i+1:end], fixed_route)
		if vt_id != 0
			pushfirst!(fixed_route, vt_id)
		end

		new_individuals[ind_nr] = copy(individual)
		new_individuals[ind_nr].sequence = reduce(append!, (individual.sequence[1:individual.route_seg[selected_route_id][1]], fixed_route, individual.sequence[individual.route_seg[selected_route_id][2]:end]), init=Int16[])
		new_individuals[ind_nr].total_cost = calculate_total_cost(instance, new_individuals[ind_nr].sequence)
		
		ind_nr += 1
	end

	return get_individual_with_lowest_total_cost(new_individuals)
end

function merge_split(instance::Instance, individual::Individual, selected_route_ids::Union{Vector{Int8}, Nothing}=nothing)::Individual
	if selected_route_ids === nothing
		selected_route_nr = rand(1:length(individual.route_seg))
		selected_route_ids = sample(1:length(individual.route_seg), selected_route_nr, replace=false, ordered=true)
	end
	unserved_task_ids = Vector{Int16}()
	new_sequence = Vector{Int16}([0])
	
	for (i, route_seg) in enumerate(individual.route_seg)
		if i in selected_route_ids
			task_ids = individual.sequence[route_seg[1]+1:route_seg[2]-1]
			append!(unserved_task_ids, task_ids)
		else
			append!(new_sequence, individual.sequence[route_seg[1]+1:route_seg[2]])
		end
	end

	append!(new_sequence, randomized_path_scanning_heuristic(instance, Set{Int16}(unserved_task_ids)).sequence[2:end])
	
	new_individual = individual_from_sequence(instance, new_sequence)
	#print("$selected_route_nr\t$selected_route_ids\t")
	#print(individual.total_cost)
	#print("\t")
	#println(new_individual.total_cost)
	
	return new_individual
end

function abc(instance::Instance, colony_size::Int64=10, local_search_limit::Int64=10, no_improvement_limit::Int64=5)
	push!(output_dump, "\nABC")

	population = Vector{Individual}(undef, colony_size)
	population[1] = global_best_solution.sequence === nothing ? randomized_path_scanning_heuristic(instance, union(instance.task_ids, instance.virtual_task_ids)) : copy(global_best_solution)
	population[2:colony_size] = [randomized_path_scanning_heuristic(instance, union(instance.task_ids, instance.virtual_task_ids)) for i in 2:colony_size]
	global global_best_solution = Individual()
	update_global_best_solution(instance, get_individual_with_lowest_total_cost(population), "initial population")

	c = 0
	while peektimer() <= time_limit
		employed_bee_phase!(instance, population, local_search_limit)
		onlooker_bee_phase!(instance, population, no_improvement_limit)
		scout_bee_phase!(instance, population)
		c += 1
	end

	push!(output_dump, string(global_best_solution.sequence, "\n", global_best_solution.route_seg, "\n", global_best_solution.route_seg_load, "\n", global_best_solution.total_cost))
end

function employed_bee_phase!(instance::Instance, population::Vector{Individual}, local_search_limit::Int64)
	for i in eachindex(population)
		if population[i].age == -1
			best_individual_i = copy(population[i])
			while best_individual_i.age < local_search_limit
				
				new_individual = merge_split(instance, best_individual_i)

				if is_newer_individual_better(best_individual_i, new_individual)
					best_individual_i = copy(new_individual)
					update_global_best_solution(instance, new_individual, "$i\tmerge_split")
				else
					best_individual_i.age += 1
				end
			end
			if best_individual_i.sequence != population[i].sequence
				population[i] = copy(best_individual_i)
			end
		end
	end
end

function onlooker_bee_phase!(instance::Instance, population::Vector{Individual}, no_improvement_limit::Int64)
	#potential_individuals = rand(population, 3)
	individual = get_individual_with_lowest_total_cost(population)
	#individual = sample(population, Weights([1/ind.total_cost for ind in population]), 1)[1]
	#individual = rand(population)
	
	individual_index = findall(x -> x==individual, population)[1]

	operators = [two_opt, swap, insert, greedy_sub_tour_mutation]
	global_best_individual = copy(individual)
	current_individual = copy(individual)
	local_best_total_cost = global_best_individual.total_cost
	i = 0
	while i < no_improvement_limit && peektimer() <= time_limit
		shuffle!(operators)
		for operator in operators
			task_ids = get_task_ids_from_sequence(current_individual.sequence)
			shuffle!(task_ids)
			for task_id in task_ids
				task_id_candidate_list = i <= no_improvement_limit/2 ? construct_candidate_list(instance, task_id, task_ids) : copy(task_ids)
				shuffle!(task_id_candidate_list)
				task_id_candidate_list = task_id_candidate_list[1:length(task_id_candidate_list)]
				for task_id_2 in task_id_candidate_list
					if task_id != task_id_2
						new_individual = operator(instance, current_individual, task_id, task_id_2)
						
						# improving solution found
						if is_newer_individual_better(current_individual, new_individual)
							current_individual = copy(new_individual)
							# new global best solution found
							if is_newer_individual_better(global_best_individual, new_individual)
								global_best_individual = copy(new_individual)
								update_global_best_solution(instance, new_individual, "$individual_index\t$i\t$operator")
							end
							break
						end
					end
				end
			end
		end
		
		# Update local best total cost
		if current_individual.total_cost < local_best_total_cost
			local_best_total_cost = current_individual.total_cost
			i = 0
		else
			i += 1
		end
	end
	
	if is_newer_individual_better(individual, global_best_individual)
		population[individual_index] = copy(global_best_individual)
	else
		population[individual_index].age = 1
	end
end

function scout_bee_phase!(instance::Instance, population::Vector{Individual})
	for i in eachindex(population)
		if population[i].age == 1
			population[i] = random_routing_plan_generator(instance)
			update_global_best_solution(instance, population[i], "$i\trandom plan")
		end
	end
end

# will change later to take into account tc_route_max
function is_newer_individual_better(old_individual::Individual, new_individual::Individual)::Bool
	if old_individual.total_cost > new_individual.total_cost
		return true
	else
		return false
	end
end

function print_population(population::Vector{Individual})
	for ind in population
		print(string(ind.total_cost) * " (" * string(ind.age) * ")\t")
	end
	println()
end

function hma(instance::Instance)
	push!(output_dump, "\nHMA")
	population = Vector{Individual}(undef, psize)
	population[1] = global_best_solution.sequence === nothing ? randomized_path_scanning_heuristic(instance, union(instance.task_ids, instance.virtual_task_ids)) : copy(global_best_solution)
	population[2:psize] = [randomized_path_scanning_heuristic(instance, union(instance.task_ids, instance.virtual_task_ids)) for i in 2:psize]
	update_global_best_solution(instance, get_individual_with_lowest_total_cost(population))
	cnt = ones(Int, length(treshold_ratio))
	x = 0
	od = true	# to execute RTTP first in the first iteration
	while peektimer() <= time_limit
		individual_1, individual_2 = x == 0 ? [population[1], sample(population[2:psize], 1)[1]] : sample(population, 2, replace=false)
		individual = route_based_crossover(instance, individual_1, individual_2)
		k = sample(1:length(cnt), Weights(cnt ./ sum(cnt)), 1)[1]
		individual = local_refinement(instance, individual, treshold_ratio[k], od)
		push!(population, individual)
		x += 1
		od = rand(Bool)
		if calculate_quality_and_distance_fitness!(instance, population) # if pool updating is successful
			cnt[k] += 1
		end
	end
	push!(output_dump, string(global_best_solution.sequence, "\n", global_best_solution.route_seg, "\n", global_best_solution.route_seg_load, "\n", global_best_solution.total_cost))
end

function route_based_crossover(instance::Instance, individual_1::Individual, individual_2::Individual)::Individual
	new_individual_sequence = Vector{Int16}()
	a = rand(1:length(individual_1.route_seg))
	b = rand(1:length(individual_2.route_seg))

	new_individual_sequence = reduce(append!, (individual_1.sequence[1:individual_1.route_seg[a][1]], individual_2.sequence[individual_2.route_seg[b][1]+1:individual_2.route_seg[b][2]-1], individual_1.sequence[individual_1.route_seg[a][2]:end]), init=Int16[])
	
	task_id_list_1 = individual_1.sequence[individual_1.route_seg[a][1]+1:individual_1.route_seg[a][2]-1]
	task_id_list_2 = individual_2.sequence[individual_2.route_seg[b][1]+1:individual_2.route_seg[b][2]-1]
	
	unserved_tasks = setdiff(Set{Int16}(task_id_list_1), Set{Int16}(task_id_list_2))
	duplicate_tasks = setdiff(Set{Int16}(task_id_list_2), Set{Int16}(task_id_list_1))
	
	# keep one from each duplicate task in the positions which results with a better total cost
	while length(duplicate_tasks) > 0
		task_id = pop!(duplicate_tasks)
		task_pos = findall(x -> x == task_id, new_individual_sequence)
		position_costs = Vector{Int32}([0, 0])
		
		for (i, j) in [(1, 2), (2, 1)]
			position_costs[i] = min(instance.min_cost[get_arc(instance, new_individual_sequence[task_pos[i]-1]).tail_node, get_arc(instance, new_individual_sequence[task_pos[i]]).head_node] + instance.min_cost[get_arc(instance, new_individual_sequence[task_pos[i]]).tail_node, get_arc(instance, new_individual_sequence[task_pos[i]+1]).head_node] + instance.min_cost[get_arc(instance, new_individual_sequence[task_pos[j]-1]).tail_node, get_arc(instance, new_individual_sequence[task_pos[j]+1]).head_node], inf)
		end
		
		if position_costs[1] < position_costs[2]
			deleteat!(new_individual_sequence, task_pos[2])
		else
			deleteat!(new_individual_sequence, task_pos[1])
		end
	end
	
	# if a route plan became empty -> remove the excess 0s
	if issubset([0, 0], new_individual_sequence)
		i = 2
		while i <= length(new_individual_sequence)
			if new_individual_sequence[i-1] == 0 && new_individual_sequence[i] == 0
				deleteat!(new_individual_sequence, i)
			else
				i += 1
			end
		end
		if new_individual_sequence[end] != 0
			push!(new_individual_sequence, 0)
		end
	end
	
	# add the unserved tasks in random order
	unserved_tasks = collect(unserved_tasks)
	shuffle!(unserved_tasks)
	
	new_individual = individual_from_sequence(instance, new_individual_sequence)
	
	while length(unserved_tasks) > 0
		task_id = pop!(unserved_tasks)
		
		potential_route_ids = Vector{Int16}()
		for (r_id, r_load) in enumerate(new_individual.route_seg_load)
			if r_load + instance.tasks[task_id].demand <= instance.capacity
				push!(potential_route_ids, r_id)
			end
		end
		
		if length(potential_route_ids) == 0
			append!(new_individual.sequence, [task_id, 0])
			push!(new_individual.route_seg, (new_individual.route_seg[end][2], new_individual.route_seg[end][2]+2))
			push!(new_individual.route_seg_load, instance.tasks[task_id].demand)
			add_cost = instance.min_cost[instance.dummy_arc.tail_node, get_arc(instance, task_id).head_node] + instance.tasks[task_id].serv_cost + instance.min_cost[get_arc(instance, task_id).tail_node, instance.dummy_arc.head_node]
			new_individual.total_cost = new_individual.total_cost < inf && 0 <= add_cost < inf ? new_individual.total_cost + add_cost : inf
		else
			r_task_best_i_cost_diff = inf+1 								# the cost difference of the best position
			r_task_best_r = potential_route_ids[1]							# the route plan id of the best position
			r_task_best_i = new_individual.route_seg[r_task_best_r][1]+1	# the best position
			
			for r_id in potential_route_ids, i in new_individual.route_seg[r_id][1]+1:new_individual.route_seg[r_id][2]
				
				cost_diff = instance.min_cost[get_arc(instance, new_individual.sequence[i-1]).tail_node, get_arc(instance, task_id).head_node] + instance.min_cost[get_arc(instance, task_id).tail_node, get_arc(instance, new_individual.sequence[i]).head_node] - instance.min_cost[get_arc(instance, new_individual.sequence[i-1]).tail_node, get_arc(instance, new_individual.sequence[i]).head_node]
				
				if r_task_best_i_cost_diff > cost_diff >= 0
					r_task_best_r = r_id
					r_task_best_i = i
					r_task_best_i_cost_diff = cost_diff
				end
				
			end
			
			insert!(new_individual.sequence, r_task_best_i, task_id)
			new_individual.route_seg[r_task_best_r] = (new_individual.route_seg[r_task_best_r][1], new_individual.route_seg[r_task_best_r][2]+1)
			if r_task_best_r < length(new_individual.route_seg)
				for r_id in r_task_best_r+1:length(new_individual.route_seg)
					new_individual.route_seg[r_id] = (new_individual.route_seg[r_id][1]+1, new_individual.route_seg[r_id][2]+1)
				end
			end
			new_individual.route_seg_load[r_task_best_r] += instance.tasks[task_id].demand
			add_cost = new_individual.total_cost < inf && 0 <= r_task_best_i_cost_diff < inf ? r_task_best_i_cost_diff + instance.tasks[task_id].serv_cost : 0
			new_individual.total_cost = new_individual.total_cost < inf && 0 <= add_cost < inf ? new_individual.total_cost + add_cost : inf
		end
	end
	
	update_global_best_solution(instance, new_individual, "route_based_crossover")
	
	return new_individual
end

function local_refinement(instance::Instance, individual::Individual, r::Float16, od::Bool)::Individual
	new_individual = copy(individual)
	# RTTP first
	if od
		#println("RTTP")
		new_individual = rttp(instance, new_individual, r)
		#println("IDP")
		new_individual = idp(instance, new_individual)
	# IDP first
	else
		#println("IDP")
		new_individual = idp(instance, new_individual)
		#println("RTTP")
		new_individual = rttp(instance, new_individual, r)
	end
	return new_individual
end

function manage_penalty_parameter!(penalty_parameter::Float16, feasible_count::Int32, infeasible_count::Int32, excess_demand::Int32)::Tuple{Float16, Int32, Int32}
	# infeasible solution
	if excess_demand != 0
		infeasible_count += 1
		feasible_count = 0
	# feasible solution
	else
		feasible_count += 1
		infeasible_count = 0
	end
	# halve the penalty parameter
	if feasible_count == 5
		penalty_parameter = penalty_parameter / 2
		feasible_count = 0
	# double the penalty parameter
	elseif infeasible_count == 5
		penalty_parameter = penalty_parameter * 2
		infeasible_count = 0
	end

	return penalty_parameter, feasible_count, infeasible_count
end

# Randomized Tabu Tresholding Procedure (RTTP)
function rttp(instance::Instance, individual::Individual, r::Float16)::Individual
	operators = [insert, double_insert, swap, two_opt]
	global_best_individual = copy(individual)
	current_individual = copy(individual)
	local_best_total_cost = global_best_individual.total_cost
	w = 0
	while w < W && peektimer() <= time_limit
		# Mixed phase
		#println("Mixed phase")
		T = rand(28:33)
		for k in 1:T
			shuffle!(operators)
			for operator in operators
				task_ids = get_task_ids_from_sequence(current_individual.sequence)
				shuffle!(task_ids)
				for task_id in task_ids
					feasible_move_individuals = Vector{Individual}()
					improving_solution_found = false
					task_id_candidate_list = construct_candidate_list(instance, task_id, task_ids)
					shuffle!(task_id_candidate_list)
					for task_id_2 in task_id_candidate_list
						new_individual = operator(instance, current_individual, task_id, task_id_2)
						
						# improving solution found
						if is_newer_individual_better(current_individual, new_individual)
							improving_solution_found = true
							current_individual = copy(new_individual)
							# new global best solution found
							if is_newer_individual_better(global_best_individual, new_individual)
								global_best_individual = copy(new_individual)
								update_global_best_solution(instance, new_individual, operator)
							end
							break
						end
						
						# feasible move
						if new_individual.total_cost <= (1 + r) * local_best_total_cost
							push!(feasible_move_individuals, new_individual)
						end
					end
					if !improving_solution_found && length(feasible_move_individuals) > 0
						current_individual = get_individual_with_lowest_total_cost(feasible_move_individuals)
					end
				end
			end
		end

		# Improving phase
		#println("Improving phase")
		improvement_phase = true
		while improvement_phase
			improvement_phase = false
			shuffle!(operators)
			for operator in operators
				task_ids = get_task_ids_from_sequence(current_individual.sequence)
				shuffle!(task_ids)
				for task_id in task_ids
					task_id_candidate_list = construct_candidate_list(instance, task_id, task_ids)
					shuffle!(task_id_candidate_list)
					for task_id_2 in task_id_candidate_list
						new_individual = operator(instance, current_individual, task_id, task_id_2)
						
						# improving solution found
						if is_newer_individual_better(current_individual, new_individual)
							improvement_phase = true
							current_individual = copy(new_individual)
							# new global best solution found
							if is_newer_individual_better(global_best_individual, new_individual)
								global_best_individual = copy(new_individual)
								update_global_best_solution(instance, new_individual, operator)
							end
							break
						end
					end
				end
			end
		end
		
		# Update local best total cost
		if current_individual.total_cost < local_best_total_cost
			local_best_total_cost = current_individual.total_cost
			w = 0
		else
			w += 1
		end
	end
	
	return global_best_individual
end

# Infeasible Descent Procedure (IDP) - examine all the moves in the neighborhood and execute only one (the best) move at thw end of the search
function idp(instance::Instance, individual::Individual)::Individual
	best_individual = copy(individual)
	best_individual_cost = best_individual.total_cost
	global_best_individual = copy(individual)
	
	# First stage
	penalty_parameter::Float16 = best_individual.total_cost / (2 * instance.capacity)
	feasible_count::Int32 = 0
	infeasible_count::Int32 = 0
	improvement_found = false
	
	task_ids = get_task_ids_from_sequence(individual.sequence)
    operators = [insert, double_insert, swap]
	for operator in operators, task_id_1 in task_ids, task_id_2 in task_ids
		if task_id_1 != task_id_2
			new_individual = operator(instance, individual, task_id_1, task_id_2, false)
			excess_demand = calculate_excess_demand(instance, new_individual.route_seg_load)
			new_individual_cost = new_individual.total_cost + excess_demand * penalty_parameter
			
			if new_individual_cost < best_individual_cost
				improvement_found = true
				best_individual = copy(new_individual)
				best_individual_cost = new_individual_cost
				penalty_parameter, feasible_count, infeasible_count = manage_penalty_parameter!(penalty_parameter, feasible_count, infeasible_count, excess_demand)
				
				if excess_demand == 0 && is_newer_individual_better(global_best_individual, new_individual)
					global_best_individual = copy(new_individual)
					update_global_best_solution(instance, new_individual, operator)
				end
			end
		end
	end
	
	# Second stage
	if !improvement_found && peektimer() <= time_limit
		penalty_parameter = individual.total_cost / (2 * instance.capacity)
		feasible_count = 0
		infeasible_count = 0
		
		r_nr = length(individual.route_seg)
		r_ids::Vector{Int8} = collect(1:r_nr)
		possible_combinations = collect(combinations(r_ids))
		shuffle!(possible_combinations)
		for i in 1:min(length(possible_combinations), 100)
			new_individual = merge_split(instance, individual, pop!(possible_combinations))
			excess_demand = calculate_excess_demand(instance, new_individual.route_seg_load)
			new_individual_cost = new_individual.total_cost + excess_demand * penalty_parameter
			
			if new_individual_cost < best_individual_cost
				improvement_found = true
				best_individual = copy(new_individual)
				best_individual_cost = new_individual_cost
				penalty_parameter, feasible_count, infeasible_count = manage_penalty_parameter!(penalty_parameter, feasible_count, infeasible_count, excess_demand)
				
				if excess_demand == 0 && is_newer_individual_better(global_best_individual, new_individual)
					global_best_individual = copy(new_individual)
					update_global_best_solution(instance, new_individual, "merge_split")
				end
			end
		end
		
		# First stage again, if improvement was found
		if improvement_found && peektimer() <= time_limit
			current_individual = copy(best_individual)
			penalty_parameter = current_individual.total_cost / (2 * instance.capacity)
			feasible_count = 0
			infeasible_count = 0
			
			task_ids = get_task_ids_from_sequence(current_individual.sequence)
			operators = [insert, double_insert, swap]
			for operator in operators, task_id_1 in task_ids, task_id_2 in task_ids
				if task_id_1 != task_id_2
					new_individual = operator(instance, current_individual, task_id_1, task_id_2, false)
					excess_demand = calculate_excess_demand(instance, new_individual.route_seg_load)
					new_individual_cost = new_individual.total_cost + excess_demand * penalty_parameter
					
					if new_individual_cost < best_individual_cost
						best_individual = copy(new_individual)
						best_individual_cost = new_individual_cost
						penalty_parameter, feasible_count, infeasible_count = manage_penalty_parameter!(penalty_parameter, feasible_count, infeasible_count, excess_demand)
						
						if excess_demand == 0 && is_newer_individual_better(global_best_individual, new_individual)
							global_best_individual = copy(new_individual)
							update_global_best_solution(instance, new_individual, operator)
						end
					end
				end
			end
		end
	
	end
	
	return global_best_individual
end

# Infeasible Descent Procedure (IDP) - execute all the imporiving moves during the search
function idp2(instance::Instance, individual::Individual)::Individual
	best_individual = copy(individual)
	best_individual_cost = best_individual.total_cost
	global_best_individual = copy(individual)
	
	# First stage
	penalty_parameter::Float16 = best_individual.total_cost / (2 * instance.capacity)
	feasible_count::Int32 = 0
	infeasible_count::Int32 = 0
	
	task_ids = get_task_ids_from_sequence(best_individual.sequence)
    operators = [insert, double_insert, swap]
	for operator in operators, task_id_1 in task_ids, task_id_2 in task_ids
		if task_id_1 != task_id_2
			new_individual = operator(instance, best_individual, task_id_1, task_id_2, false)
			excess_demand = calculate_excess_demand(instance, new_individual.route_seg_load)
			new_individual_cost = new_individual.total_cost + excess_demand * penalty_parameter
			
			if new_individual_cost < best_individual_cost
				best_individual = copy(new_individual)
				best_individual_cost = new_individual_cost
				penalty_parameter, feasible_count, infeasible_count = manage_penalty_parameter!(penalty_parameter, feasible_count, infeasible_count, excess_demand)
				
				if excess_demand == 0 && is_newer_individual_better(global_best_individual, new_individual)
					global_best_individual = copy(new_individual)
					update_global_best_solution(instance, new_individual, operator)
				end
			end
		end
	end
	
	# Second stage
	improvement_found = false
	if best_individual.total_cost == individual.total_cost
		penalty_parameter = best_individual.total_cost / (2 * instance.capacity)
		feasible_count = 0
		infeasible_count = 0
		
		r_nr = length(best_individual.route_seg)
		r_ids::Vector{Int8} = collect(1:r_nr)
		possible_combinations = collect(combinations(r_ids))
		shuffle!(possible_combinations)
		for i in 1:min(length(possible_combinations), 100)
			new_individual = merge_split(instance, best_individual, pop!(possible_combinations))
			excess_demand = calculate_excess_demand(instance, new_individual.route_seg_load)
			new_individual_cost = new_individual.total_cost + excess_demand * penalty_parameter
			
			if new_individual_cost < best_individual_cost
				improvement_found = true
				best_individual = copy(new_individual)
				best_individual_cost = new_individual_cost
				penalty_parameter, feasible_count, infeasible_count = manage_penalty_parameter!(penalty_parameter, feasible_count, infeasible_count, excess_demand)
				
				if excess_demand == 0 && is_newer_individual_better(global_best_individual, new_individual)
					global_best_individual = copy(new_individual)
					update_global_best_solution(instance, new_individual, "merge_split")
				end
			end
		end
	end
	
	# First stage again, if improvement was found
	if improvement_found
		penalty_parameter = best_individual.total_cost / (2 * instance.capacity)
		feasible_count = 0
		infeasible_count = 0
		
		task_ids = get_task_ids_from_sequence(best_individual.sequence)
		operators = [insert, double_insert, swap]
		for operator in operators, task_id_1 in task_ids, task_id_2 in task_ids
			if task_id_1 != task_id_2
				new_individual = operator(instance, best_individual, task_id_1, task_id_2, false)
				excess_demand = calculate_excess_demand(instance, new_individual.route_seg_load)
				new_individual_cost = new_individual.total_cost + excess_demand * penalty_parameter
				
				if new_individual_cost < best_individual_cost
					best_individual = copy(new_individual)
					best_individual_cost = new_individual_cost
					penalty_parameter, feasible_count, infeasible_count = manage_penalty_parameter!(penalty_parameter, feasible_count, infeasible_count, excess_demand)
					
					if excess_demand == 0 && is_newer_individual_better(global_best_individual, new_individual)
						global_best_individual = copy(new_individual)
						update_global_best_solution(instance, new_individual, operator)
					end
				end
			end
		end
	end
	
	return global_best_individual
end

function calculate_distance_between_two_tasks(instance::Instance, task_id_1::Int16, task_id_2::Int16)::Float16
	return instance.min_cost[get_arc(instance, task_id_1).tail_node, get_arc(instance, task_id_2).head_node]
end

function construct_candidate_list(instance::Instance, task_id::Int16, task_ids::Vector{Int16}, csize::Int16=Int16(12))::Vector{Int16}
	deleteat!(task_ids, findall(x -> x==task_id, task_ids)[1])
	dist_values::Vector{Float16} = [calculate_distance_between_two_tasks(instance, task_id, other_task_id) for other_task_id in task_ids]
	return task_ids[sortperm(dist_values)][1:min(csize, length(task_ids))]
end

function hamming_distance(instance::Instance, individual_1::Individual, individual_2::Individual)::Int16
	n = length(instance.task_ids)
    m = min(length(individual_1.route_seg), length(individual_2.route_seg))
	
    individual_dh_links = [[(get_arc(instance, individual.sequence[i]).head_node, get_arc(instance, individual.sequence[i+1]).tail_node) for i in 1:length(individual.sequence)-1] for individual in [individual_1, individual_2]]

	same_dh_link_nr = length(findall(x->x in individual_dh_links[1], individual_dh_links[2]))
	
	return n + m - same_dh_link_nr
end

function get_ranking(list_length::Int8, ordering::Vector{Int64})
	ranking = zeros(Int8, list_length)
	rank = 1
	for i in ordering
		ranking[i] = rank
		rank += 1
	end
	return ranking
end

function calculate_quality_and_distance_fitness!(instance::Instance, population::Vector{Individual}):Bool
	population_size::Int8 = length(population)
	
	OR = get_ranking(population_size, sortperm([individual.total_cost for individual in population]))

	AD_pop = zeros(Float32, population_size)
	for i in 1:population_size, j in population_size
		if i != j
			AD_pop[i] += hamming_distance(instance, population[i], population[j])
		end
	end
	AD_pop = [value / (population_size - 1) for value in AD_pop]
	DR = get_ranking(population_size, sortperm(AD_pop, rev=true))

	population_qdf = [alpha * OR[i] + (1-alpha) * DR[i] for i in eachindex(population)]
	
	max_qdf_value_index = argmax(population_qdf)
	deleteat!(population, max_qdf_value_index)
	
	return max_qdf_value_index != length(population)+1
end

function reroute_one_route(instance, initial_solution::Individual, initial_solution_wo_new_tasks::Individual, new_tasks::Union{Vector{Int16}, Nothing})
	push!(output_dump, "\nRR1")
	tasks_demand = 0
	for task_id in new_tasks
		tasks_demand += instance.tasks[task_id].demand
	end

	potential_route_ids = Vector{Int8}()
	for r_id in 1:length(initial_solution_wo_new_tasks.route_seg)
		if tasks_demand + initial_solution_wo_new_tasks.route_seg_load[r_id] <= instance.capacity
			push!(potential_route_ids, r_id)
		end
	end

	best_individual = copy(initial_solution)

	if length(potential_route_ids) > 0
		for r_id in potential_route_ids
			individual = copy(initial_solution_wo_new_tasks)
			for task_id in new_tasks
				r_task_best_i_cost_diff = inf
				r_task_best_i = -1
				for i in individual.route_seg[r_id][1]+1:individual.route_seg[r_id][2]
					cost_diff = min(instance.min_cost[get_arc(instance, individual.sequence[i-1]).tail_node, get_arc(instance, task_id).head_node] + instance.min_cost[get_arc(instance, task_id).tail_node, get_arc(instance, individual.sequence[i]).head_node] - instance.min_cost[get_arc(instance, individual.sequence[i-1]).tail_node, get_arc(instance, individual.sequence[i]).head_node], inf)
					if cost_diff < r_task_best_i_cost_diff
						r_task_best_i_cost_diff = cost_diff
						r_task_best_i = i
					end
				end
				insert!(individual.sequence, r_task_best_i, task_id)
				individual.route_seg[r_id] = (individual.route_seg[r_id][1], individual.route_seg[r_id][2]+1)
				if r_id < length(individual.route_seg)
					for r_id_2 in r_id+1:length(individual.route_seg)
						individual.route_seg[r_id_2] = (individual.route_seg[r_id_2][1]+1, individual.route_seg[r_id_2][2]+1)
					end
				end
				individual.route_seg_load[r_id] += instance.tasks[task_id].demand
				add_cost = r_task_best_i_cost_diff + instance.tasks[task_id].serv_cost
				individual.total_cost = min(individual.total_cost + add_cost, inf)
			end
			if is_newer_individual_better(best_individual, individual)
				best_individual = copy(individual)
			end
		end
	end

	global global_best_solution = best_individual
	push!(output_dump, string(global_best_solution.sequence, "\n", global_best_solution.route_seg, "\n", global_best_solution.route_seg_load, "\n", global_best_solution.total_cost))
end

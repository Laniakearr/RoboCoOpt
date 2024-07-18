from linkage_physics import *
import pygad
import numpy
import copy,pickle

# Global Parameters
MAX_NODE = 6

# linkage Parameters:
#state definition:
NOT_USED=-1
MOVABLE=0
FIXED=1

#Date format:
#[-1,rad,ctrX,ctrY,state[0]]+ (this is for motor)
#[C1[i],C2[i],len1[i],len2[i],state[i] for i in range(K)] (this is for nodes)

K=MAX_NODE
B=10
maxTrial=100
maxDist = 2.0 * math.sqrt(2.0) * B

#from LinkageAnnealer::nrN(:
def get_node_number(solution):
    return len(solution) // 5

# From LinkageAnnealer
def generate_init_state(ret=None):
    if ret is None:
        ret = []
    for i in range(len(ret)//5, K):
        # set state
        if i == 0:
            state = MOVABLE
        elif i == 1:
            state = random.choice([NOT_USED, FIXED])
        else:
            state = random.choice([NOT_USED, MOVABLE, FIXED])
        # initialize according to state
        if state == NOT_USED:
            break
        elif state == FIXED:
            ret += [-1, -1, random.uniform(-B, B), random.uniform(-B, B), state]
        elif i == 0:
            # ret+=[-1,random.uniform(B/10,B),random.uniform(-B,B),random.uniform(-B,B),state]
            ret += [-1, random.uniform(B / 10, B), 0.0, 0.0, state]
        else:
            s = [-1, -1, -1, -1, -1]
            while s[0] == s[1]:
                s = [random.randint(0, i - 1), random.randint(0, i - 1), random.uniform(0, maxDist),
                     random.uniform(0, maxDist), state]
            ret += s
    # remove
    while ret[-1] == FIXED:
        ret = ret[0:len(ret) - 5]
    return ret

# generate a single initial solution
def generate_solution():
    solution = []
    while True:
        solution = generate_init_state()
        if check_feasibility(solution, min_node_num=3):
            break

    # extend the solution to required length
    cur_node_num = get_node_number(solution)
    if cur_node_num < MAX_NODE:
        #print(len(solution), solution)
        for i in range(MAX_NODE - cur_node_num):
            solution += [-1, -1, -1, -1, -1]
        # print("after extension", len(solution), solution)
    return solution

# generate initial population
def generate_population(sol_per_pop):
    print("start generate initial population")
    initial_population = []
    for i in range(sol_per_pop):
        solution = generate_solution()
        initial_population.append(solution)
        print(i, solution)
    print("initial population generated")
    print(initial_population)
    return initial_population

# generate limitation on each value in solution
# TODO: check if it is correct
def generate_gen_space():
    # motor value
    gene_space = [[-1], random.uniform(B/10,B), [0.0], [0.0], [0]]
    # first node value
    gene_space += [[-1], [-1], random.uniform(-B, B), random.uniform(-B, B), [1]]
    # other nodes value
    for i in range(2, K):
        gene_space+=[random.randint(-1,i-1),random.randint(-1,i-1),random.uniform(0,maxDist),random.uniform(0,maxDist),[-1, 0, 1]]
    return gene_space

# TODO: change the function to stop infinite loop
def check_topology_feasibility(solution):
    # check if every node is connected to the last node
    node_num = get_node_number(solution)
    visited = [False for i in range(node_num)]
    visited[node_num - 1] = True
    stack = [node_num - 1]
    iter = 0
    while len(stack) > 0 and iter < 10:
        #print("I am here")
        id = stack[-1]
        stack.pop()
        if int(solution[id * 5 + 4]) == FIXED or id == 0:
            ss = []
        else:
            ss = [int(solution[id * 5 + 0]), int(solution[id * 5 + 1])]
        for i in ss:
            stack.append(i)
            visited[i] = True
        ++iter
    for i in range(node_num):
        if not visited[i]:
            return False
    # check if every movable node is connected to motor-node
    visited = [False for i in range(node_num)]
    visited[0] = True
    stack = [0]
    while len(stack) > 0:
        id = stack[-1]
        stack.pop()
        for j in range(id + 1, node_num):
            if int(solution[j * 5 + 0]) == id or int(solution[j * 5 + 1]) == id:
                stack.append(j)
                visited[j] = True
    for i in range(node_num):
        if int(solution[i * 5 + 4]) == MOVABLE and not visited[i]:
            return False
    return True

# TODO: check if if is working
def check_geometry_feasibility(solution, nrSample=32):
    link = set_to_linkage(solution)
    for i in range(nrSample):
        theta, succ = link.forward_kinematic(math.pi * 2 * i / nrSample)
        if not succ:
            return False
        for pos in link.node_position:
            for d in range(2):
                if pos[d] < -B or pos[d] > B:
                    return False
    return True

# check the feasibility of the input solution
def check_feasibility(solution, min_node_num=0):
    return check_topology_feasibility(solution) \
           and check_geometry_feasibility(solution) \
           and get_node_number(solution) > min_node_num

# TODO: check if if is working
def set_to_linkage(solution):
    link=Linkage(get_node_number(solution))
    link.rad_motor=solution[1]
    link.ctr_motor=(solution[2],solution[3])
    for i in range(1,get_node_number(solution)):
        link.U[i+1]=1
        state=solution[i*5+4]
        if state==MOVABLE:
            link.F[i+1]=0
            link.C1[i+1]=solution[i*5+0]+1
            link.C2[i+1]=solution[i*5+1]+1
            link.len1[i+1]=solution[i*5+2]
            link.len2[i+1]=solution[i*5+3]
        elif state==FIXED:
            link.F[i+1]=1
            link.node_position[i+1]=(solution[i*5+2],solution[i*5+3])
        else: assert False
    return link

def fitness_func(ga_instance, solution, solution_idx):
    print(ga_instance, solution, solution_idx)
    if not check_feasibility(solution):
        #TODO: Can we modify/reset the solution to make it feasible? Or just dump it?
        return -1
    link = set_to_linkage(solution)
    robot = create_robot(link, sep=5.)
    if robot is None:
        return 0.
    else:
        return robot.eval_performance(10.)

if __name__ == '__main__':
    # desired distance
    desired_output = 25
    fitness_function = fitness_func

    num_generations = 50
    num_parents_mating = 4

    sol_per_pop = 5
    #num_genes=30
    initial_population=generate_population(sol_per_pop)
    gene_space = generate_gen_space()

    #init_range_low = -2
    #init_range_high = 5

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 50

    ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating,
                           fitness_func=fitness_function,
                           #sol_per_pop=sol_per_pop,
                           #num_genes=num_genes,
                           initial_population=initial_population,
                           #init_range_low=init_range_low,
                           #init_range_high=init_range_high,
                           parent_selection_type=parent_selection_type,
                           keep_parents=keep_parents,
                           crossover_type=crossover_type,
                           mutation_type=mutation_type,
                           mutation_percent_genes=mutation_percent_genes,
                           gene_space=gene_space)

    ga_instance.run()
    solution, solution_fitness, solution_idx = ga_instance.best_solution()
    print("Parameters of the best solution : {solution}".format(solution=solution))
    print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))

    prediction = numpy.sum(numpy.array(function_inputs) * solution)
    print("Predicted output based on the best solution : {prediction}".format(prediction=prediction))
# '''

'''
# Test function
def fitness_func(ga_instance, solution, solution_idx):
    output = numpy.sum(solution*function_inputs)
    fitness = 1.0 / numpy.abs(output - desired_output)
    return fitness

if __name__ == '__main__':
    function_inputs = [4, -2, 3.5, 5, -11, -4.7]
    desired_output = 44

    fitness_function = fitness_func

    num_generations = 50
    num_parents_mating = 4

    sol_per_pop = 8
    num_genes = len(function_inputs)

    init_range_low = -2
    init_range_high = 5

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 10

    ga_instance = pygad.GA(num_generations=num_generations,
                           num_parents_mating=num_parents_mating,
                           fitness_func=fitness_function,
                           sol_per_pop=sol_per_pop,
                           num_genes=num_genes,
                           init_range_low=init_range_low,
                           init_range_high=init_range_high,
                           parent_selection_type=parent_selection_type,
                           keep_parents=keep_parents,
                           crossover_type=crossover_type,
                           mutation_type=mutation_type,
                           mutation_percent_genes=mutation_percent_genes)
    ga_instance.run()
    solution, solution_fitness, solution_idx = ga_instance.best_solution()
    print("Parameters of the best solution : {solution}".format(solution=solution))
    print("Fitness value of the best solution = {solution_fitness}".format(solution_fitness=solution_fitness))

    prediction = numpy.sum(numpy.array(function_inputs) * solution)
    print("Predicted output based on the best solution : {prediction}".format(prediction=prediction))
'''

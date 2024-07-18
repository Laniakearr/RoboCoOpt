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

# Note that data means the old state data, and solution is for GA
def data_to_solution(data):
    #print("data", data)
    solution = data
    # extend the solution to required length
    cur_node_num = get_node_num(data)
    if cur_node_num < MAX_NODE:
        # print(len(solution), solution)
        for i in range(MAX_NODE - cur_node_num):
            solution += [-1, -1, -1, -1, -1]
        # print("after extension", len(solution), solution)
    #print("solution", solution)
    return solution

def solution_to_data(solution):
    #print("solution", solution)
    node_num = len(solution) // 5
    while (int(solution[node_num * 5 - 1]) is NOT_USED) and node_num > 0:
        node_num = node_num - 1
    data = solution[:node_num*5]
    data[0] = -1
    data[4] = int(data[4])
    for i in range(1, node_num):
        data[5 * i + 0] = int(data[5 * i + 0])
        data[5 * i + 1] = int(data[5 * i + 1])
        data[5 * i + 4] = int(data[5 * i + 4])
    #print("data", data)
    return data

#from LinkageAnnealer::nrN(:
def get_node_num(data):
    return len(data) // 5

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
    while True:
        data = generate_init_state()
        if not check_data_validity(data): print("something is wrong with check_data_validity function!")
        if check_feasibility(data, min_node_num=3):
            break
    return data_to_solution(data)

# generate initial population
def generate_population(sol_per_pop):
    print("start generate initial population")
    initial_population = []
    for i in range(sol_per_pop):
        solution = generate_solution()
        initial_population.append(solution)
        print(i, solution)
        # check initial performance
        link = set_to_linkage(solution_to_data(solution))
        robot = create_robot(link, sep=5.)
        if robot is not None:
            print("performance", robot.eval_performance(10.))
    print("initial population generated")
    #print(initial_population)
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

# TODO: I add this to check some other cases to prevent inf. loop, please double check
#  the checks are based on generate_init_state --Tina
def check_data_validity(data, debug=False):
    # debug = True # comment this out to reduce printing info
    for i in range(get_node_num(data)):
        state = data[i * 5 + 4]
        first_connect = data[i * 5]
        second_connect = data[i * 5 + 1]
        if i==0:
            if state is not MOVABLE:
                if debug: print("motor not movable")
                return False
            if first_connect != -1:
                if debug: print("motor first connect wrong")
                return False
        elif i==1:
            if state is not FIXED:
                if debug: print("first node not fixed")
                return False
            if first_connect != -1:
                if debug: print("first node first connect wrong")
                return False
            if second_connect != -1:
                if debug: print("first node second connect wrong")
                return False
        else:
            if state is FIXED:
                if first_connect != -1:
                    if debug: print("FIXED node first connect wrong")
                    return False
                if second_connect != -1:
                    if debug: print("FIXED node second connect wrong")
                    return False
            elif state is MOVABLE:
                if first_connect == second_connect:
                    if debug: print("connectes to same node")
                    return False
                if first_connect == -1:
                    if debug: print("movable node first connect wrong")
                    return False
                if second_connect == -1:
                    if debug: print("movable node second connect wrong")
                    return False
            else:
                if debug: print("there is a not used point in structure")
                return False
    return True

#from LinkageAnnealer
def check_topology_feasibility(data):
    # check if every node is connected to the last node
    node_num = get_node_num(data)
    visited = [False for i in range(node_num)]
    visited[node_num - 1] = True
    stack = [node_num - 1]
    iter = 0
    while len(stack) > 0 and iter < 100:
        id = stack[-1]
        stack.pop()
        if data[id * 5 + 4] == FIXED or id == 0:
            ss = []
        else:
            ss = [data[id * 5 + 0], data[id * 5 + 1]]
        for i in ss:
            stack.append(i)
            visited[i] = True
        iter += 1
        if (iter == 100): print("warning, inf.loop:", data)
    for i in range(node_num):
        if not visited[i]:
            return False
    # check if every movable node is connected to motor-node
    visited = [False for i in range(node_num)]
    visited[0] = True
    stack = [0]
    iter = 0
    while len(stack) > 0 and iter < 100:
        id = stack[-1]
        stack.pop()
        for j in range(id + 1, node_num):
            if data[j * 5 + 0] == id or data[j * 5 + 1] == id:
                stack.append(j)
                visited[j] = True
        iter += 1
        if (iter == 100): print("warning, inf.loop:", data)
    for i in range(node_num):
        if data[i * 5 + 4] == MOVABLE and not visited[i]:
            return False
    return True

#from LinkageAnnealer
def check_geometry_feasibility(data, nrSample=32):
    link = set_to_linkage(data)
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
def check_feasibility(data, min_node_num=0):
    return check_data_validity(data) \
           and check_topology_feasibility(data) \
           and check_geometry_feasibility(data) \
           and get_node_num(data) > min_node_num

#from LinkageAnnealer
def set_to_linkage(data):
    link=Linkage(get_node_num(data))
    link.rad_motor=data[1]
    link.ctr_motor=(data[2],data[3])
    for i in range(1,get_node_num(data)):
        link.U[i+1]=1
        state=data[i*5+4]
        if state==MOVABLE:
            link.F[i+1]=0
            link.C1[i+1]=data[i*5+0]+1
            link.C2[i+1]=data[i*5+1]+1
            link.len1[i+1]=data[i*5+2]
            link.len2[i+1]=data[i*5+3]
        elif state==FIXED:
            link.F[i+1]=1
            link.node_position[i+1]=(data[i*5+2],data[i*5+3])
        else: assert False
    return link

def fitness_func(ga_instance, solution, solution_idx):
    #print("solution index:", solution_idx)
    #print(solution)
    data = solution_to_data(solution.tolist())
    if not check_feasibility(data):
        #TODO: Can we modify/reset the data to make it feasible? Or just dump it?
        return -1
    link = set_to_linkage(data)
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

    sol_per_pop = 10
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

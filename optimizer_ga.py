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

# note the only functions modified are __init__, check_feasibility, check_data_validity
class LinkageGA():
    NOT_USED=-1
    MOVABLE=0
    FIXED=1
    #state definition is as follows:
    #[-1,rad,ctrX,ctrY,state[0]]+
    #[C1[i],C2[i],len1[i],len2[i],state[i] for i in range(K)]
    def __init__(self, state=[], K=5, B=10):
        self.state=state
        self.K=K
        self.B=B
        self.maxTrial=100
        if len(self.state) == 0:
            while True:
                self.state = self.generate_init_state()
                if self.check_topology_feasibility() and self.check_geometry_feasibility():
                    break
    def generate_init_state(self, ret=None):
        if ret is None:
            ret=[]
        maxDist=2.0*math.sqrt(2.0)*self.B
        for i in range(len(ret)//5,self.K):
            #state
            if i==0:
                state=self.MOVABLE
            elif i==1:
                state=random.choice([self.NOT_USED,self.FIXED])
            else:
                state=random.choice([self.NOT_USED,self.MOVABLE,self.FIXED])
            #initialize according to state
            if state==self.NOT_USED:
                break
            elif state==self.FIXED:
                ret+=[-1,-1,random.uniform(-self.B,self.B),random.uniform(-self.B,self.B),state]
            elif i==0:
                #ret+=[-1,random.uniform(self.B/10,self.B),random.uniform(-self.B,self.B),random.uniform(-self.B,self.B),state]
                ret+=[-1,random.uniform(self.B/10,self.B),0.0,0.0,state]
            else:
                s=[-1,-1,-1,-1,-1]
                while s[0]==s[1]:
                    s=[random.randint(0,i-1),random.randint(0,i-1),random.uniform(0,maxDist),random.uniform(0,maxDist),state]
                ret+=s
        #remove
        while ret[-1]==self.FIXED:
            ret=ret[0:len(ret)-5]
        return ret
    def check_feasibility(self, min_node_num=0):
        return self.check_data_validity() \
               and self.check_topology_feasibility() \
               and self.check_geometry_feasibility() \
               and self.nrN() > min_node_num
    # TODO: I add this to check some other cases to prevent inf. loop, please double check
    #  the checks are based on generate_init_state --Tina
    def check_data_validity(self, debug=False):
        debug = True # comment this out to reduce printing info
        data = self.state
        for i in range(self.nrN()):
            state = data[i * 5 + 4]
            first_connect = data[i * 5]
            second_connect = data[i * 5 + 1]
            if i == 0:
                if state is not MOVABLE:
                    if debug: print("motor not movable")
                    return False
                if first_connect != -1:
                    if debug: print("motor first connect wrong")
                    return False
            elif i == 1:
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
    # check the feasibility of the input solution
    def check_topology_feasibility(self):
        #check if there is at least one non-motor movable node
        if self.state[-1] != self.MOVABLE or len(self.state)//5<3:
            return False
        #check if every node is connected to the last node
        visited=[False for i in range(self.nrN())]
        visited[self.nrN()-1]=True
        stack=[self.nrN()-1]
        while len(stack)>0:
            id=stack[-1]
            stack.pop()
            if self.state[id*5+4]==self.FIXED or id==0:
                ss=[]
            else: ss=[self.state[id*5+0],self.state[id*5+1]]
            for i in ss:
                stack.append(i)
                visited[i]=True
        for i in range(self.nrN()):
            if not visited[i]:
                return False
        #check if every movable node is connected to motor-node
        visited=[False for i in range(self.nrN())]
        visited[0]=True
        stack=[0]
        while len(stack)>0:
            id=stack[-1]
            stack.pop()
            for j in range(id+1,self.nrN()):
                if self.state[j*5+0]==id or self.state[j*5+1]==id:
                    stack.append(j)
                    visited[j]=True
        for i in range(self.nrN()):
            if self.state[i*5+4]==self.MOVABLE and not visited[i]:
                return False
        return True
    def check_geometry_feasibility(self):
        self.link=self.set_to_linkage()
        return self.link.check_geometry_feasibility()
    def change_geometry(self):
        #we can change rad_motor,len1,len2
        #we cannot change ctr_motor
        ids=[]
        for i in range(self.nrN()):
            if i==0:
                ids+=[1]#ids+=[1,2,3]
            else: ids+=[i*5+2,i*5+3]
        id=random.choice(ids)
        ctr=self.state[id]
        delta=self.B
        trial=0
        while trial<self.maxTrial:
            val=random.uniform(ctr-delta,ctr+delta)
            self.state[id]=val
            if self.check_geometry_feasibility():
                break
            trial=trial+1
        return trial<self.maxTrial
    def add_node(self):
        if len(self.state)==self.K*5:
            return False
        nrN0=self.nrN()
        state0=copy.deepcopy(self.state)
        trial=0
        while trial<self.maxTrial:
            self.state=self.generate_init_state(ret=copy.deepcopy(state0))
            if self.check_topology_feasibility() and self.check_geometry_feasibility() and self.nrN()>nrN0:
                break
            trial=trial+1
        return trial<self.maxTrial
    def remove_node(self):
        if len(self.state)==5:
            return False
        self.state=self.state[0:len(self.state)-5]
        return self.check_topology_feasibility() and self.check_geometry_feasibility()
    def set_to_linkage(self):
        link=Linkage(self.nrN())
        link.rad_motor=self.state[1]
        link.ctr_motor=(self.state[2],self.state[3])
        for i in range(1,self.nrN()):
            link.U[i+1]=1
            state=self.state[i*5+4]
            if state==self.MOVABLE:
                link.F[i+1]=0
                link.C1[i+1]=self.state[i*5+0]+1
                link.C2[i+1]=self.state[i*5+1]+1
                link.len1[i+1]=self.state[i*5+2]
                link.len2[i+1]=self.state[i*5+3]
            elif state==self.FIXED:
                link.F[i+1]=1
                link.node_position[i+1]=(self.state[i*5+2],self.state[i*5+3])
            else: assert False
        return link
    def nrN(self):
        return len(self.state)//5
    def move(self):
        self.mutation()
    def mutation(self):
        state0=copy.deepcopy(self.state)
        while True:
            self.state=copy.deepcopy(state0)
            op_type=random.randint(0,2)
            if op_type==0:
                #change length
                if self.change_geometry():
                    break
            elif op_type==1:
                #add node
                if self.add_node():
                    break
            else:
                assert op_type==2
                #remove node
                if self.remove_node():
                    break
    def cross_over_once(self, mother, father):
        nrN_mother = len(mother)//5
        nrN_father = len(father)//5
        nrN_max = max(nrN_mother,nrN_father)
        result = []
        for i in range(nrN_max):
            choice=random.randint(0,1)
            if choice == 0:
                if len(mother) < i*5+5:
                    break
                result += mother[i*5:i*5+5]
            else:
                if len(father) < i*5+5:
                    break
                result += father[i*5:i*5+5]
        return result
    def cross_over(self, mother, father):
        while True: #this will always terminate by only choosing mother/father
            self.state = self.cross_over_once(mother, father)
            if self.check_geometry_feasibility() and self.check_geometry_feasibility():
                break
    def energy(self):
        link=self.set_to_linkage()
        robot=create_robot(link, sep=5.)
        if robot is None:
            return 0.
        else: return -robot.eval_performance(10.)

# Note that data means the old state data, and solution is for GA
def data_to_solution(linkage):
    #print("data", data)
    solution = linkage.state
    cur_node_num = linkage.nrN()
    if cur_node_num < MAX_NODE:
        # print(len(solution), solution)
        for i in range(MAX_NODE - cur_node_num):
            solution += [-1, -1, -1, -1, -1]
        # print("after extension", len(solution), solution)
    #print("solution", solution)
    return np.array(solution)

def solution_to_data(solution):
    #print("solution", solution)
    solution = solution.tolist()
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
    linkage = LinkageGA(state=copy.deepcopy(data))
    #print("linkage", linkage.state)
    return linkage

# generate a single initial solution
def generate_solution():
    # check initial performance
    linkage = LinkageGA()
    link = linkage.set_to_linkage()
    robot = create_robot(link, sep=5.)
    if robot is not None:
        print("performance", robot.eval_performance(10.))
    print(linkage.state)
    return data_to_solution(linkage)

# generate initial population
def generate_population(sol_per_pop):
    print("start generate initial population")
    initial_population = []
    for i in range(sol_per_pop):
        solution = generate_solution()
        initial_population.append(solution)
        print(i, len(solution), solution)
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

def fitness_func(ga_instance, solution, solution_idx):
    #print("solution index:", solution_idx)
    #print(solution)
    linkage = solution_to_data(solution)
    #print(linkage.state)
    if not linkage.check_feasibility():
        #TODO: Can we modify/reset the data to make it feasible? Or just dump it?
        return -1
    link = linkage.set_to_linkage()
    robot = create_robot(link, sep=5.)
    if robot is None:
        return 0.
    else:
        return robot.eval_performance(10.)

def crossover_func(parents, offspring_size, ga_instance):
    offspring = []
    idx = 0
    while len(offspring) != offspring_size[0]:
        parent1 = parents[idx % parents.shape[0], :].copy()
        parent2 = parents[(idx + 1) % parents.shape[0], :].copy()

        linkageP1 = solution_to_data(parent1)
        linkageP2 = solution_to_data(parent2)
        linkageChild = LinkageGA()

        #print("mother", parent1)
        #print("father", parent2)

        #print("motherData", linkageP1.state, type(linkageP1.state))
        #print("fatherData", linkageP2.state, type(linkageP2.state))
        linkageChild.cross_over(linkageP1.state, linkageP2.state)
        #print("childData", linkageChild.state, type(linkageP2.state))
        child = data_to_solution(linkageChild)

        #print("child", child)
        offspring.append(child)
        idx += 1

    return numpy.array(offspring)

def mutation_func(offspring, ga_instance):
    for chromosome_idx in range(offspring.shape[0]):
        linkage = solution_to_data(offspring[chromosome_idx])
        #print("before mutation", offspring[chromosome_idx])
        linkage.mutation()
        offspring[chromosome_idx] = data_to_solution(linkage)
        #print("after mutation", offspring[chromosome_idx])
    return offspring

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

    #crossover_type = "single_point"

    #mutation_type = "random"
    #mutation_percent_genes = 50

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
                           crossover_type=crossover_func,
                           mutation_type=mutation_func,
                           #mutation_percent_genes=mutation_percent_genes,
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

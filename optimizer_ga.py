from linkage_physics import *
import pygad
import numpy
import copy,pickle

class LinkageGA():
    NOT_USED=-1
    MOVABLE=0
    FIXED=1
    #state definition is as follows:
    #[-1,rad,ctrX,ctrY,state[0]]+
    #[C1[i],C2[i],len1[i],len2[i],state[i] for i in range(K)]
    def __init__(self, K=5, B=10):
        self.K=K
        self.B=B
        self.maxTrial=100
        while True:
            self.state=self.generate_init_state()
            if self.check_topology_feasibility() and self.check_geometry_feasibility() and self.nrN()>3:
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
        #while ret[-1]==self.FIXED:
        #    ret=ret[0:len(ret)-5]
        return ret
    def check_topology_feasibility(self):
        #check if every node is connected to the last node
        visited=[False for i in range(self.nrN())]
        visited[self.nrN()-1]=True
        stack=[self.nrN()-1]
        while len(stack)>0:
            id=int(stack[-1])
            stack.pop()
            if self.state[id*5+4]==self.FIXED or id==0:
                ss=[]
            else: ss=[self.state[id*5+0],self.state[id*5+1]]
            for i in ss:
                stack.append(i)
                # if (abs(i - int(i)) > 0.1): print(i)
                visited[int(i)]=True
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
    def check_geometry_feasibility(self,nrSample=32):
        self.link=self.set_to_linkage()
        for i in range(nrSample):
            theta,succ=self.link.forward_kinematic(math.pi*2*i/nrSample)
            if not succ:
                return False
            for pos in self.link.node_position:
                for d in range(2):
                    if pos[d]<-self.B or pos[d]>self.B:
                        return False
        return True
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
        state0=self.state[0:len(self.state)-5]
        while state0[-1]==self.FIXED:
            state0=state0[0:len(state0)-5]
        trial=0
        while trial<self.maxTrial:
            self.state=self.generate_init_state(ret=copy.deepcopy(state0))
            if self.check_topology_feasibility() and self.check_geometry_feasibility() and self.nrN()>0:
                break
            trial=trial+1
        return trial<self.maxTrial
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
    def energy(self):
        link=self.set_to_linkage()
        robot=create_robot(link, sep=5.)
        if robot is None:
            return 0.
        else: return -robot.eval_performance(10.)

LINK = LinkageGA()

def fitness_func(ga_instance, solution, solution_idx):
    print(solution)
    LINK.state = solution
    if not LINK.check_topology_feasibility():
        return -1
    if not LINK.check_geometry_feasibility():
        return -1
    link = self.set_to_linkage()
    robot = create_robot(link, sep=5.)
    if robot is None:
        return 0.
    else:
        return robot.eval_performance(10.)

# def fitness_func(ga_instance, solution, solution_idx):
#     ...
#     return [fitness1, fitness2, ..., fitnessN]
# '''



if __name__ == '__main__':
    K = 5
    B = 10
    maxDist = 2.0 * math.sqrt(2.0) * B

    print("state", LINK.state)
    #print("input", function_inputs)
    function_inputs = LINK.state
    desired_output = 15
    for i in range(len(function_inputs) // 5, K):
        function_inputs += [0, 0, 0, 0, 0]

    fitness_function = fitness_func

    num_generations = 50
    num_parents_mating = 4

    sol_per_pop = 10
    num_genes = len(function_inputs)

    init_range_low = -2
    init_range_high = 5
    # state definition is as follows:
    # [-1,rad,ctrX,ctrY,state[0]]+
    # [C1[i],C2[i],len1[i],len2[i],state[i] for i in range(K)]
    gene_space = [[-1], random.uniform(B/10,B), [0.0], [0.0], [0]]
    gene_space += [[-1], [-1], random.uniform(-B, B), random.uniform(-B, B), [1]]
    for i in range(2, K):
        gene_space+=[random.randint(-1,i-1),random.randint(-1,i-1),random.uniform(0,maxDist),random.uniform(0,maxDist),[-1, 0, 1]]

    parent_selection_type = "sss"
    keep_parents = 1

    crossover_type = "single_point"

    mutation_type = "random"
    mutation_percent_genes = 50

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

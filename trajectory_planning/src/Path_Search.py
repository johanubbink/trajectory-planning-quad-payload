from Node import *
import heapq #Tree structure to sort Q
import numpy as np #Python matrix library
from collections import deque #FIFO Q for forward search


class Astar:
    #initialize the search algorithm
    def __init__(self, xi, goal, U, valid_u, f, cost_to_come, cost_to_go, bounds):
        self.xi = xi
        self.U = U
        self.valid_u = valid_u
        self.goal = goal
        self.f = f
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.bounds = bounds
        self.path_found = False
        
    #function to search for path    
    def search_path(self):
        #Create the initial node
        x_node = Node(0,0,self.xi,0,0,0)
        
        #Add initial node to list of all nodes
        self.nodes_list = [x_node]
        self.path_found = False
        last_node = 0
        
        #insert initial node_id into Q
        Q = [] #Q - Nodes to still be visited
        t_cost = x_node.cost_to_come + x_node.cost_to_go
        #add node id and node cost to Q
        #heapq.heappop will return node id with lowest cost
        heapq.heappush(Q, (t_cost, x_node.node_id))
        
        #Check if the goal is reached
        x = self.xi
        if (self.goal(x)):
            self.path_found = True
            
        while (Q and (not self.path_found)):
            #get node with lowest cost in Q
            node_id = heapq.heappop(Q)[1]
            self.nodes_list[node_id].visited = True
            node = self.nodes_list[node_id]
            x = node.state
            #print (node)
            if self.goal(x):
                self.path_found  = True
                last_node = node
            
            #run all inputs on current node
            for u, input_type  in zip(self.U, range(len(self.U))):

                #get next state
                if self.valid_u(x,input_type,node.input_type):

                    result = self.f(x,u)
                    #print (input_type)
                    xmod = result[:,-1] #get last state at end of f(x,u)
                
                    if (self.bounds(result)):
                        new_cost = node.cost_to_come + self.cost_to_come(xmod,input_type,node)
                        new_node = Node(node.node_id,len(self.nodes_list),xmod,
                                        input_type,
                                        new_cost,
                                        self.cost_to_go(xmod))
                    

                        #add node_cost and node_id to Q
                        t_cost = new_node.cost_to_come + new_node.cost_to_go
                        heapq.heappush(Q, ( t_cost, new_node.node_id))
                        #add node to list of all nodes.
                        self.nodes_list.append(new_node)
                    
        #print (last_node)
        #traverse tree back to root to get path
        path = self.trace_back(self.nodes_list,last_node)
        self.path = path
        self.Q = Q
        #self.nodes_list = nodes_list
        return path
        
    
    #function to get path last node
    def trace_back(self, nodes_list, last_node):
        path = [last_node]
        parent = last_node.parent_id
        while (parent != 0):
            next_node = nodes_list[parent]
            #print("Cost to come: ",next_node.cost_to_come)
            #print("Cost to go: ",next_node.cost_to_go)      
            path.append(next_node)
            parent = next_node.parent_id
        return np.fliplr([path])[0]
    
    #function that calculates input sequence for planned path
    def get_input_seq(self):
        if self.path_found:
            input_sequence = self.U[self.path[0].input_type]
            for node in self.path[1:]:
                #print("Cost to come: ",node.cost_to_come)
                #print("Cost to go: ",node.cost_to_go)                
                input_sequence = np.concatenate([input_sequence,
                                             self.U[node.input_type]])
            return input_sequence
        else:
            return "No path is found yet"

class Astar_hier_cost:
    #initialize the search algorithm
    def __init__(self, xi, goal, U, f, cost_to_come, cost_to_go, bounds, spec_pop):
        #spec_pop(heap,nodelist) return node_id, Q
        self.xi = xi
        self.U = U
        self.goal = goal
        self.f = f
        self.cost_to_come = cost_to_come
        self.cost_to_go = cost_to_go
        self.bounds = bounds
        self.path_found = False
        self.spec_pop = spec_pop
        
    #function to search for path    
    def search_path(self):
        #Create the initial node
        x_node = Node(0,0,self.xi,0,0, self.cost_to_go(self.xi))
        
        #Add initial node to list of all nodes
        self.nodes_list = [x_node]
        self.path_found = False
        last_node = 0
        
        #insert initial node_id into Q
        Q = [] #Q - Nodes to still be visited
        t_cost = x_node.cost_to_come + x_node.cost_to_go
        #add node id and node cost to Q
        #heapq.heappop will return node id with lowest cost
        heapq.heappush(Q, (t_cost, x_node.node_id))
        
        #Check if the goal is reached
        x = self.xi
        if (self.goal(x)):
            self.path_found = True
            
        while (Q and (not self.path_found)):
            #get node with lowest cost in Q
            #node_id = heapq.heappop(Q)[1]
            node_id, Q = self.spec_pop(Q,self.nodes_list)
            self.nodes_list[node_id].visited = True
            node = self.nodes_list[node_id]
            x = node.state
            #print (node)
            if self.goal(x):
                self.path_found  = True
                last_node = node
                
            #run all inputs on current node
            for u, input_type  in zip(self.U, range(len(self.U))):
                #get next state
                result = self.f(x,u)
                
                xmod = result[:,-1] #get last state at end of f(x,u)
                
                if (self.bounds(result)):
                    new_cost_c = node.cost_to_come + self.cost_to_come(xmod,input_type,node.input_type)
                    new_node = Node(node.node_id,len(self.nodes_list),xmod,
                                    input_type,
                                    new_cost_c,
                                    self.cost_to_go(xmod))
                    

                    #add node_cost and node_id to Q
                    #t_cost = new_node.cost_to_come + new_node.cost_to_go
                    t_cost = new_node.cost_to_go
                    heapq.heappush(Q, ( t_cost, new_node.node_id))
                    #add node to list of all nodes.
                    self.nodes_list.append(new_node)
                    
        #print (last_node)
        #traverse tree back to root to get path
        path = self.trace_back(self.nodes_list,last_node)
        self.path = path
        self.Q = Q
        #self.nodes_list = nodes_list
        return path
        
    
    #function to get path last node
    def trace_back(self, nodes_list, last_node):
        path = [last_node]
        parent = last_node.parent_id
        while (parent != 0):
            next_node = nodes_list[parent]
            path.append(next_node)
            parent = next_node.parent_id
        return np.fliplr([path])[0]
    
    #function that calculates input sequence for planned path
    def get_input_seq(self):
        if self.path_found:
            input_sequence = self.U[self.path[0].input_type]
            for node in self.path[1:]:
                input_sequence = np.concatenate([input_sequence,
                                             self.U[node.input_type]])
            return input_sequence
        else:
            return "No path is found yet"
    
    
    

class Forward_Search:
    def __init__(self, xi, goal, U, f, bounds):
        self.xi = xi
        self.U = U
        self.goal = goal
        self.f = f
        self.bounds = bounds
    
    def trace_back(self,nodes_list, last_node):
        #function to get path from nodes list
        path = [last_node]
        parent = last_node.parent_id
        while (parent != 0):
            next_node = nodes_list[parent]
            path.append(next_node)
            parent = next_node.parent_id
        return np.fliplr([path])[0]
    
    def get_input_seq(self):
        input_sequence = self.U[self.path[0].input_type]
        for node in self.path[1:]:
            input_sequence = np.concatenate([input_sequence,
                                             self.U[node.input_type]])
        return input_sequence
    
    def search_path(self):
        #Create the initial node
        x_node = Node(0,0,self.xi,0,0)
        
        #Add initial node to list of all nodes
        nodes_list = [x_node]
        path_found = False
        last_node = 0
        
        #insert initial node_id into Q
        Q = deque([x_node.node_id])
        
        #Check if the goal is reached
        x = self.xi
        if (self.goal(x)):
            path_found = True
            print ("Success")
        else:
            path_found = False
            
        while (Q and (not path_found)):
            node = nodes_list[Q.popleft()]
            x = node.state
                
            #run all inputs on current node
           
            for u, input_type  in zip(self.U, range(len(self.U))):
                #get next state

                result = self.f(x,u)
                xmod = result[:,-1]
                if (self.bounds(result)):
                    
                    new_node = Node(node.node_id,len(nodes_list),xmod,input_type)
                    
                    #Check if node reached goal
                    if self.goal(xmod):
                        path_found  = True
                        last_node = new_node
                    
                    #add node_cost and node_id to Q
                    Q.append(new_node.node_id)
                    #add node to list of all nodes.
                    nodes_list.append(new_node)   

        path = self.trace_back(nodes_list,last_node)
        self.path = path
        self.Q = Q
        self.nodes_list = nodes_list
        return path

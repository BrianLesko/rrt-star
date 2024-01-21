# Brian Lesko
# 1/17/2024
# RRT Class

import numpy as np 

class RRT:
    def __init__(self, start, goal):
        self.n = len(start) # n is the dimension of the space
        self.start = start
        self.goal = goal
        self.nodes = [self.start]  # Use list for dynamic appending
        self.node_x = [start[0]]
        self.node_y = [start[1]]
        self.edges = []  # Store edges as tuples (parent, child)
        self.costs = [0]
        self.parents = [0]

    def get_x_sample(self):
        self.x_sample = 6*np.random.uniform(-1, 1, self.n)
        return self.x_sample
    
    def get_nearest_node(self):
        distances = np.linalg.norm(self.nodes - self.x_sample, axis=1)
        nearest_index = np.argmin(distances)
        self.cost_of_nearest = self.get_cost(nearest_index)
        # RRT*
        self.x_nearest_index = nearest_index
        self.parents.append(nearest_index)
        return nearest_index
    
    def get_x_new(self, max_step = 1.333):
        vector = self.x_sample - self.nodes[self.x_nearest_index]
        distance = np.linalg.norm(vector)
        step_size = min(max_step, distance)*np.random.normal(.777, (0.9 - 0.5) / (2 * 0.675))
        step = step_size * vector / distance
        self.x_new = self.nodes[self.x_nearest_index] + step
        # RRT*
        self.cost = distance + self.cost_of_nearest
        self.costs.append(self.cost)
        self.x_new_idx = len(self.nodes)
        return self.x_new
    
    # RRT*
    def get_x_near_list_idx(self,radius=.888):
        distances = np.linalg.norm(self.nodes - self.x_new, axis=1)
        self.near_indices = np.where(distances < radius)[0]
        return self.near_indices

    def check_costs(self):
        for i in self.near_indices:
            current_cost = self.get_cost(i)
            distance_to_x_new = np.linalg.norm(self.nodes[i] - self.x_new)
            cost_of_x_new = self.cost 
            cost_using_x_new = cost_of_x_new + distance_to_x_new
            if cost_using_x_new < current_cost:
                self.costs[i] = cost_using_x_new
                self.parents[i] = self.x_new_idx
                #self.edges.append((i, len(self.nodes) - 1))
            
    def get_cost(self, node_index):
        return self.costs[node_index]

    def rewire(self, x_new_index, radius):
        for i, x_near in enumerate(self.nodes):
            if np.linalg.norm(np.array(x_near) - np.array(self.nodes[x_new_index])) < radius:
                cost_through_x_new = self.get_cost(x_new_index) + np.linalg.norm(np.array(x_near) - np.array(self.nodes[x_new_index]))
                if cost_through_x_new < self.get_cost(i):
                    self.costs[i] = cost_through_x_new
                    # Update the parent of x_near
                    self.edges = [(parent, child) if child != i else (x_new_index, child) for parent, child in self.edges]

    def check_collision(self):
        x_new = self.x_new 
        self.nodes.append(x_new)
        self.node_x.append(x_new[0])
        self.node_y.append(x_new[1])
        # self.rewire(len(self.nodes) - 1, radius=1.0)  # Add this line
        return False
    


        
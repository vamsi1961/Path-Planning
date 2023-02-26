import numpy as np
import matplotlib.pyplot as plt
import math

show_animation = True


class Node:

    def __init__(self,G=0,H=0,coordinate = None, parent = None):

        self.G = G
        self.H = H
        self.F = G+H
        self.parent = parent
        self.coordinate = coordinate
        

    def reset_f(self):
        self.F = self.G + self.H

def hcost(node_coordinate,goal):
    
    dx = abs(node_coordinate[0] - goal[0])
    dy = abs(node_coordinate[1] - goal[1])
    hcost = dx+dy
    return hcost
    

def gcost(fixed_node , update_node_coordinate):
    dx = abs(fixed_node[0] - update_node_coordinate[0])
    dy = abs(fixed_node[1] - update_node_coordinate[1])
    gc = math.hypot(dx,dy)


def boundary_And_obstacles(start,goal,top_vertex,bottom_vertex,obs_number):
    lry = list(range(bottom_vertex[1],top_vertex[1]))
    lx = [bottom_vertex[0]] * len(lry)
    rx = [top_vertex[0]] * len(lry)

    dx = list(range(bottom_vertex[0]+1,top_vertex[0]))
    ux = [bottom_vertex[0]] + dx + [top_vertex[0]]
    uy = [top_vertex[1]]*len(ux)
    dy = [bottom_vertex[1]]*len(dx)

    x = lx + rx + ux + dx
    y = lry + lry + uy+dy

    ob_x = np.random.randint(bottom_vertex[0]+1,top_vertex[0],obs_number).tolist()
    ob_y = np.random.randint(bottom_vertex[1]+1,top_vertex[1],obs_number).tolist()

    #stack obx,oby and boundaries,
    obstacles = np.vstack(ob_x,ob_y).T.tolist()
    obstacles = [pos for pos in obstacles if pos != start and pos != goal]
    obs_array = np.array(obstacles)
    boundaries = np.vstack(x,y).T
    bound_obs = np.vstack((boundaries,obs_array))

    return bound_obs,obstacles


def find_neighbor(node,ob,closed):
    pass

def find_node_index(coordinate,node_list):
    pass

def find_path(open_list,closed_list,goal,onstacle):
    pass

def node_to_coordinates(node_list):
    pass

def check_node_coincide(close_ls1,closed_ls2):
    pass

def find_surrounding(coordinate,obstacle):
    pass

def get_border_line(node_closed_ls,obstacle):
    pass


def get_path(org_list,goal_list,coordinate):
    pass

def random_coordinate(bottom_vertex,top_vertex):
    pass

def draw(close_origin,close_goal,start_end,end,bound):
    pass

def draw_control(org_closed,goal_closed,flag,start,end,bound,obstacle):
    pass

def search_control(start,end,bound,obstacle):
    pass

def main(obstacle_number = 1500):
    pass

if __name__ == '__main__':
    main(obstacle_number= 1500)












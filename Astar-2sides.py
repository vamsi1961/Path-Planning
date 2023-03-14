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


def boundary_and_obstacles(start,goal,top_vertex,bottom_vertex,obs_number):
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
    ob_list = ob.tolist()
    neighbor : list = []
    for x in range(node.coordinate[0] - 1 ,node.coordinate[0] +2 ):
        for y in range(node.coordinate[0] - 1 ,node.coordinate[0] +2 ):
            if [x,y] not in ob_list:
                neighbor.append([x,y])

    neighbor.remove(node.coordinate)

    top_nei = [node.cooridnate[0] , node.cooridnate[0] + 1]
    bom_nei = [node.cooridnate[0] , node.cooridnate[0] - 1]
    left_nei = [node.cooridnate[0]-1 , node.cooridnate[0] ]
    right_nei = [node.cooridnate[0] +1 , node.cooridnate[0] ]

    lt_nei = [node.cooridnate[0]-1 , node.cooridnate[0] + 1]
    rt_nei = [node.cooridnate[0]+1 , node.cooridnate[0] + 1]
    lb_nei = [node.cooridnate[0]-1 , node.cooridnate[0] -1]
    rb_nei = [node.cooridnate[0] +1 , node.cooridnate[0] -1]

    if top_nei and left_nei in ob_list and lt_nei in neighbor:
        neighbor.remove(lt_nei)
    
    if top_nei and right_nei in ob_list and rt_nei in neighbor:
        neighbor.remove(rt_nei)

    if bom_nei and left_nei in ob_list and lb_nei in neighbor:
        neighbor.remove(lb_nei)

    if bom_nei and right_nei in ob_list and rb_nei in neighbor:
        neighbor.remove(rb_nei)

    neighbor = [x for x in neighbor if x not in closed]
    return neighbor


def find_node_index(coordinate,node_list):
    n = [node for node in node_list if node.coordinate == coordinate]
    return n

def find_path(open_list,closed_list,goal,onstacle):
    pass

def node_to_coordinates(node_list):
    coordinate_list = [node.coordinate for node in node_list ]
    return coordinate_list
 
def check_node_coincide(closed_ls1,closed_ls2):
    n1 = node_to_coordinates(closed_ls1)
    n2 = node_to_coordinates(closed_ls2)

    intersect_ls = [node for node in n1 if node in n2]

    return intersect_ls


# return surroundings, obstacles arround node and draw the border line 
def find_surrounding(coordinate,obstacle):
    boundary : list =[]

    for x in range(coordinate[0] - 1 ,coordinate[0] +2 ):
        for y in range(coordinate[0] - 1 ,coordinate[0] +2 ):
            if [x,y] in obstacle:
                boundary.append([x,y])

    return boundary

def get_border_line(node_closed_ls,obstacle):
    pass


def get_path(org_list,goal_list,coordinate):
    pass

def random_coordinate(bottom_vertex,top_vertex):
    coordinate = [np.random.randint(bottom_vertex[0] +1 , top_vertex[0]), np.random.randint(bottom_vertex[1] +1 , top_vertex[1])]

    return coordinate

def draw(close_origin,close_goal,start,end,bound):
    # plot the map
    if not close_goal.tolist():  # ensure the close_goal not empty
        # in case of the obstacle number is really large (>4500), the
        # origin is very likely blocked at the first search, and then
        # the program is over and the searching from goal to origin
        # will not start, which remain the closed_list for goal == []
        # in order to plot the map, add the end coordinate to array
        close_goal = np.array([end])
    plt.cla()
    plt.gcf().set_size_inches(11, 9, forward=True)
    plt.axis('equal')
    plt.plot(close_origin[:, 0], close_origin[:, 1], 'oy')
    plt.plot(close_goal[:, 0], close_goal[:, 1], 'og')
    plt.plot(bound[:, 0], bound[:, 1], 'sk')
    plt.plot(end[0], end[1], '*b', label='Goal')
    plt.plot(start[0], start[1], '^b', label='Origin')
    plt.legend()
    plt.pause(0.0001)

def draw_control(org_closed,goal_closed,flag,start,end,bound,obstacle):
    pass


# 
def searching_control(start,end,bound,obstacle):
    
    origin = Node(coordinate = start , H = hcost(start,end))
    goal = Node (coordinate= end, H = hcost(end,start))

    origin_open : list = [origin]
    origin_close : list = []

    goal_open = [goal]
    goal_close = []

    target_goal = end

    flag = 0

    path = None

    while True:
        origin_open , origin_close = 



def main(obstacle_number = 1500):
    
    top_vertex = [60,60]
    bottom_vertex = [0,0]

    start = random_coordinate(bottom_vertex,top_vertex)
    end = random_coordinate(bottom_vertex,top_vertex)
    
    bound,obstacle = boundary_and_obstacles(start, end , top_vertex , bottom_vertex , obstacle_number)

    path = searching_control(start , end , bound , obstacle)

    if not show_animation:
        print(path)


if __name__ == '__main__':
    main(obstacle_number= 1500)





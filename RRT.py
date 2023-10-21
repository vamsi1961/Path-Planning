import math
import random

import matplotlib.pyplot as plt
import numpy as np

show_animation = True

class RRT:
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class Bounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self, start, goal, obstacles, rand_area, expand_dist=3.0, path_resolution=0.5, goal_sample_rate=5, max_iterations=500, play_area=None, robot_radius=0.0):
        self.start = self.Node(start[0], start[1])
        self.goal = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.play_area = None
        if play_area is not None:
            self.play_area = self.Bounds(play_area)
        self.expand_dist = expand_dist
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iterations = max_iterations
        self.obstacle_list = obstacles
        self.node_list = []
        self.robot_radius = robot_radius

    def plan(self, animation=True):
        self.node_list = [self.start]
        for i in range(self.max_iterations):
            random_node = self.get_random_node()
            nearest_index = self.get_nearest_node_index(self.node_list, random_node)
            nearest_node = self.node_list[nearest_index]
            new_node = self.steer(nearest_node, random_node, self.expand_dist)
            if self.check_inside_play_area(new_node, self.play_area) and self.check_collision(new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)
            if animation and i % 5 == 0:
                self.draw_tree(random_node)
            if self.calculate_distance_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dist:
                final_node = self.steer(self.node_list[-1], self.goal, self.expand_dist)
                if self.check_collision(final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_path(len(self.node_list) - 1)
            if animation and i % 5:
                self.draw_tree(random_node)
        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        dist, angle = self.calculate_distance_and_angle(new_node, to_node)
        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        if extend_length > dist:
            extend_length = dist
        num_expand = math.floor(extend_length / self.path_resolution)
        for _ in range(num_expand):
            new_node.x += self.path_resolution * math.cos(angle)
            new_node.y += self.path_resolution * math.sin(angle)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        dist, _ = self.calculate_distance_and_angle(new_node, to_node)
        if dist <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y
        new_node.parent = from_node
        return new_node

    def generate_final_path(self, goal_index):
        path = [[self.goal.x, self.goal.y]]
        node = self.node_list[goal_index]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        return path

    def calculate_distance_to_goal(self, x, y):
        dx = x - self.goal.x
        dy = y - self.goal.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rand = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:
            rand = self.Node(self.goal.x, self.goal.y)
        return rand

    def draw_tree(self, random_node):
        plt.clf()
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if random_node is not None:
            plt.plot(random_node.x, random_node.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(random_node.x, random_node.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, random_node):
        dist_list = [(node.x - random_node.x) ** 2 + (node.y - random_node.y) ** 2 for node in node_list]
        min_index = dist_list.index(min(dist_list))
        return min_index

    @staticmethod
    def check_inside_play_area(node, play_area):
        if play_area is None:
            return True
        if node.x < play_area.xmin or node.x > play_area.xmax or node.y < play_area.ymin or node.y > play_area.ymax:
            return False
        return True

    @staticmethod
    def check_collision(node, obstacle_list, robot_radius):
        if node is None:
            return False
        for (ox, oy, size) in obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]
            if min(d_list) <= (size + robot_radius) ** 2:
                return False
        return True

    @staticmethod
    def calculate_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dist = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        return dist, angle

def main(goal_x=6.0, goal_y=10.0):
    print("Start " + __file__)

    obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]
    rrt = RRT(start=[0, 0], goal=[goal_x, goal_y], obstacles=obstacle_list, rand_area=[-2, 15], robot_radius=0.8)
    path = rrt.plan(animation=show_animation)

    if path is None:
        print("Cannot find a path")
    else:
        print("Found a path!")

        if show_animation:
            rrt.draw_tree()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)
            plt.show()

if __name__ == '__main__':
    main()

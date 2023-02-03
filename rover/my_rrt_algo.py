import numpy as np
from random import random, randint
import matplotlib.pyplot as plt
from matplotlib import collections as mc
from collections import deque


class Line():
    ''' Define line '''
    def __init__(self, p0, p1):
        self.p = np.array(p0)
        self.dirn = np.array(p1) - np.array(p0)
        self.dist = np.linalg.norm(self.dirn)
        self.dirn = self.dirn / self.dist # normalize

    def path(self, t):
        return self.p + t * self.dirn

class Graph:
    def __init__(self, start_pos: tuple, goal_pos: tuple):
        self.start_pos = start_pos
        self.goal_pos = goal_pos

        self.vertices = [self.start_pos]
        self.edges = []
        self.success = False
        print("type of start pos")
        print(self.start_pos)
        print(type(self.start_pos))

        self.vert2indx = {self.start_pos: 0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

    def add_vertice(self, pos: tuple):
        try:
            indx = self.vert2indx[pos]
        except:
            indx = len(self.vertices)
            self.vertices.append(pos)
            self.vert2indx[pos] = indx
            self.neighbors[indx] = []
        return indx

    def add_edge(self, indx1, indx2, cost):
        self.edges.append((indx1, indx2))
        self.neighbors[indx1].append((indx2, cost))
        self.neighbors[indx2].append((indx1, cost))


class RRT(object):
    def __init__(self, start_pos: tuple, goal_pos: tuple, n_iter, radius, step_size, obstacles: list):
        self.start_pos = start_pos
        self.goal_pos = goal_pos
        self.graph = Graph(start_pos, goal_pos)
        self.n_iter = n_iter
        self.radius = radius
        self.step_size = step_size
        self.obstacles = obstacles

    def intersection(self, line, center):
        ''' Check line-sphere (circle) intersection '''
        a = np.dot(line.dirn, line.dirn)
        b = 2 * np.dot(line.dirn, line.p - center)
        c = np.dot(line.p - center, line.p - center) - self.radius * self.radius
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return False
        t1 = (-b + np.sqrt(discriminant)) / (2 * a);
        t2 = (-b - np.sqrt(discriminant)) / (2 * a);
        if (t1 < 0 and t2 < 0) or (t1 > line.dist and t2 > line.dist):
            return False
        return True
    

    def distance(self, vex, obstacle):
        return np.sqrt(pow((vex[0]-obstacle[0]),2)+pow((vex[1]-obstacle[1]),2))

    def is_colliding(self):
        for obstacle in self.obstacles:
            if self.distance(self.random_vex, obstacle) < self.radius:
                return True
            else:
                return False

    def line_collides_obstacle(self, line):
        for obs in self.obstacles:
            if self.intersection(line, obs):
                return True
        return False

    def nearest(self):
        Nvex = None
        Nidx = None
        minDist = float("inf")

        for idx, v in enumerate(self.graph.vertices):
            line = Line(v, self.random_vex)
            if self.line_collides_obstacle(line):
                continue

            dist = self.distance(self.random_vex,v)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v

        return Nvex, Nidx

    def create_new_vertex(self, nearest_vertice):
        dirn = np.array(self.random_vex) - np.array(nearest_vertice)
        length = np.linalg.norm(dirn)
        dirn = (dirn / length) * min(self.step_size, length)

        new_vertex = (nearest_vertice[0]+dirn[0], nearest_vertice[1]+dirn[1])
        return new_vertex


    def run(self):
        for _ in range(self.n_iter):
            print("start pos", self.start_pos)
            print("goal pos", self.goal_pos)
            if (self.start_pos[0] < self.goal_pos[0]):
                random_pos_x = randint(self.start_pos[0], self.goal_pos[0])
            else:
                random_pos_x = randint(self.goal_pos[0], self.start_pos[0])

            if (self.start_pos[1] < self.goal_pos[1]):
                random_pos_y = randint(self.start_pos[1], self.goal_pos[1])
            else:
                random_pos_y = randint(self.goal_pos[1], self.start_pos[1])
            
            # random_pos_y = randint(self.start_pos[1], self.goal_pos[1])
            self.random_vex = (random_pos_x, random_pos_y)

            if self.is_colliding():
                continue

            nearest_vertice, near_indx = self.nearest()
            if nearest_vertice is None:
                continue

            new_vertex = self.create_new_vertex(nearest_vertice)
            new_indx = self.graph.add_vertice(new_vertex)
            distance = self.distance(new_vertex, nearest_vertice)
            self.graph.add_edge(new_indx, near_indx, distance)

            distance = self.distance(new_vertex, self.graph.goal_pos)
            if distance < 2 * self.radius:
                end_indx = self.graph.add_vertice(self.graph.goal_pos)
                self.graph.add_edge(new_indx, end_indx, distance)
                self.graph.success = True
                print("success")
                break
        return self.graph

class Dijkstra(object):
    def __init__(self, graph: Graph):
        self.graph = graph
    def run(self):
        srcIdx = self.graph.vert2indx[self.graph.start_pos]
        dstIdx = self.graph.vert2indx[self.graph.goal_pos]

        # build dijkstra
        nodes = list(self.graph.neighbors.keys())
        dist = {node: float('inf') for node in nodes}
        prev = {node: None for node in nodes}
        dist[srcIdx] = 0

        while nodes:
            curNode = min(nodes, key=lambda node: dist[node])
            nodes.remove(curNode)
            if dist[curNode] == float('inf'):
                break

            for neighbor, cost in self.graph.neighbors[curNode]:
                newCost = dist[curNode] + cost
                if newCost < dist[neighbor]:
                    dist[neighbor] = newCost
                    prev[neighbor] = curNode

        # retrieve path
        path = deque()
        curNode = dstIdx
        while prev[curNode] is not None:
            path.appendleft(self.graph.vertices[curNode])
            curNode = prev[curNode]
        path.appendleft(self.graph.vertices[curNode])
        return list(path)

        
def plot(G, obstacles, radius, path=None):
    '''
    Plot RRT, obstacles and shortest path
    '''
    px = [x for x, y in G.vertices]
    py = [y for x, y in G.vertices]
    fig, ax = plt.subplots()

    for obs in obstacles:
        circle = plt.Circle(obs, radius, color='red')
        ax.add_artist(circle)

    ax.scatter(px, py, c='cyan')
    ax.scatter(G.start_pos[0], G.start_pos[1], c='black')
    ax.scatter(G.goal_pos[0], G.goal_pos[1], c='black')

    lines = [(G.vertices[edge[0]], G.vertices[edge[1]]) for edge in G.edges]
    lc = mc.LineCollection(lines, colors='green', linewidths=2)
    ax.add_collection(lc)

    if path is not None:
        paths = [(path[i], path[i+1]) for i in range(len(path)-1)]
        lc2 = mc.LineCollection(paths, colors='blue', linewidths=3)
        ax.add_collection(lc2)

    ax.autoscale()
    ax.margins(0.1)
    plt.show()


        
def main():
    startpos = (228., 258.)
    endpos = (405., 293.)
    # obstacles = [(1., 1.), (2., 2.)]
    obstacles = []
    n_iter = 200
    radius = 5.0
    # stepSize = 0.7
    stepSize = 10.0
    # startpos = (1., 1.)
    # endpos = (5., 5.)
    # # obstacles = [(1., 1.), (2., 2.)]
    # obstacles = []
    # n_iter = 800
    # radius = 0.5
    # stepSize = 0.7
    rrt_obj = RRT(startpos, endpos, n_iter, radius, stepSize, obstacles)
    rrt_graph = rrt_obj.run()

    if rrt_graph.success:
        path = Dijkstra(rrt_graph).run()
        print(path)
        plot(rrt_graph, obstacles, radius, path)
    else:
        plot(rrt_graph, obstacles, radius)

if __name__ == '__main__':
    main()
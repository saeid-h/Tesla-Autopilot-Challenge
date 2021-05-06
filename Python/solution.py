
# https://www.algorithms-and-technologies.com/a_star/python

import numpy as np
EPSILON = 1e-6

class Solution():
    def __init__(self, network, R=6356.752, speed=105, max_distance=320):
        self.SPEED = speed #km/hr
        self.MAX_DISTANCE = max_distance #km
        self.MAX_CHARGE = self.MAX_DISTANCE / self.SPEED #hr
        self.R = R
        self.graph = self.make_graph(network)

    def make_graph(self, network):
        adjancy_matrix = {}
        graph = {}
        for x in network:
            adjancy_matrix.update({x[0]:{}})
            for y in network:
                d = self.geo_distance(x[1:3], y[1:3])
                adjancy_matrix[x[0]].update({y[0]:(d/self.SPEED, x[-1]/self.SPEED)})
        return adjancy_matrix

    def geo_distance(self, P1, P2):
        # http://www.jtrive.com/calculating-distance-between-geographic-coordinate-pairs.html
        phi_1, theta_1 = P1
        phi_2, theta_2 = P2
        phi_1 *= (np.pi/180)
        phi_2 *= (np.pi/180)
        theta_1 *= (np.pi/180)
        theta_2 *= (np.pi/180)
        a = np.sin((phi_1-phi_2)/2) ** 2 + np.cos(phi_1) * np.cos(phi_2) * np.sin((theta_1-theta_2)/2) ** 2
        d = 2 * self.R * np.arcsin(np.sqrt(a))
        return d
    

    def find_path(self):
        self.total_time = 0
        if not self.stops:
            return None
        self.optimize_charging()
        goal = self.stops[-1]
        stops = self.stops[::-1]
        stop = stops.pop() 
        self.path = stop + ', '
        next_stop = stops.pop()
        while next_stop != goal:
            self.total_time += self.graph[stop][next_stop][0]
            stop = next_stop
            next_stop = stops.pop()
            self.path += '{}, {:.5f}, '.format(stop, self.charging_time[stop][next_stop])
            self.total_time += self.charging_time[stop][next_stop]
        self.total_time += self.graph[stop][next_stop][0]
        self.path += next_stop
        return self.path

    def optimize_charging(self):
        stops = self.stops
        n = len(stops)
        if n < 3: return 
        rate = [0] * n
        d = [0] * n
        for i in range(n-1):
            self.charging_time[stops[i]][stops[i+1]] = 0
        for i in range(n):
            rate[i] = self.graph[stops[i]][stops[i]][1]
            if i < n-1:
                d[i] = self.graph[stops[i]][stops[i+1]][0] 
        for i in range(1, n-1):
            t_new = max(0,d[i]-self.battery[stops[i]]) / rate[i] + EPSILON
            self.charging_time[stops[i]][stops[i+1]] += t_new
            self.battery[stops[i+1]] = self.battery[stops[i]] + t_new * rate[i] - d[i]
            if rate[i] > rate[i+1]:
                t_new = min(self.MAX_CHARGE-self.battery[stops[i+1]], max(0,d[i+1]-self.battery[stops[i+1]])) / rate[i] + EPSILON
                self.charging_time[stops[i]][stops[i+1]] += t_new
                self.battery[stops[i+1]] += t_new * rate[i]
                if i+2 < n:
                    self.battery[stops[i+2]] += t_new * rate[i]

    def a_star(self, start, goal):
        travel_time = {key: float("inf") for key in self.graph.keys()}
        travel_time[start] = 0
        priorities = {key: float("inf") for key in self.graph.keys()}
        heuristic = {city: self.graph[city][goal][0] for city in self.graph.keys()}
        priorities[start] = heuristic[start]
        visited = {key: False for key in self.graph.keys()}
        self.comes_before = {}
        self.battery = {key: None for key in self.graph.keys()}
        self.battery[start] = self.MAX_CHARGE
        self.charging_time = {first: {second: None for second in self.graph.keys()} for first in self.graph.keys()}

        while True:
            lowest_priority = float("inf")
            current_city = None
            for city in priorities.keys():
                if priorities[city] < lowest_priority and not visited[city]:
                    lowest_priority = priorities[city]
                    current_city = city
            if current_city is None:
                return None
            elif current_city == goal:
                self.travel_time = travel_time
                self.distance = travel_time[current_city]
                self.priorities = priorities
                self.stops = [goal]
                next_stop = goal
                while next_stop != start:
                    next_stop = self.comes_before[next_stop]
                    self.stops.append(next_stop)
                self.stops = self.stops[::-1]
                self.find_path()
                return self.comes_before

            visited[current_city] = True
            for next_city in self.graph[current_city]:
                dist, rate_1 = self.graph[current_city][next_city]
                if dist < self.MAX_CHARGE and dist != 0 and not visited[next_city]:
                    if travel_time[current_city] + dist < travel_time[next_city]:
                        if self.graph[current_city][goal][0] > self.battery[current_city] and self.battery[current_city] < dist:
                            self.charging_time[current_city][next_city] = (dist-self.battery[current_city]) / rate_1 + EPSILON
                        else:
                            self.charging_time[current_city][next_city] = 0
                        self.battery[next_city] = self.battery[current_city] - dist + self.charging_time[current_city][next_city] * rate_1
                        if travel_time[next_city] > travel_time[current_city] + dist + self.charging_time[current_city][next_city]:
                            travel_time[next_city] = travel_time[current_city] + dist + self.charging_time[current_city][next_city]
                            priorities[next_city] = travel_time[next_city] + heuristic[next_city] 
                            self.comes_before.update({next_city: current_city})
                    visited[current_city] = True


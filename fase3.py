#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys

"""
Created on Sun Mar 29 20:49:06 2020

@author: isegura
"""
"""
Developed by:
- Gonzalo Pato Montemayor, 100429860, group 121
- Gonzalo Prats Juliani, 100429904, group 121
"""


class HealthCenter():
    def __init__(self, name=None):
        self.name = name

    def __eq__(self, other):
        return other is not None and self.name == other.name

    def __str__(self):
        return self.name


class AdjacentVertex:
    def __init__(self, vertex, weight):
        self.vertex = vertex
        self.weight = weight

    def __str__(self):
        return '(' + str(self.vertex) + ',' + str(self.weight) + ')'


class Map():
    def __init__(self):
        self.centers = {}
        self.vertices = {}

    def addHealthCenter(self, center):
        i = len(self.centers)
        self.centers[i] = center
        self.vertices[i] = []

    def _getIndice(self, center):
        for index in self.centers.keys():
            if self.centers[index] == center:
                return index
        return -1

    def __str__(self):
        result = ''
        for i in self.vertices.keys():
            result += str(self.centers[i]) + ':\n'
            for adj in self.vertices[i]:
                result += ('\t' + str(self.centers[adj.vertex])
                           + ', distance:' + str(adj.weight) + '\n')
        return result

    def addConnection(self, center1, center2, distance):
        # print('new conexion:',center1,center2)
        index1 = self._getIndice(center1)
        index2 = self._getIndice(center2)
        if index1 == -1:
            print(center1, ' not found!')
            return
        if index2 == -1:
            print(center2, ' not found!!')
            return
        self.vertices[index1].append(AdjacentVertex(index2, distance))
        # print('adding:',index2,index1,distance)
        self.vertices[index2].append(AdjacentVertex(index1, distance))

    def areConnected(self, center1, center2):
        index1 = self._getIndice(center1)
        index2 = self._getIndice(center2)
        if index1 == -1:
            print(center1, ' not found!')
            return
        if index2 == -1:
            print(center2, ' not found!!')
            return

        for adj in self.vertices[index1]:
            if adj.vertex == index2:
                return adj.weight
        # print(pto1,pto2," no están conectados")
        return 0

    def removeConnection(self, center1, center2):
        index1 = self._getIndice(center1)
        index2 = self._getIndice(center2)
        if index1 == -1:
            print(center1, ' not found!')
            return
        if index2 == -1:
            print(center2, ' not found!!')
            return

        for adj in self.vertices[index1]:
            if adj.vertex == index2:
                self.vertices[index1].remove(adj)
                break

        for adj in self.vertices[index2]:
            if adj.vertex == index1:
                self.vertices[index2].remove(adj)
                break

    def createPath(self):
        """This function prints the vertices by dfs algorithm"""
        # print('dfs traversal:')
        # Mark all the vertices as not visited
        visited = [False] * len(self.vertices)
        paths = []
        for v in self.vertices:
            if visited[v] is False:
                self._dfs(v, visited, paths)

        print()
        return paths

    def _dfs(self, v, visited, paths):
        # Mark the current node as visited and print it
        visited[v] = True
        # print(self.centers[v], end = ' ')
        paths.append(self.centers[v])
        # Recur for all the vertices adjacent to this vertex
        for adj in self.vertices[v]:
            i = adj.vertex
            if visited[i] is False:
                self._dfs(i, visited, paths)

    def printSolution(self, distances, previous, v):
        """imprime los caminos mínimos desde v"""
        for i in range(len(self.vertices)):
            if distances[i] == sys.maxsize:
                print("There is not path from ", v, ' to ', i)
            else:
                minimum_path = []
                prev = previous[i]
                while prev != -1:
                    minimum_path.insert(0, self.centers[prev])
                    prev = previous[prev]

            minimum_path.append(self.centers[i])

            print('Ruta mínima de:', self.centers[v], '->', self.centers[i],
                  ", distance", distances[i], ', ruta: ', end=' ')
            for x in minimum_path:
                print(x, end=' ')
            print()

    def minDistance(self, distances, visited):
        """This functions returns the vertex (index) with the mininum
        distance. To do this, we see in the list distances. We
        only consider the set of vertices that have not been visited"""
        # Initilaize minimum distance for next node
        min = sys.maxsize

        # returns vertex with minimum distance from the non-visited vertices
        for i in range(len(self.vertices)):
            if distances[i] <= min and visited[i] is False:
                min = distances[i]
                min_index = i

        return min_index

    def dijkstra(self, v=0):
        """"This function takes the index of a delivery point pto and
        calculates its mininum path to the rest of vertices by using
        the Dijkstra algoritm. Devuelve una lista con las distancias y una
        lista con los vértices anteriores a uno dado en el camino mínimo"""

        # we use a Python list of boolean to save those nodes
        # that have already been visited
        visited = [False] * len(self.vertices)

        # this list will save the previous vertex
        previous = [-1] * len(self.vertices)

        # This array will save the accumulate distance from v to each node
        distances = [sys.maxsize] * len(self.vertices)
        # The distance from v to itself is 0
        distances[v] = 0

        for i in range(len(self.vertices)):
            # Pick the vertex with the minimum distance vertex.
            # u is always equal to v in first iteration
            u = self.minDistance(distances, visited)
            # Put the minimum distance vertex in the shotest path tree
            visited[u] = True

            # Update distance value of the u's adjacent vertices only if
            # the current distance is greater than new distance and the
            # vertex in not in the shotest path tree
            for adj in self.vertices[u]:
                i = adj.vertex
                w = adj.weight
                if visited[i] is False and distances[i] > distances[u] + w:
                    distances[i] = distances[u] + w
                    previous[i] = u

        # finally, we print the minimum path from v to the other vertices
        # self.printSolution(distances,previous,v)
        return previous, distances

    def minimumPath(self, start, end):
        """calcula la ruta mínima entre dos puntos de entrega"""
        indexStart = self._getIndice(start)
        if indexStart == -1:
            print(str(start) + " does not exist")
            return None
        indexEnd = self._getIndice(end)
        if indexEnd == -1:
            print(str(end) + " does not exist")
            return None

        previous, distances = self.dijkstra(indexStart)

        # construimos el camino mínimo
        minimum_path = []
        prev = previous[indexEnd]
        while prev != -1:
            minimum_path.insert(0, str(self.centers[prev]))
            prev = previous[prev]

        minimum_path.append(str(self.centers[indexEnd]))
        return minimum_path, distances[indexEnd]

    def minimumPathBF(self, start, end):
        """"calcula y devuelve la ruta mínima entre start y end,
        aplicando el algoritmo de Bellman-Ford. Puedes implementar
        otras funciones auxiliares si lo consideras necesario """
        minimum_path = []
        minimumDistance = 0

        # Get start and end indices
        S = self._getIndice(start)
        E = self._getIndice(end)

        # Check if they exist in the list
        if S == -1:
            print("No health center named {} in the map." .format(start))
            return minimum_path, minimumDistance
        if E == -1:
            print("No health center named {} in the map." .format(end))
            return minimum_path, minimumDistance

        print("Finding path from {} to {}" .format(self.centers[S],
                                                   self.centers[E]))

        # Initialize d and p
        d = []
        p = []
        for i in range(len(self.centers)):
            d.append(sys.maxsize)
            p.append(None)
        d[S] = 0  # d[start] = 0

        for i in range(1, len(self.centers)):  # Loop from 1 to |V| - 1
            for u in self.centers.keys():  # For each u
                for v in self.vertices[u]:  # For each (u,v)
                    if d[v.vertex] > d[u] + v.weight:  # Update if necessary
                        d[v.vertex] = d[u] + v.weight
                        p[v.vertex] = u

        for u in self.centers.keys():  # Check convergence
            for v in self.vertices[u]:
                if d[v.vertex] > d[u] + v.weight:
                    return False, 'the algorithm does not converge'

        minimumDistance = d[E]  # minimum distance to the last vertex
        if minimumDistance == sys.maxsize:  # If == inf, there's no path
            # It should not happen, as the graph is undirected
            print("There's no path between {} {}" .format(start, end))
            return minimum_path, minimumDistance
        # Introduce one by one the vertices that create the shortest path
        minimum_path.insert(0, str(end))
        prev = p[E]  # Previous of end
        while prev is not None:
            minimum_path.insert(0, str(self.centers[prev]))  # insert at 0
            prev = p[prev]

        return minimum_path, minimumDistance

        # O(|V||E|) as you have to once through all vertices (|V|-1) and
        # inside the loop 2*all the edges in the graph (for each u, (u,v))
        # while doing simple operations

    def minimumPathFW(self, start, end):
        """"calcula y devuelve la ruta mínima entre start y end,
        aplicando el algoritmo de Floyd-Warshall. Puedes implementar
        otras funciones auxiliares si lo consideras necesario"""
        minimum_path = []
        minimumDistance = 0
        # Get start and end indices
        S = self._getIndice(start)
        E = self._getIndice(end)

        # Check if they exist in the list
        if S == -1:
            print("No health center named {} in the map." .format(start))
            return minimum_path, minimumDistance
        if E == -1:
            print("No health center named {} in the map." .format(end))
            return minimum_path, minimumDistance

        print("Finding path from {} to {}" .format(self.centers[S],
                                                   self.centers[E]))

        # Initialize D and P
        D = [[sys.maxsize for i in range(len(self.centers))]
             for j in range(len(self.centers))]
        P = [[None for i in range(len(self.centers))]
             for j in range(len(self.centers))]

        for i in self.centers.keys():
            D[i][i] = 0  # distance to an element from itself (main diagonal)
            # And now set the distances from one element to another
            for k in self.vertices[i]:
                D[i][k.vertex] = k.weight
                P[i][k.vertex] = i

        for k in range(len(self.centers)):  # loop from 0 to |V| - 1
            for i in range(len(self.centers)):
                for j in range(len(self.centers)):
                    res = D[i][k] + D[k][j]
                    if res < D[i][j]:  # update if necessary
                        D[i][j] = res
                        P[i][j] = P[k][j]

        minimumDistance = D[S][E]  # minimum distance to the last vertex

        if minimumDistance == sys.maxsize:  # If == inf, there's no path
            # It should not happen, as the graph is undirected
            print("There's no path between {} {}" .format(start, end))
            return minimum_path, minimumDistance

        # Introduce one by one the vertices that create the shortest path
        minimum_path.insert(0, str(end))
        prev = P[S][E]  # From start to end through the node prev
        while prev is not None:
            minimum_path.insert(0, str(self.centers[prev]))  # insert at 0
            prev = P[S][prev]  # From start to the previous of end

        return minimum_path, minimumDistance

        # O(|V|^3) since we are using a triple nested loop for the
        # implementation of this function but only simple operations
        # are performed


def test():
    # https://www.bogotobogo.com/python/images/Dijkstra/graph_diagram.png

    m = Map()
    for c in ['A', 'B', 'C', 'D', 'E', 'F']:
        m.addHealthCenter(HealthCenter(c))

    print(m)
    m.addConnection(m.centers[0], m.centers[1], 7)  # A,B,7
    m.addConnection(m.centers[0], m.centers[2], 9)  # A,C,9
    m.addConnection(m.centers[0], m.centers[5], 14)  # A,F,14

    m.addConnection(m.centers[1], m.centers[2], 10)  # B,C,10
    m.addConnection(m.centers[1], m.centers[3], 15)  # B,D,15

    m.addConnection(m.centers[2], m.centers[3], 11)  # C,D,11
    m.addConnection(m.centers[2], m.centers[5], 2)  # C,F,2

    m.addConnection(m.centers[3], m.centers[4], 6)  # D,E,6

    m.addConnection(m.centers[4], m.centers[5], 9)  # E,F,9
    print(m)

    c1 = m.centers[0]
    c2 = m.centers[3]
    print(c1, c2, ' are connected?:', m.areConnected(c1, c2))

    c2 = m.centers[1]
    print(c1, c2, ' are connected?:', m.areConnected(c1, c2))

    m.removeConnection(c1, c2)
    print(m)

    print('createPath:', end=' ')
    ruta = m.createPath()
    # print('Ruta:',ruta)
    for r in ruta:
        print(r, end=' ')
    print()

    minimum_path, d = m.minimumPath(c1, c2)
    for p in minimum_path:
        print(p, end=' ')
    print('total distance:', d)

    # añade más pruebas para probar los dos nuevos
    # métodos minimumPathBF y minimumPathFW
    minimum_path, d = m.minimumPathBF(c1, c2)
    for p in minimum_path:
        print(p, end=' ')
    print('total distance:', d)

    minimum_path, d = m.minimumPathFW(c1, c2)
    for p in minimum_path:
        print(p, end=' ')
    print('total distance:', d)


# Descomenar para usarlo en Spyder
if __name__ == '__main__':
    test()

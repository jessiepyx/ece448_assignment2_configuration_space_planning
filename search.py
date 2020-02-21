# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains search functions.
"""
# Search should return the path and the number of states explored.
# The path should be a list of tuples in the form (alpha, beta, gamma) that correspond
# to the positions of the path taken by your search algorithm.
# Number of states explored should be a number.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,astar)
# You may need to slight change your previous search functions in MP1 since this is 3-d maze

from collections import deque
from heapq import heappop, heappush


def search(maze, searchMethod):
    return {
        "bfs": bfs,
    }.get(searchMethod, [])(maze)


def bfs(maze):
    # Write your code here
    """
    This function returns optimal path in a list, which contains start and objective.
    If no path found, return None. 
    """
    # Copied from MP1 and modified
    # Single objective
    # state = (position, obj_list)
    parent = dict()
    path = []
    pos = maze.getStart()
    objs = maze.getObjectives()
    init_state = (pos, tuple(objs))
    cur_state = init_state
    frontier = deque()
    frontier.append(init_state)
    # visited nodes include frontier
    visited = set()
    visited.add(cur_state)
    while len(frontier) > 0:
        cur_state = frontier.popleft()
        pos = cur_state[0]
        objs_tuple = cur_state[1]
        objs = list(objs_tuple)
        new_objs = objs
        if pos in new_objs:
            break
        neighbors = maze.getNeighbors(pos[0], pos[1], pos[2])
        for new_pos in neighbors:
            new_state = (new_pos, tuple(new_objs))
            if new_state not in visited:
                parent[new_state] = cur_state
                frontier.append(new_state)
                visited.add(new_state)
    while cur_state != init_state:
        path.append(cur_state[0])
        cur_state = parent[cur_state]
    if len(path) == 0:
        return None
    path.append(cur_state[0])
    path.reverse()
    return path

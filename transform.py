# transform.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
# 
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains the transform function that converts the robot arm map
to the maze.
"""
import copy
from arm import Arm
from maze import Maze
from search import *
from geometry import *
from const import *
from util import *


def transformToMaze(arm, goals, obstacles, window, granularity):
    """This function transforms the given 2D map to the maze in MP1.
    
        Args:
            arm (Arm): arm instance
            goals (list): [(x, y, r)] of goals
            obstacles (list): [(x, y, r)] of obstacles
            window (tuple): (width, height) of the window
            granularity (int): unit of increasing/decreasing degree for angles

        Return:
            Maze: the maze instance generated based on input arguments.

    """
    arm_limits = arm.getArmLimit()
    offsets = [i[0] for i in arm_limits]
    # be careful to make sure embedded lists are DEEP copied
    input_map = [[SPACE_CHAR] * int((arm_limits[1][1] - arm_limits[1][0]) / granularity + 1)
                 for _ in range(int((arm_limits[0][1] - arm_limits[0][0]) / granularity + 1))]
    start_x, start_y = angleToIdx(arm.getArmAngle(), offsets, granularity)
    input_map[start_x][start_y] = START_CHAR
    for i in range(len(input_map)):
        for j in range(len(input_map[0])):
            if i != start_x or j != start_y:
                arm.setArmAngle(idxToAngle((i, j), offsets, granularity))
                if doesArmTipTouchGoals(arm.getEnd(), goals):
                    input_map[i][j] = OBJECTIVE_CHAR
                else:
                    arm_links = arm.getArmPosDist()
                    if doesArmTouchObjects(arm_links, goals, True) or \
                            doesArmTouchObjects(arm_links, obstacles, False) or \
                            not isArmWithinWindow(arm.getArmPos(), window):
                        input_map[i][j] = WALL_CHAR
    maze = Maze(input_map, offsets, granularity)
    return maze

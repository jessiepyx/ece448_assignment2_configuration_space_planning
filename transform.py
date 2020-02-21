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
    """This function transforms the given 3D map to the maze in MP1.
    
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
    # handle one, two or three arm links
    num_links = len(arm_limits)
    if num_links == 3:
        input_map = [[[SPACE_CHAR] * int((arm_limits[2][1] - arm_limits[2][0]) / granularity + 1)
                      for _ in range(int((arm_limits[1][1] - arm_limits[1][0]) / granularity + 1))]
                     for _ in range(int((arm_limits[0][1] - arm_limits[0][0]) / granularity + 1))]
        start_x, start_y, start_z = angleToIdx(arm.getArmAngle(), offsets, granularity)
        input_map[start_x][start_y][start_z] = START_CHAR
        for i in range(len(input_map)):
            for j in range(len(input_map[0])):
                for k in range(len(input_map[0][0])):
                    if i != start_x or j != start_y or k != start_z:
                        arm.setArmAngle(idxToAngle((i, j, k), offsets, granularity))
                        arm_links = arm.getArmPosDist()
                        if not isArmWithinWindow(arm.getArmPos(), window) or \
                                doesArmTouchObjects(arm_links, obstacles, False):
                            input_map[i][j][k] = WALL_CHAR
                        elif doesArmTipTouchGoals(arm.getEnd(), goals):
                            input_map[i][j][k] = OBJECTIVE_CHAR
                        elif doesArmTouchObjects(arm_links, goals, True):
                            input_map[i][j][k] = WALL_CHAR
    elif num_links == 2:
        offsets.append(0)
        input_map = [[[SPACE_CHAR] for _ in range(int((arm_limits[1][1] - arm_limits[1][0]) / granularity + 1))]
                     for _ in range(int((arm_limits[0][1] - arm_limits[0][0]) / granularity + 1))]
        start_x, start_y = angleToIdx(arm.getArmAngle(), offsets, granularity)
        input_map[start_x][start_y][0] = START_CHAR
        for i in range(len(input_map)):
            for j in range(len(input_map[0])):
                if i != start_x or j != start_y:
                    arm.setArmAngle(idxToAngle((i, j), offsets, granularity))
                    arm_links = arm.getArmPosDist()
                    if not isArmWithinWindow(arm.getArmPos(), window) or \
                            doesArmTouchObjects(arm_links, obstacles, False):
                        input_map[i][j][0] = WALL_CHAR
                    elif doesArmTipTouchGoals(arm.getEnd(), goals):
                        input_map[i][j][0] = OBJECTIVE_CHAR
                    elif doesArmTouchObjects(arm_links, goals, True):
                        input_map[i][j][0] = WALL_CHAR
    else:
        offsets.append(0)
        offsets.append(0)
        input_map = [[[SPACE_CHAR]] for _ in range(int((arm_limits[0][1] - arm_limits[0][0]) / granularity + 1))]
        start_x, = angleToIdx(arm.getArmAngle(), offsets, granularity)
        input_map[start_x][0][0] = START_CHAR
        for i in range(len(input_map)):
            if i != start_x:
                arm.setArmAngle(idxToAngle((i,), offsets, granularity))
                arm_links = arm.getArmPosDist()
                if not isArmWithinWindow(arm.getArmPos(), window) or \
                        doesArmTouchObjects(arm_links, obstacles, False):
                    input_map[i][0][0] = WALL_CHAR
                elif doesArmTipTouchGoals(arm.getEnd(), goals):
                    input_map[i][0][0] = OBJECTIVE_CHAR
                elif doesArmTouchObjects(arm_links, goals, True):
                    input_map[i][0][0] = WALL_CHAR
    maze = Maze(input_map, offsets, granularity)
    return maze

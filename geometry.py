# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Jongdeog Lee (jlee700@illinois.edu) on 09/12/2018

"""
This file contains geometry functions that relate with Part1 in MP2.
"""

import math
import numpy as np
from const import *


def square_distance_seg2point(start, end, point):
    """Compute the square of distance between a line segment and a point.

        Args:
            start: one end of the segment (x1, y1)
            end: the other end the segment (x2, y2)
            point: (x0, y0)

        Returns:
            float: distance * distance
    """
    x1, y1 = start
    x2, y2 = end
    x0, y0 = point
    # foot of perpendicular (x, y)
    try:
        x = float((y0 - y1) * (y2 - y1) * (x2 - x1) + (x2 - x1) ** 2 * x0 + (y2 - y1) ** 2 * x1) / (
                (x2 - x1) ** 2 + (y2 - y1) ** 2)
    except ZeroDivisionError:
        # line is approximately horizontal
        x = x0
    try:
        y = float((x0 - x1) * (x2 - x1) * (y2 - y1) + (y2 - y1) ** 2 * y0 + (x2 - x1) ** 2 * y1) / (
                (y2 - y1) ** 2 + (x2 - x1) ** 2)
    except ZeroDivisionError:
        # line is approximately vertical
        y = y0
    # if the foot is not within the segment, compute distance to both endpoints instead
    if x <= min(x1, x2) or x >= max(x1, x2) or y <= min(y1, y2) or y >= max(y1, y2):
        return min((x0 - x1) ** 2 + (y0 - y1) ** 2, (x0 - x2) ** 2 + (y0 - y2) ** 2)
    else:
        numerator = ((y0 - y2) * (x1 - x2) - (x0 - x2) * (y1 - y2)) ** 2
        denominator = (y1 - y2) ** 2 + (x1 - x2) ** 2
        return float(numerator) / denominator


def computeCoordinate(start, length, angle):
    """Compute the end coordinate based on the given start position, length and angle.

        Args:
            start (tuple): base of the arm link. (x-coordinate, y-coordinate)
            length (int): length of the arm link
            angle (int): degree of the arm link from x-axis to counter-clockwise

        Return:
            End position (int,int):of the arm link, (x-coordinate, y-coordinate)
    """
    rad = angle * np.pi / 180
    x = start[0] + int(length * np.cos(rad))
    y = start[1] - int(length * np.sin(rad))
    return x, y


def doesArmTouchObjects(armPosDist, objects, isGoal=False):
    """Determine whether the given arm links touch any obstacle or goal

        Args:
            armPosDist (list): start and end position and padding distance of all arm links [(start, end, distance)]
            objects (list): x-, y- coordinate and radius of object (obstacles or goals) [(x, y, r)]
            isGoal (bool): True if the object is a goal and False if the object is an obstacle.
                           When the object is an obstacle, consider padding distance.
                           When the object is a goal, no need to consider padding distance.
        Return:
            True if touched. False if not.
    """
    for arm_link in armPosDist:
        start, end, padding = arm_link
        for obj in objects:
            x, y, r = obj
            if not isGoal:
                r += padding
            if square_distance_seg2point(start, end, (x, y)) <= r ** 2:
                return True
    return False


def doesArmTipTouchGoals(armEnd, goals):
    """Determine whether the given arm tip touch goals

        Args:
            armEnd (tuple): the arm tip position, (x-coordinate, y-coordinate)
            goals (list): x-, y- coordinate and radius of goals [(x, y, r)]. There can be more than one goal.
        Return:
            True if arm tick touches any goal. False if not.
    """
    x0, y0 = armEnd
    for goal in goals:
        x, y, r = goal
        if (x0 - x) ** 2 + (y0 - y) ** 2 <= r ** 2:
            return True
    return False


def isArmWithinWindow(armPos, window):
    """Determine whether the given arm stays in the window

        Args:
            armPos (list): start and end positions of all arm links [(start, end)]
            window (tuple): (width, height) of the window

        Return:
            True if all parts are in the window. False if not.
    """
    for arm_link in armPos:
        start, end = arm_link
        x1, y1 = start
        x2, y2 = end
        w, h = window
        if not 0 < x1 < w or not 0 < x2 < w or not 0 < y1 < h or not 0 < y2 < h:
            return False
    return True


if __name__ == '__main__':
    computeCoordinateParameters = [((150, 190), 100, 20), ((150, 190), 100, 40), ((150, 190), 100, 60),
                                   ((150, 190), 100, 160)]
    resultComputeCoordinate = [(243, 156), (226, 126), (200, 104), (57, 156)]
    testRestuls = [computeCoordinate(start, length, angle) for start, length, angle in computeCoordinateParameters]
    assert testRestuls == resultComputeCoordinate

    testArmPosDists = [((100, 100), (135, 110), 4), ((135, 110), (150, 150), 5)]
    testObstacles = [[(120, 100, 5)], [(110, 110, 20)], [(160, 160, 5)], [(130, 105, 10)]]
    resultDoesArmTouchObjects = [
        True, True, False, True, False, True, False, True,
        False, True, False, True, False, False, False, True
    ]

    testResults = []
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle))

    print("\n")
    for testArmPosDist in testArmPosDists:
        for testObstacle in testObstacles:
            testResults.append(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))
            # print(testArmPosDist)
            # print(doesArmTouchObjects([testArmPosDist], testObstacle, isGoal=True))

    assert resultDoesArmTouchObjects == testResults

    testArmEnds = [(100, 100), (95, 95), (90, 90)]
    testGoal = [(100, 100, 10)]
    resultDoesArmTouchGoals = [True, True, False]

    testResults = [doesArmTipTouchGoals(testArmEnd, testGoal) for testArmEnd in testArmEnds]
    assert resultDoesArmTouchGoals == testResults

    testArmPoss = [((100, 100), (135, 110)), ((135, 110), (150, 150))]
    testWindows = [(160, 130), (130, 170), (200, 200)]
    resultIsArmWithinWindow = [True, False, True, False, False, True]
    testResults = []
    for testArmPos in testArmPoss:
        for testWindow in testWindows:
            testResults.append(isArmWithinWindow([testArmPos], testWindow))
    assert resultIsArmWithinWindow == testResults

    print("Test passed\n")

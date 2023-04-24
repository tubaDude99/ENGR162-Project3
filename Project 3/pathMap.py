# pathMap.py
import turtle
from vector import Vector2

def PathMap(posList, xSize, ySize, width):
    pathMap = [[0 for j in range(ySize)] for i in range(xSize)]
    prevPos = (None,None)
    for i in range(len(posList)):
        x = round(posList[i].x / width) + 1
        y = round(posList[i].y / width) + round(ySize/2)
        if prevPos != (x,y):
            if x >= 0 and y >= 0:
                pathMap[xSize - 1 - x][y] += 1
            prevPos = (x,y)
    return pathMap
def DisplayMap(pathMap):
    for i in range(len(pathMap)):
        for j in range(len(pathMap[0])):
            print(pathMap[i][j], end="")
        print()
def DrawMap(posList, zoom=10):
    Turtle = turtle.Turtle()
    Turtle.ht()
    Turtle.pd()
    for i in range(len(posList)):
        Turtle.goto(posList[i].y*zoom, posList[i].x*zoom)
    Turtle.pu()
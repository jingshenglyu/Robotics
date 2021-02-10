'''
Author: Jingsheng Lyu
Date: 2021-02-09 22:13:08
LastEditors: Jingsheng Lyu
LastEditTime: 2021-02-09 23:20:06
FilePath: /Robotics/AI_for_Robotics/Lesson4_Search/L4.py
Github: https://github.com/jingshenglyu
Web: https://jingshenglyu.github.io/
E-Mail: jingshenglyu@gmail.com
'''
# ----------
# User Instructions:
# 
# Define a function, search() that returns a list
# in the form of [optimal path length, row, col]. For
# the grid shown below, your function should output
# [11, 4, 5].
#
# If there is no valid path from the start point
# to the goal, your function should return the string
# 'fail'
# ----------

# Grid format:
#   0 = Navigable space
#   1 = Occupied space


grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):

    # open list elements are of the type: [g, x, y]
    



    return path


##### Do Not Modify ######

import grader

try:
    response = grader.run_grader(search)
    print(response)    
    
except Exception as err:
    print(str(err))

##### SOLUTION: Run this cell to watch the solution video ######
# from IPython.display import HTML
# HTML('<iframe width="560" height="315" src="https://www.youtube.com/embed/cl8Kdkr4Gbg" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>')
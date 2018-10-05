# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']
delta_init = '-'

# using A star search with manhattan distance
def man_distance(x1, y1):
    d = abs(x1 - goal[0]) + abs(y1 - goal[1])
    return d

def search(grid,init,goal,cost):
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    
    expand = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    h = man_distance(x, y) 
    # state are of type [cost, x, y, [parent_x, parent_y,delta associated]
    open = [[h, x, y, [0, 0, delta_init] ]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand

    while not found and not resign:
        if len(open) == 0:
            resign = True
            return 'fail'
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            # set up expand
            parent_info = next[3]
            if parent_info[2] != '-':
                x_info = parent_info[0]
                y_info = parent_info[1]
                expand[x_info][y_info] = parent_info[2]

            x = next[1]
            y = next[2]
            h = next[0]
            
            if x == goal[0] and y == goal[1]:
                found = True
                expand[x][y] = '*'
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            h2 = man_distance(x2, y2) + 1
                            tmp_grid = [h2, x2, y2, [x, y, delta_name[i]]] 
                            open.append(tmp_grid)
                            closed[x2][y2] = 1

    return expand # make sure you return the shortest path

if __name__ == "__main__":
	print(search(grid, init, goal, cost))

# -----------
# User Instructions:
# 
# Modify the function search so that it returns
# a table of values called expand. This table
# will keep track of which step each node was
# expanded.
#
# Make sure that the initial cell in the grid 
# you return has the value 0.
# ----------

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0],
        [0, 0, 1, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def possible_grids(grid, item, moves, cost, expanded):
    grids = []
    
    for i in range(len(moves)):
        x = item[1] + moves[i][0]
        y = item[2] + moves[i][1]        
        if x >= 0 and x < len(grid) and y >= 0 and y < len(grid[0]):
            if expanded[x][y] == 0 and grid[x][y] == 0:                
                new_grid = [item[0] + cost, x, y]
                grids.append(new_grid)  
                expanded[x][y] = 1
    return grids

def search(grid,init,goal,cost):        
    
    # double array of expanded/marked grids
    expanded = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    expanded[init[0]][init[1]] = 1
    # Information about order of expansion of noes
    expand = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]
    count = 0

    x = init[0]
    y = init[1]
    g = 0
    # open list elements type is [g, x, y]
    open = [[g, x, y]]

    found = False   
    path = []    
    
    while found is False and len(open) != 0:                                            
        # remove node
        open.sort()
        open.reverse()        
        item = open.pop()
        path.append(item)

        if item[1] == goal[0] and item[2] == goal[1]:
            found = True   
            #print(item)
        else:                
            # expand
            expand[item[1]][item[2]] = count
            count += 1

            ex_grids = possible_grids(grid, item, delta, cost, expanded)
            for g in ex_grids:
                open.append(g)
    
    if not found and len(open) == 0:
        print('fail')
    return expand

if __name__ == "__main__":
	print(search(grid, init, goal, cost))
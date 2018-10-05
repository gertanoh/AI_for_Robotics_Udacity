# -------------------
# Background Information
#
# In this problem, you will build a planner that helps a robot
# find the shortest way in a warehouse filled with boxes
# that he has to pick up and deliver to a drop zone.
# 
# For example:
#
# warehouse = [[ 1, 2, 3],
#              [ 0, 0, 0],
#              [ 0, 0, 0]]
# dropzone = [2,0] 
# todo = [2, 1]
# 
# The robot starts at the dropzone.
# The dropzone can be in any free corner of the warehouse map.
# todo is a list of boxes to be picked up and delivered to the dropzone.
#
# Robot can move diagonally, but the cost of a diagonal move is 1.5.
# The cost of moving one step horizontally or vertically is 1.
# So if the dropzone is at [2, 0], the cost to deliver box number 2
# would be 5.

# To pick up a box, the robot has to move into the same cell as the box.
# When the robot picks up a box, that cell becomes passable (marked 0)
# The robot can pick up only one box at a time and once picked up 
# it has to return the box to the dropzone by moving onto the dropzone cell.
# Once the robot has stepped on the dropzone, the box is taken away, 
# and it is free to continue with its todo list.
# Tasks must be executed in the order that they are given in the todo list.
# You may assume that in all warehouse maps, all boxes are
# reachable from beginning (the robot is not boxed in).

# -------------------
# User Instructions
#
# Design a planner (any kind you like, so long as it works!)
# in a function named plan() that takes as input three parameters: 
# warehouse, dropzone, and todo. See parameter info below.
#
# Your function should RETURN the final, accumulated cost to do
# all tasks in the todo list in the given order, which should
# match with our answer. You may include print statements to show 
# the optimum path, but that will have no effect on grading.
#
# Your solution must work for a variety of warehouse layouts and
# any length of todo list.
# 
# Add your code at line 76.
# 
# --------------------
# Parameter Info
#
# warehouse - a grid of values, where 0 means that the cell is passable,
# and a number 1 <= n <= 99 means that box n is located at that cell.
# dropzone - determines the robot's start location and the place to return boxes 
# todo - list of tasks, containing box numbers that have to be picked up
#
# --------------------
# Testing
#
# You may use our test function below, solution_check(),
# to test your code for a variety of input parameters. 

warehouse = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone = [2,0] 
todo = [2, 1]

# ------------------------------------------
# plan - Returns cost to take all boxes in the todo list to dropzone
#
# ----------------------------------------
# modify code below
# ----------------------------------------
delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ], # go right
		 [-1, -1], # diagonal up left
		 [+1, -1],  # diagonal down left
		 [-1, +1], # diagonal up right
		 [1, 1]]   # diagonal up down

# using A star search with diagonal distance
D1_cost = 1 	# cost of horizontal or vertical move 
D2_cost = 1.5 	# cost of diagonal move
costs = [1, 1, 1, 1, 1.5, 1.5, 1.5, 1.5]

def heuristic(point, goal):
	dx = abs(point[0] - goal[0]) 
	dy = abs(point[1] - goal[1])
	
	return D1_cost * (dx + dy) + (D2_cost - 2 * D1_cost) * min(dx, dy)		 
 
# open_list is 2D
def find_element(open_list, x, y):
	index = -1
	for i in range(len(open_list)):
		if open_list[i][2] == x and open_list[i][3] == y:
			index = i
			break
	
	return index
 

def search(grid,init,goal):
	
	ret = 0.0
	
	closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
	closed[init[0]][init[1]] = 1

	
	x = init[0]
	y = init[1]
	h = heuristic(init, goal) 
	
	# state are of type [cost, x, y]
	open = [[h, 0, x, y]]	
	

	found = False  # flag that is set when search is complete
	resign = False # flag set if we can't find expand

	# mark the goal as passable so the robot can move into 
	grid[goal[0]][goal[1]] = 0
	while not found and not resign:
		if len(open) == 0:
			resign = True
			return 'fail'
		else:
			open.sort()
			open.reverse()
			next = open.pop()
						
			# set up expand            
			x = next[2]
			y = next[3]
			g = next[1]
					
			if x == goal[0] and y == goal[1]:
				found = True
				ret = g
			else:
				for i in range(len(delta)):
					x2 = x + delta[i][0]
					y2 = y + delta[i][1]
					
					if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
						if grid[x2][y2] == 0:																				
							if closed[x2][y2] == 0:
								g2 = g + costs[i]
								h2 = heuristic([x2, y2], goal) + g2
								tmp_grid = [h2, g2, x2, y2] 
								open.append(tmp_grid)
								closed[x2][y2] = 1							
							# index is closed but a shorter path exists
							elif closed[x2][y2] == 1 and len(open) != 0:																
								index = find_element(open, x2, y2)
								if (heuristic([x2, y2], goal) + g + costs[i]) < open[index][0]:
									g2 = g + costs[i]
									h2 = heuristic([x2, y2], goal) + g2
									open[index][0] = h2
									open[index][1] = g2
								
				
	return ret
def goal_coord(warehouse, goal):	
	coord = []
	for i in range(len(warehouse)):
		for j in range(len(warehouse[0])):
			if warehouse[i][j] == goal:
				coord.append(i)
				coord.append(j)	
	return coord

def plan(warehouse, dropzone, todo):    
	cost = 0.0
	# mark it as passable
	warehouse[dropzone[0]][dropzone[1]] = 0
	
	for td in todo:	
		g_point = goal_coord(warehouse, td)
		cost += search(warehouse, dropzone, g_point)
		cost += search(warehouse, g_point, dropzone)						
		
	return cost



################# TESTING ##################
       
# ------------------------------------------
# solution check - Checks your plan function using
# data from list called test[]. Uncomment the call
# to solution_check to test your code.
#
def solution_check(test, epsilon = 0.00001):
    answer_list = []
    
    import time
    start = time.clock()
    correct_answers = 0
    for i in range(len(test[0])):
        user_cost = plan(test[0][i], test[1][i], test[2][i])
        true_cost = test[3][i]
        if abs(user_cost - true_cost) < epsilon:
            print("\nTest case", i+1, "passed!")
            answer_list.append(1)
            correct_answers += 1
            #print "#############################################"
        else:
            print( "\nTest case ", i+1, "unsuccessful. Your answer ", user_cost, "was not within ", epsilon, "of ", true_cost )
            answer_list.append(0)
    runtime =  time.clock() - start
    if runtime > 1:
        print ("Your code is too slow, try to optimize it! Running time was: ", runtime)
        return False
    if correct_answers == len(answer_list):
        print( "\nYou passed all test cases!")
        return True
    else:
        print ("\nYou passed", correct_answers, "of", len(answer_list), "test cases. Try to get them all!")
        return False
#Testing environment
# Test Case 1 
warehouse1 = [[ 1, 2, 3],
             [ 0, 0, 0],
             [ 0, 0, 0]]
dropzone1 = [2,0] 
todo1 = [2, 1]
true_cost1 = 9
# Test Case 2
warehouse2 = [[   1, 2, 3, 4],
              [   0, 0, 0, 0],
              [   5, 6, 7, 0],
              [ 'x', 0, 0, 8]] 
dropzone2 = [3,0] 
todo2 = [2, 5, 1]
true_cost2 = 21

# Test Case 3
warehouse3 = [[   1, 2,  3,  4, 5, 6,  7],
              [   0, 0,  0,  0, 0, 0,  0],
              [   8, 9, 10, 11, 0, 0,  0],
              [ 'x', 0,  0,  0, 0, 0, 12]] 
dropzone3 = [3,0] 
todo3 = [5, 10]
true_cost3 = 18

# Test Case 4
warehouse4 = [[ 1, 17, 5, 18,  9, 19,  13],
              [ 2,  0, 6,  0, 10,  0,  14],
              [ 3,  0, 7,  0, 11,  0,  15],
              [ 4,  0, 8,  0, 12,  0,  16],
              [ 0,  0, 0,  0,  0,  0, 'x']] 
dropzone4 = [4,6]
todo4 = [13, 11, 6, 17]
true_cost4 = 41

testing_suite = [[warehouse1, warehouse2, warehouse3, warehouse4],
                 [dropzone1, dropzone2, dropzone3, dropzone4],
                 [todo1, todo2, todo3, todo4],
                 [true_cost1, true_cost2, true_cost3, true_cost4]]


#if __name__ == "__main__":
#	print("cost to goal :", plan(warehouse1, dropzone1, todo1))
#	print("cost to goal :", plan(warehouse2, dropzone2, todo2))
#	print("cost to goal :", plan(warehouse3, dropzone3, todo3))
#	print("cost to goal :", plan(warehouse4, dropzone4, todo4))	
solution_check(testing_suite) #UNCOMMENT THIS LINE TO TEST YOU CODE
 
#Given the list motions=[1,1] which means the robot 
#moves right and then right again, compute the posterior 
#distribution if the robot first senses red, then moves 
#right one, then senses green, then moves right again, 
#starting with a uniform prior distribution.

p=[0.2, 0.2, 0.2, 0.2, 0.2]
world=['green', 'green', 'red', 'green', 'red']
measurements = ['red']
motions = [1]
pHit = 0.9
pMiss = 0.1

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for i in range(len(q)):
        q[i] = q[i] / s
    return q

def sensing_red_after_one_move(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)    
    return s
	
def move(p, U):
	q = []
	q.append(0)
	for i in range(len(p) - 1):				
		s =  p[i]
		if i == len(p) - 2:
			s += p[i+1]
		q.append(s)
	return q

# Motion and sense
if __name__ == "__main__":
	
	p = sense(p, measurements[0])	
	p = move(p, motions[0])	
	s = sensing_red_after_one_move(p, measurements[0])
	print (s)         


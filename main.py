from AstarSearch import AStarSearch as AStar
import numpy as np

#10x10 grid
grid = [[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0],
		[0,0,0,0,0,0,0,0,0,0]]

start = [0,0]
goal = [9,9]

astar = AStar(grid, start, goal)

while True:
	astar.find_moves()
	if astar.find_goal():
		break
	astar.select_move()

depth_path = astar.depth_path
heuristic_value_grid = astar.heuristicGrid


#Now to find the best path, we follow the position depths in ascending order
#Only move to positions with a position depth that is one higher than current position
def get_path(pos, path, depth_path):
	#mark current pos as part of the path
	path[pos[0]][pos[1]] = 1
	curr_depth = depth_path[pos[0]][pos[1]]

	#Find the possible moves
	possible_moves = find_moves(pos)

	print(f"possible moves: {possible_moves}")

	#Iterate through all the possible moves
	for move in possible_moves:
		#Check if move is valid and within boundaries
		if astar.bound_check(move):
			possible_depth = depth_path[move[0]][move[1]]
			#check if move position depth is one higher than current position depth
			if possible_depth <= curr_depth and possible_depth != 0:
				#That move is our new position and depth
				pos = move
				curr_depth = possible_depth
				#Reiterate process
				get_path(pos, path, depth_path)

	return path

def find_moves(pos):
	up = [pos[0] - 1, pos[1]]
	down = [pos[0] + 1, pos[1]]
	left = [pos[0], pos[1] - 1]
	right = [pos[0], pos[1] + 1]

	topright = [pos[0] - 1, pos[1] + 1]
	topleft = [pos[0] - 1, pos[1] - 1]
	bottomright = [pos[0] + 1, pos[1] + 1]
	bottomleft = [pos[0] - 1, pos[1] - 1]

	possible_moves = [up, down, left, right, topright, topleft, bottomright, bottomleft]

	return possible_moves

depth_path[start[0]][start[1]] = 99
#list of path from start to goal
path = np.zeros([len(grid), len(grid)], dtype=int)
path = get_path(astar.start, path, depth_path)
path[start[0]][start[1]] = 99

print(f'Path Depth \n {depth_path}\n')
print(f'Heuristic Grid \n {heuristic_value_grid}\n')
print(f'Path \n {path}\n')

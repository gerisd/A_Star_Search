from BFS import BreadthFirstSearch as BFS
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


bfs = BFS(grid, start, goal)

while True:
	bfs.find_moves()
	if bfs.find_goal():
		break
	bfs.select_move()

depth_path = bfs.depth_path

#Now to find the best path, we follow the position depths in ascending order
#Only move to positions with a position depth that is one higher than current position
def get_path(pos, path, depth_path):
	#mark current pos as part of the path
	path[pos[0]][pos[1]] = 1
	curr_depth = depth_path[pos[0]][pos[1]]

	#Find the possible moves
	possible_moves = find_moves(pos)

	#Iterate through all the possible moves
	for move in possible_moves:
		#Check if move is valid and within boundaries
		if bfs.bound_check(move):
			possible_depth = depth_path[move[0]][move[1]]
			#check if move position depth is one higher than current position depth
			if curr_depth == possible_depth - 1:
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

	possible_moves = [up, down, left, right]

	return possible_moves


#list of path from start to goal
path = np.zeros([len(grid), len(grid)], dtype=int)
path = get_path(bfs.start, path, depth_path)
path[start[0]][start[1]] = 99

print(f'Path Depth \n {depth_path}')
print(f'\nPath \n {path}')

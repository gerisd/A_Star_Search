import numpy as np
import ast

class AStarSearch:

	def __init__(self, grid, start, goal):
		self.grid = grid
		self.goal = goal
		self.start = start
		self.pos = start
		self.pos_depth = 0

		#list of visited nodes
		self.visited = []

		#key: str(position on grid), value: move depth in grid  
		self.not_visited = {}

		#list of depth for each visited node
		self.depth_path = np.zeros([len(grid), len(grid)], dtype=int)

		#Heuristic value chart
		self.heuristicGrid = np.zeros([len(grid), len(grid)], dtype=int)


	#Finds all possible moves in every linear direction (top, bottom, left and right of current position)
	#And determines which of those moves haven't been visited yet
	def find_moves(self):
		up = [self.pos[0] - 1, self.pos[1]]
		down = [self.pos[0] + 1, self.pos[1]]
		left = [self.pos[0], self.pos[1] - 1]
		right = [self.pos[0], self.pos[1] + 1] 

		topright = [self.pos[0] - 1, self.pos[1] + 1]
		topleft = [self.pos[0] - 1, self.pos[1] - 1]
		bottomright = [self.pos[0] + 1, self.pos[1] + 1]
		bottomleft = [self.pos[0] - 1, self.pos[1] - 1]

		possible_moves = [up, down, left, right, topright, topleft, bottomright, bottomleft]

		#iterate through all the possible moves and determine which are valid and haven't been visited yet
		for move in possible_moves:
			#check if the move is valid
			if self.bound_check(move):
				#check if the move has been visited 
				if (str(move) not in self.visited) and (str(move) not in self.not_visited):
					#Adding move to list of not_visited nodes and marking its depth within the grid
					self.not_visited[str(move)] = self.pos_depth + 1 + self.heuristic(move)

					#Heuristic Grid
					self.heuristicGrid[move[0]][move[1]] = self.pos_depth + 1 + self.heuristic(move)

		#current position has been explored
		self.visited.append(str(self.pos)) 

	#select which move to go to next from the non visited dictionary
	def select_move(self):

		#Find smallest position depth, it is the next node to visit
		not_visited_sorted = sorted(self.not_visited, key=self.not_visited.get, reverse=False)

		curr_pos = not_visited_sorted[0]

		#convert pos string back to list 
		self.pos = self.deconvert_list_string(curr_pos)
		curr_total_value = self.not_visited.pop(curr_pos)
		#self.pos_depth = self.not_visited.pop(curr_pos) - self.heuristic(self.pos) #to get positional depth rather than heuristic depth
		self.pos_depth = curr_total_value - (curr_total_value - self.heuristic(self.pos)) #get heuristic depth rather than positional depth

		self.depth_path[self.pos[0], self.pos[1]] = round(self.pos_depth)

	#Determines if the move is valid and within the bounds of the grid
	def bound_check(self, move):
		#Check to see if the move is within the bounds of the grid
		if (move[0] < 0) or (move[0] >= len(self.grid)):
			return False
		if (move[1] < 0) or (move[1] >= len(self.grid)):
			return False

		#check to see if the possible move is not a wall or obstacle
		if self.grid[move[0]][move[1]] == 1:
			return False
		return True

	#Checks to see if goal is within reach - hence within the updated non visted list 
	def find_goal(self):
		if str(self.goal) in self.not_visited:
			self.pos = self.goal
			self.pos_depth = self.not_visited.pop(str(self.pos))
			self.depth_path[self.pos[0], self.pos[1]] = self.pos_depth
			return True

		return False

	def heuristic(self, position):
		position_np = np.array(position)
		goal_np = np.array(self.goal)
		
		euclidean_distance = position_np - goal_np
		euclidean_distance = euclidean_distance * euclidean_distance
		euclidean_distance = np.sqrt(sum(euclidean_distance))

		return round(euclidean_distance)

	#convert string of list back to list
	def deconvert_list_string(self, strg):
		val = ast.literal_eval(strg)
		#val = [x.strip() for x in val]
		return val

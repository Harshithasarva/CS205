import copy
import time

#creates a class object for the state node
class Node:
  def __init__(self, node):
    self.state = node
    self.children = [] #stores child nodes
    self.parent = None # To keep track of the parent nodes to print the solution path
    self.heuristic_cost = 0 #h(n) - heuristic value based on the selection
    self.depth = 0 #g(n) - this code uses depth as the cost to expand a node
    self.fscore = 0 #f(n) - g(n)+h(n)

#returns heuristic cost using misplaced tiles from the goal state
def misplaced_tiles(state):
  goal_state = [[1,2,3],[4,5,6],[7,8,0]]
  misplaced_tiles_count = 0
  for i in range(0,len(state)):
    for j in range(0, len(state)):
      if state[i][j] != goal_state[i][i] and state[i][j]!=0 : 
        misplaced_tiles_count += 1
  return misplaced_tiles_count

#returns heuristic value using manhattan distance from the goal state
#manhattan distance is calculated using (x1, y1) and (x2, y2) is |x1 - x2| + |y1 - y2|
def manhattan_distance(curr_state):
   goal_state = [[1,2,3],[4,5,6],[7,8,0]]
   man_distance = 0
   pos_in_goal = dict() #stores the positions of tiles in the goal state 
   pos_in_curr = dict() #stores the positions of tiles in the current state argument 

   #positions of the tiles in the goal state - hardcoded as it is fixed 
   pos_in_goal = {1: [0, 0], 2: [0, 1], 3: [0, 2], 4: [1, 0], 5: [1, 1], 6: [1, 2], 7: [2, 0], 8: [2, 1], 0: [2, 2]} 
   
   #OR
   #positions of tiles can be found using iteration as well - in case of a 15-puzzle
   for r in range(0, len(goal_state)):
     for c in range(0, len(goal_state)):
       if goal_state[r][c] not in pos_in_goal:
         pos_in_goal[goal_state[r][c]] = [r, c]

   #iterating through the curr_state to store the indices
   for r in range(0, len(curr_state)):
     for c in range(0, len(curr_state)):
       if curr_state[r][c] not in pos_in_curr:
         pos_in_curr[curr_state[r][c]] = [r, c]

   for i in pos_in_goal.keys():
     man_distance += abs(pos_in_goal[i][0] - pos_in_curr[i][0])
     man_distance += abs(pos_in_goal[i][1] - pos_in_curr[i][1])
   return man_distance

#locates the position of zero, returns row and col which is used in expand nodes method
def locateZero(state):
  zero_row = 0
  zero_col = 0
  for i in range(0, len(state)):
    for j in range(0, len(state)):
      if state[i][j] == 0:
        zero_row = i
        zero_col = j
  return zero_row, zero_col

#checks if the state is goal state and return True if it is
def check_goal(state):
  goal_state = [[1,2,3],[4,5,6],[7,8,0]]
  for i in range(0, len(state)):
    for j in range(0, len(state)):
      if state[i][j] != goal_state[i][j]:
        return False
  return True

#generates the child nodes of curr_node by moving the blank/zero LEFT, RIGHT, UP, DOWN
def expand_nodes(curr_node, visited):
  r, c = locateZero(curr_node.state)
  child_list = [] #stores the children of the curr_node 
  
  #move left
  if c <= 2 and c != 0 :
    left_move = copy.deepcopy(curr_node.state)
    left_move[r][c], left_move[r][c-1] = left_move[r][c-1], left_move[r][c]
    if left_move not in visited: #checking if the node generated is already visited
      child_list.append(Node(left_move))
  
  # move right
  if c >=0 and c != 2 :
    right_move = copy.deepcopy(curr_node.state)
    right_move[r][c], right_move[r][c+1] = right_move[r][c+1], right_move[r][c]
    if right_move not in visited:
      child_list.append(Node(right_move))

  #move up
  if r <= 2 and r != 0:
    up_move = copy.deepcopy(curr_node.state)
    up_move[r][c], up_move[r-1][c] = up_move[r-1][c], up_move[r][c]
    if up_move not in visited:
      child_list.append(Node(up_move))

  #move down
  if r >= 0 and r != 2:
    down_move = copy.deepcopy(curr_node.state)
    down_move[r][c], down_move[r+1][c] = down_move[r+1][c], down_move[r][c]
    if down_move not in visited:
      child_list.append(Node(down_move))
      
  curr_node.children = child_list #updating the children of curr_node 
  return curr_node

#prints the solution path if the goal state is achieved
def print_solution_path(node):
  print_list = [] 
  print("\nPrinting the solution path......\n")
  while node.parent: #iterating through the parent nodes to find the solution path
    sol =  "Node State: "+str(node.state) + " Depth:"+ str(node.depth) 
    node = node.parent
    print_list.append(sol)

  temp = "Node State: "+str(node.state) + " Depth:"+ str(node.depth)
  print_list.append(temp)

  for i in range(len(print_list)-1,0,-1):
    print(print_list[i])
    print(" || ")
    print(" || ")
    print(" || ")
  print(print_list[0] + " ---> GOAL STATE")


#General Search algorithm implementation 
def general_search(state, alg_choice):
  print("starting the search.... \n")

  #records the start time of the algorithm
  start_time = time.time()

  #creates a node object for the initial state
  initial_node = Node(state)

  heuristic = 0 #default value
  #updates the heuristic cost of the node, h(n) in UCS = 0
  if alg_choice == 2:
    heuristic = misplaced_tiles(state)
    
  elif alg_choice == 3:
    heuristic = manhattan_distance(state)
    
  #initial_node.heuristic_cost = heuristic
  initial_node.heuristic_cost = heuristic
  initial_node.fscore = heuristic
  
  dq = [initial_node] #initializes the queue with the initial_node
  visited = [initial_node.state] #keeps track of the visited nodes
  curr_queue_size = 1 #current size of the queue
  max_queue_size = 1 #maximum queue size during the trace
  nodes_expanded = 1 #to track the total nodes expanded
  
  while True:
    #If the queue is empty - no nodes to expand
    if len(dq) == 0:
      print("Failure, PUZZLE CANNOT BE SOLVED")
      return 

    #sort the queue in the ascending order of the the fscore
    dq = sorted(dq, key=lambda fn: fn.fscore)
    curr_node = dq.pop(0) #pop the cheapest estimated cost node
    curr_queue_size -= 1 
    
    #checks for the goal state and returns the parameters of the path
    if check_goal(curr_node.state):
      print("Goal State Found, PUZZLE SOLVED")
      print("\nPrinting the solution details:\n ")
      end_time = time.time()
      print("- Time taken to find the goal state", round((end_time - start_time),  6), "seconds")
      print("- Depth of the solution is found to be ", curr_node.depth)
      print("- Number of nodes expanded during the search are", nodes_expanded)
      print("- Maximum queue size at any given time is", max_queue_size)
      print_solution_path(curr_node)
      return 

    #curr_node updated with its child nodes
    expanded_curr = expand_nodes(curr_node, visited)

    #updates the queue by adding the children of current node and it's cost parameters
    for child in expanded_curr.children:
      child.parent = curr_node
      child.depth = curr_node.depth + 1

      if alg_choice == 2:
        heuristic = misplaced_tiles(child.state)
      elif alg_choice == 3:
        heuristic = manhattan_distance(child.state)
      
      child.heuristic_cost = heuristic
      child.fscore = child.depth + child.heuristic_cost

      dq.append(child)
      curr_queue_size += 1
      nodes_expanded += 1 
      visited.append(child.state)

    if curr_queue_size > max_queue_size:
      max_queue_size = curr_queue_size
     

def main():
  print("Welcome to the 8-puzzle solver \n")
  input_choice = int(input("Enter 1 if you want to choose the default puzzle \n Enter 2 if you want to enter a new puzzle \n"))
  if input_choice == 1:
    input_puzzle = [[1,3,6], [ 5,0,7], [4,8,2]] #set the default puzzle
    
  elif input_choice == 2:
    print("Enter your puzzle here, use a zero to represent the blank in the puzzle \n")
    row1 = list(map(int, input("Enter row1 with spaces between numbers").split()))
    row2 = list(map(int, input("Enter row2 with spaces between numbers").split()))
    row3 = list(map(int, input("Enter row3 with spaces between numbers").split()))
    input_puzzle = [row1, row2, row3]
    #print("Entered input puzzle is", input_puzzle)
  else:
    print("Invalid Choice, Select 1 or 2 again \n")

  print("Input puzzle is", input_puzzle)
  
  #Choosing the Algorithm:
  input_alg = int(input("\nChoose an algorithm to solve the puzzle: \n 1. Uniform Search Cost \n 2. A* Algorithm with misplaces tiles \n 3. A* Algorithm with manhattan distance \n"))

  #calling the general search algorithm to implement the puzzle
  general_search(input_puzzle, input_alg)

if __name__ == "__main__":
    main()

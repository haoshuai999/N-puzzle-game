
from __future__ import division
from __future__ import print_function
from collections import deque

import sys
import math
import time
import heapq
# if sys.platform == "win32":
#     import psutil
#     print("psutil", psutil.Process().memory_info().rss)
# else:
#     # Note: if you execute Python from cygwin,
#     # the sys.platform is "cygwin"
#     # the grading system's sys.platform is "linux2"
#     import resource
#     print("resource", resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)


#### SKELETON CODE ####
## The Class that Represents the Puzzle
class PuzzleState(object):
    """
        The PuzzleState stores a board configuration and implements
        movement instructions to generate valid children.
    """
    def __eq__(self, other):
        return tuple(self.config) == other

    def __hash__(self):
        return hash(tuple(self.config))

    def __contains__(self, other):
        return tuple(self.config) in other

    def __lt__(self, other):
        return tuple(self.config) > tuple(self.config)

    def __init__(self, config, n, parent=None, action="Initial", cost=0):
        """
        :param config->List : Represents the n*n board, for e.g. [0,1,2,3,4,5,6,7,8] represents the goal state.
        :param n->int : Size of the board
        :param parent->PuzzleState
        :param action->string
        :param cost->int
        """
        if n*n != len(config) or n < 2:
            raise Exception("The length of config is not correct!")
        if set(config) != set(range(n*n)):
            raise Exception("Config contains invalid/duplicate entries : ", config)

        self.n        = n
        self.cost     = cost
        self.parent   = parent
        self.action   = action
        self.config   = config
        self.children = []

        # Get the index and (row, col) of empty block
        self.blank_index = self.config.index(0)

    def display(self):
        """ Display this Puzzle state as a n*n board """
        for i in range(self.n):
            print(self.config[3*i : 3*(i+1)])

    def move_up(self):
        """
        Moves the blank tile one row up.
        :return a PuzzleState with the new configuration
        """
        temp_config = self.config[:]
        if self.blank_index > self.n - 1:
            temp_config[self.blank_index] = temp_config[self.blank_index - 3]
            temp_config[self.blank_index - 3] = 0
            cost = self.cost + 1
            return PuzzleState(temp_config, self.n, parent= self, action= "Up", cost = cost)
        return None

    def move_down(self):
        """
        Moves the blank tile one row down.
        :return a PuzzleState with the new configuration
        """
        temp_config = self.config[:]
        if self.blank_index < self.n * self.n - self.n:
            temp_config[self.blank_index] = temp_config[self.blank_index + 3]
            temp_config[self.blank_index + 3] = 0
            cost = self.cost + 1
            return PuzzleState(temp_config, self.n, parent= self, action= "Down", cost = cost)
        return None

    def move_left(self):
        """
        Moves the blank tile one column to the left.
        :return a PuzzleState with the new configuration
        """
        temp_config = self.config[:]
        if self.blank_index % self.n != 0:
            temp_config[self.blank_index] = temp_config[self.blank_index - 1]
            temp_config[self.blank_index - 1] = 0
            cost = self.cost + 1
            return PuzzleState(temp_config, self.n, parent= self, action= "Left", cost = cost)
        return None

    def move_right(self):
        """
        Moves the blank tile one column to the right.
        :return a PuzzleState with the new configuration
        """
        temp_config = self.config[:]
        if (self.blank_index + 1) % self.n != 0:
            temp_config[self.blank_index] = temp_config[self.blank_index + 1]
            temp_config[self.blank_index + 1] = 0
            cost = self.cost + 1
            return PuzzleState(temp_config, self.n, parent= self, action= "Right", cost = cost)
        return None

    def expand(self):
        """ Generate the child nodes of this node """

        # Node has already been expanded
        if len(self.children) != 0:
            return self.children

        # Add child nodes in order of UDLR
        children = [
            self.move_up(),
            self.move_down(),
            self.move_left(),
            self.move_right()]

        # Compose self.children of all non-None children states
        self.children = [state for state in children if state is not None]
        return self.children

# Function that Writes to output.txt

### Students need to change the method to have the corresponding parameters
def writeOutput(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth ):
    ### Student Code Goes here
    path_to_goal = str(path_to_goal)
    cost_of_path = str(cost_of_path)
    nodes_expanded =str(nodes_expanded)
    search_depth = str(search_depth)
    max_search_depth = str(max_search_depth)
    with open("output.txt", 'w') as f:
        f.write("path_to_goal:" + path_to_goal + "\n")
        f.write("cost_of_path:" + cost_of_path + "\n")
        f.write("nodes_expanded:" + nodes_expanded + "\n")
        f.write("search_depth:" + search_depth + "\n")
        f.write("max_search_depth:" + max_search_depth + "\n")
    f.close()

def bfs_search(initial_state):
    """BFS search"""
    ### STUDENT CODE GOES HERE ###

    frontier = deque([initial_state])
    frontier_save = set()
    frontier_save.add(tuple(initial_state.config))
    explored = set()
    path_to_goal = []
    nodes_expanded = 0
    cost_of_path = 0
    max_search_depth = 0

    while frontier:
        state = frontier.popleft()
        frontier_save.remove(tuple(state.config))
        explored.add(tuple(state.config))

        # if test_goal(state):
        #     search_depth = state.cost
        #     while state.parent:
        #         path_to_goal.append(state.action)
        #         state = state.parent
        #         cost_of_path = cost_of_path + 1
        #     print("path_to_goal:", list(reversed(path_to_goal)))
        #     print("cost_of_path:", cost_of_path)
        #     print("nodes_expanded:", nodes_expanded)
        #     print("search_depth:", search_depth)
        #     print("max_search_depth:", max_search_depth)
        
        #     break

        state.children = state.expand()

        for child in state.children:

            if child not in frontier_save and child not in explored:
                max_search_depth = child.cost + 1
                nodes_expanded = nodes_expanded + 1
                # print(child.config)
                # print(frontier_save)
                # print(child not in frontier_save)
                # print(child not in explored)
                frontier.append(child)
                frontier_save.add(tuple(child.config))

                if test_goal(child):
                    search_depth = child.cost
                    while child.parent:
                        path_to_goal.append(child.action)
                        child = child.parent
                        cost_of_path = cost_of_path + 1
                    path_to_goal = path_to_goal[::-1]
                    print("path_to_goal:", path_to_goal)
                    print("cost_of_path:", cost_of_path)
                    print("nodes_expanded:", nodes_expanded)
                    print("search_depth:", search_depth)
                    print("max_search_depth:", max_search_depth)
                    writeOutput(path_to_goal,cost_of_path,nodes_expanded,search_depth,max_search_depth)
                    return

    return None

def dfs_search(initial_state):
    """DFS search"""
    ### STUDENT CODE GOES HERE ###
    frontier = [initial_state]
    frontier_save = set()
    frontier_save.add(tuple(initial_state.config))
    explored = set()
    path_to_goal = []
    nodes_expanded = 0
    cost_of_path = 0
    max_search_depth = 0

    while frontier:
        state = frontier.pop()
        if state not in explored:
	        frontier_save.remove(tuple(state.config))
	        nodes_expanded = nodes_expanded + 1
	        explored.add(tuple(state.config))

	        # if test_goal(state):
	        #     search_depth = state.cost
	        #     while state.parent:
	        #         path_to_goal.append(state.action)
	        #         state = state.parent
	        #         cost_of_path = cost_of_path + 1
	        #     path_to_goal = list(reversed(path_to_goal))
	        #     print("path_to_goal:", path_to_goal)
	        #     print("cost_of_path:", cost_of_path)
	        #     print("nodes_expanded:", nodes_expanded - 1)
	        #     print("search_depth:", search_depth)
	        #     print("max_search_depth:", max_search_depth)
	        #     writeOutput(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth)
	        #     break

	        state.children = state.expand()[::-1] 

	        for child in state.children:

	            #print(child.config)
	            #print(child not in explored)
	            if child not in frontier_save and child not in explored:
	                max_search_depth = child.cost
	                
	                frontier.append(child)
	                frontier_save.add(tuple(child.config))

	                if test_goal(child):
	                    search_depth = child.cost
	                    while child.parent:
	                        path_to_goal.append(child.action)
	                        child = child.parent
	                        cost_of_path = cost_of_path + 1
	                    path_to_goal = path_to_goal[::-1]
	                    print("path_to_goal:", path_to_goal)
	                    print("cost_of_path:", cost_of_path)
	                    print("nodes_expanded:", nodes_expanded - 1)
	                    print("search_depth:", search_depth)
	                    print("max_search_depth:", max_search_depth)
	                    writeOutput(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth)
	                    return


    return None

def A_star_search(initial_state):
    """A * search"""
    ### STUDENT CODE GOES HERE ###
    frontier = []
    heapq.heappush(frontier, (calculate_total_cost(initial_state), initial_state))
    frontier_save = {}
    frontier_save[initial_state] = calculate_total_cost(initial_state)
    explored = set()
    path_to_goal = []
    nodes_expanded = 0
    cost_of_path = 0
    max_search_depth = 0

    while frontier:
        full_state = heapq.heappop(frontier)
        state = full_state[1]
        explored.add(tuple(state.config))
        nodes_expanded = nodes_expanded + 1

        # if test_goal(state):
        #     search_depth = state.cost
        #     while state.parent:
        #         path_to_goal.append(state.action)
        #         state = state.parent
        #         cost_of_path = cost_of_path + 1
        #     print("path_to_goal:", list(reversed(path_to_goal)))
        #     print("cost_of_path:", cost_of_path)
        #     print("nodes_expanded:", nodes_expanded)
        #     print("search_depth:", search_depth)
        #     print("max_search_depth:", max_search_depth)
        #
        #     break

        state.children = state.expand()

        for child in state.children:
            if child not in frontier_save and child not in explored:
                max_search_depth = child.cost

                heapq.heappush(frontier, (calculate_total_cost(child), child))
                frontier_save[child] = calculate_total_cost(child)

            elif child in frontier_save:
                if calculate_total_cost(child) < frontier_save[child]:
                    heapq.heappush(frontier, (calculate_total_cost(child), child))

                    frontier_save[child] = calculate_total_cost(child)

            if test_goal(child):
                search_depth = child.cost
                while child.parent:
                    path_to_goal.append(child.action)
                    child = child.parent
                    cost_of_path = cost_of_path + 1
                path_to_goal = path_to_goal[::-1]
                print("path_to_goal:", path_to_goal)
                print("cost_of_path:", cost_of_path)
                print("nodes_expanded:", nodes_expanded)
                print("search_depth:", search_depth)
                print("max_search_depth:", max_search_depth)
                writeOutput(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth)
                return

    return None


def calculate_total_cost(state):
    """calculate the total estimated cost of a state"""
    ### STUDENT CODE GOES HERE ###
    total_cost = state.cost
    i = 8
    while i > 0:
        total_cost = total_cost + calculate_manhattan_dist(state.config.index(i), i, state.n)
        i = i - 1
    return total_cost

def calculate_manhattan_dist(idx, value, n):
    """calculate the manhattan distance of a tile"""
    ### STUDENT CODE GOES HERE ###
    distance = abs(idx - value)
    mandistance = (distance % n) + (distance // n)
    return mandistance

def test_goal(puzzle_state):
    """test the state is the goal state or not"""
    ### STUDENT CODE GOES HERE ###
    #print(puzzle_state.cost)
    goal_state = []
    for i in range(puzzle_state.n * puzzle_state.n):
    	goal_state.append(i)
    if puzzle_state.config == goal_state:
        return True
    else:
        return False

# Main Function that reads in Input and Runs corresponding Algorithm
def main():
    search_mode = sys.argv[1].lower()
    begin_state = sys.argv[2].split(",")
    begin_state = list(map(int, begin_state))
    board_size  = int(math.sqrt(len(begin_state)))
    hard_state  = PuzzleState(begin_state, board_size)
    start_time  = time.time()

    if   search_mode == "bfs": bfs_search(hard_state)
    elif search_mode == "dfs": dfs_search(hard_state)
    elif search_mode == "ast": A_star_search(hard_state)
    else:
        print("Enter valid command arguments !")

    end_time = time.time()
    print("running time: %.8f"%(end_time-start_time))


if __name__ == '__main__':
    main()
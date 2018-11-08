# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
	
	"""
	Use of depth first search algorithm to traverse the problem as a tree.
	The objective is to find the fastest way to reach the goal.
	Save the steps needed to reach the goal and return them.
	The main problem with DFS algorithm is that if the tree or problem is 
	too large (meaning that the tree has a high depth), the algorithm is
	unefficient.
	"""
	
    #init variables
    stack = util.Stack()
    visited = []
    initialState = problem.getStartState()

    #push the starting point into stack
    stack.push((initialState,[], 0))

    while not stack.isEmpty(): 
        (node,direction, cost) = stack.pop() #get the node

        if(problem.isGoalState(node)): #finish the execution if goal is found
        	return direction

        if(not node in visited):    #if the node is not visited
        	visited.append(node)   #add it to the visited list
	        for child in problem.getSuccessors(node):  #for all childs of the node do a push
	               stack.push((child[0],direction + [child[1]], cost + child[2])) 

    return direction

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    #init variables
    stack = util.Queue()
    visited = []
    initialState = problem.getStartState()

    #push the starting point into stack
    stack.push((initialState,[], 0))

    while not stack.isEmpty(): 
        (node,direction, cost) = stack.pop() #get the node

        if(problem.isGoalState(node)): #finish the execution if goal is found
        	return direction

        if(not node in visited):    #if the node is not visited
        	visited.append(node)   #add it to the visited list
	        for child in problem.getSuccessors(node):  #for all childs of the node do a push
	               stack.push((child[0],direction + [child[1]], cost + child[2])) 

    return direction

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    
    """
    Using Priority Queue as the data structure for the function let us traverse
    the problem as a tree in the same way as DFS and BFS but taking into
    account the cost to reach the goal. Since the cost is uniform, the use
    of Priority Queue is good enough to find the node of least cost because 
    cost only takes into account the distance between the position and the goal
    and it does not take into account the fact that some paths can have 
    more cost than others depending on other factors. 
    """
    
    #init variables
    pQueue = util.PriorityQueue()
    visited = []
    cost = 0
    #push the starting point
    initialState = (problem.getStartState(), [],cost)
    pQueue.push(initialState,cost)

    while not pQueue.isEmpty(): #do it while the queue isn't empty
        node = pQueue.pop()

        if problem.isGoalState(node[0]):
            return node[1]

        if not node[0] in visited:  #if the node is not visited
            visited.append(node[0]) #add it to the visited list
            for child in problem.getSuccessors(node[0]):    #get new node and add it to the queue
                newNode = (child[0], node[1] + [child[1]], node[2] + child[2])
                pQueue.push(newNode,node[2] + child[2])

    return node[1]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    #init variables
    pQueue = util.PriorityQueue()
    visited = []

    gCost = 0
    hCost = heuristic(problem.getStartState(), problem)
    fCost = gCost + hCost
    #push the starting point
    initialState = (problem.getStartState(), [],gCost)
    pQueue.push(initialState,fCost)

    while not pQueue.isEmpty(): #do it while the queue isn't empty
        node = pQueue.pop()

        if problem.isGoalState(node[0]):
            return node[1]

        if not node[0] in visited:  #if the node is not visited
            visited.append(node[0]) #add it to the visited list
            for child in problem.getSuccessors(node[0]):    #get heuristic values for new node and push it to the queue
            	new_gCost = node[2] + child[2]
            	new_hCost = heuristic(child[0], problem)
            	new_fCost = new_gCost + new_hCost
                newNode = (child[0], node[1] + [child[1]], new_gCost)
                pQueue.push(newNode,new_fCost)

    return node[1]


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
